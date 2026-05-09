#!/usr/bin/env python3
"""
fed_server.py — FedVibroSense Aggregation Server for Raspberry Pi
=================================================================
Receives local model weights from STM32F405 node(s) over UART,
performs Federated Averaging (FedAvg), and broadcasts the updated
global model back to all clients.

Hardware:
  Raspberry Pi UART:  /dev/ttyAMA0  (GPIO14=TX, GPIO15=RX)
  Baud rate:          921600
  Wire:  RPi GPIO14 (TX, Pin 8)  → STM32 PA10 (UART1 RX)
         RPi GPIO15 (RX, Pin 10) ← STM32 PA9  (UART1 TX)
         RPi GND    (Pin 6)       — STM32 GND

Setup on Raspberry Pi:
  1. sudo raspi-config → Interface Options → Serial Port
       "Would you like a login shell over serial?" → NO
       "Would you like serial port hardware enabled?" → YES
  2. Reboot
  3. pip3 install pyserial numpy

Usage:
  # Single STM32 node, run forever:
  python3 fed_server.py

  # Three STM32 nodes, 50 rounds, save models:
  python3 fed_server.py --port /dev/ttyAMA0 --clients 3 --rounds 50 --save-models

  # Use USB-UART adapter instead of GPIO UART:
  python3 fed_server.py --port /dev/ttyUSB0 --clients 1

Author:  FedVibroSense Project
Version: 1.1
"""

import serial
import struct
import numpy as np
import argparse
import time
import os
import sys
import csv
import logging

# ─── Protocol constants (must match Config.h) ────────────────────────────────
MAGIC_UPLOAD    = 0xFE01
MAGIC_DOWNLOAD  = 0xFE02
PROTOCOL_VER    = 0x01
FL_WEIGHT_COUNT = 8067          # NN_INPUT*NN_HIDDEN + NN_HIDDEN + NN_HIDDEN*NN_OUT + NN_OUT
                                 # = 500*16 + 16 + 16*3 + 3 = 8067
HEADER_SIZE     = 5             # magic(2) + ver(1) + nweights(2)
CRC_SIZE        = 2
PAYLOAD_BYTES   = FL_WEIGHT_COUNT * 4
PACKET_SIZE     = HEADER_SIZE + PAYLOAD_BYTES + CRC_SIZE   # 32275 bytes

ACK_BYTES       = bytes([0xAC, 0xAC])
NACK_BYTES      = bytes([0x0A, 0xCE])

# ─── Logging setup ────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-5s  %(message)s",
    datefmt="%H:%M:%S",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler("fed_server.log"),
    ],
)
log = logging.getLogger("FedServer")

# ─── CRC-16/CCITT ─────────────────────────────────────────────────────────────
def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT — must match Utils_CRC16() on STM32."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

# ─── Packet receive ───────────────────────────────────────────────────────────
def receive_packet(ser: serial.Serial, client_id: int) -> np.ndarray:
    """
    Block until a complete FL upload packet is received from the STM32.

    Returns a NumPy float32 array of FL_WEIGHT_COUNT elements:
      [0:8000]   W1 (500×16 row-major)
      [8000:8016] b1 (16)
      [8016:8064] W2 (16×3 row-major)
      [8064:8067] b2 (3)

    Raises RuntimeError on any validation failure.
    """
    log.info(f"  [Client {client_id}] Waiting for packet ({PACKET_SIZE} bytes)...")
    t0 = time.time()
    raw = ser.read(PACKET_SIZE)
    elapsed = time.time() - t0

    if len(raw) != PACKET_SIZE:
        raise RuntimeError(
            f"[Client {client_id}] Incomplete packet: got {len(raw)} of {PACKET_SIZE} bytes "
            f"after {elapsed:.1f}s — check baud rate and UART wiring"
        )

    # ── Parse header ────────────────────────────────────────────────────────
    magic    = struct.unpack_from('<H', raw, 0)[0]
    version  = raw[2]
    n_weights = struct.unpack_from('<H', raw, 3)[0]

    if magic != MAGIC_UPLOAD:
        raise RuntimeError(
            f"[Client {client_id}] Bad magic: got {magic:#06x}, expected {MAGIC_UPLOAD:#06x}"
        )
    if version != PROTOCOL_VER:
        raise RuntimeError(
            f"[Client {client_id}] Protocol version mismatch: {version} vs {PROTOCOL_VER}"
        )
    if n_weights != FL_WEIGHT_COUNT:
        raise RuntimeError(
            f"[Client {client_id}] Weight count mismatch: {n_weights} vs {FL_WEIGHT_COUNT}"
        )

    # ── CRC validation ───────────────────────────────────────────────────────
    rx_crc   = struct.unpack_from('<H', raw, HEADER_SIZE + PAYLOAD_BYTES)[0]
    calc_crc = crc16_ccitt(raw[:HEADER_SIZE + PAYLOAD_BYTES])
    if rx_crc != calc_crc:
        raise RuntimeError(
            f"[Client {client_id}] CRC mismatch: received {rx_crc:#06x}, computed {calc_crc:#06x}"
        )

    # ── Extract float32 weights ──────────────────────────────────────────────
    payload = raw[HEADER_SIZE : HEADER_SIZE + PAYLOAD_BYTES]
    weights = np.frombuffer(payload, dtype=np.float32).copy()

    log.info(
        f"  [Client {client_id}] Packet OK — {elapsed:.2f}s | "
        f"CRC={calc_crc:#06x} | W1_mean={weights[:8000].mean():.6f}"
    )
    return weights

# ─── Packet send ──────────────────────────────────────────────────────────────
def send_packet(ser: serial.Serial, weights: np.ndarray, client_id: int):
    """
    Serialize and transmit the global model to one client.
    Mirrors SER_SerializeWeights() on the STM32.
    """
    header  = struct.pack('<HBH', MAGIC_DOWNLOAD, PROTOCOL_VER, FL_WEIGHT_COUNT)
    payload = weights.astype(np.float32).tobytes()
    crc_input = header + payload
    crc       = crc16_ccitt(crc_input)
    packet    = crc_input + struct.pack('<H', crc)

    assert len(packet) == PACKET_SIZE, f"Packet size mismatch: {len(packet)}"

    t0 = time.time()
    ser.write(packet)
    ser.flush()
    elapsed = time.time() - t0

    log.info(
        f"  [Client {client_id}] Global model sent — {elapsed:.2f}s | "
        f"CRC={crc:#06x} | W1_mean={weights[:8000].mean():.6f}"
    )

# ─── FedAvg ───────────────────────────────────────────────────────────────────
def fedavg(weight_list: list) -> np.ndarray:
    """
    Simple FedAvg: element-wise mean across all client weight vectors.

    For weighted FedAvg (weighting by number of local samples),
    replace np.mean with a weighted average using client sample counts.
    This implementation uses equal weighting (standard for same-size datasets).
    """
    stacked = np.stack(weight_list, axis=0)   # shape: (N_clients, FL_WEIGHT_COUNT)
    avg     = np.mean(stacked, axis=0)
    return avg

# ─── CSV logger ───────────────────────────────────────────────────────────────
class RoundLogger:
    """Saves per-round aggregation statistics to a CSV for publication plots."""

    def __init__(self, path: str):
        self.path = path
        with open(path, 'w', newline='') as f:
            csv.writer(f).writerow([
                'round', 'n_clients', 'w1_mean_global', 'w1_std_global',
                'w1_min_global', 'w1_max_global', 'weight_divergence_norm',
                'timestamp'
            ])
        log.info(f"Round log: {path}")

    def log(self, rnd: int, n_clients: int, global_w: np.ndarray,
            client_weights: list):
        w1 = global_w[:8000]
        # Weight divergence: mean L2 distance from each client to global
        divergences = [
            float(np.linalg.norm(cw - global_w)) for cw in client_weights
        ]
        mean_divergence = float(np.mean(divergences))

        with open(self.path, 'a', newline='') as f:
            csv.writer(f).writerow([
                rnd, n_clients,
                float(w1.mean()), float(w1.std()),
                float(w1.min()),  float(w1.max()),
                mean_divergence,
                time.strftime("%Y-%m-%d %H:%M:%S")
            ])

# ─── MAIN ─────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="FedVibroSense Aggregation Server",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument('--port',        default='/dev/ttyAMA0',
                        help='UART serial port')
    parser.add_argument('--baud',        default=921600, type=int,
                        help='Baud rate — must match UART_FL_BAUD in Config.h')
    parser.add_argument('--clients',     default=1, type=int,
                        help='Number of STM32 nodes per round')
    parser.add_argument('--rounds',      default=0, type=int,
                        help='FL rounds to run (0 = run forever)')
    parser.add_argument('--save-models', action='store_true',
                        help='Save global model .npy file after each round')
    parser.add_argument('--models-dir',  default='models',
                        help='Directory to save model snapshots')
    parser.add_argument('--log-csv',     default='fl_rounds.csv',
                        help='CSV file for per-round statistics')
    args = parser.parse_args()

    # ── Setup ────────────────────────────────────────────────────────────────
    if args.save_models:
        os.makedirs(args.models_dir, exist_ok=True)
        log.info(f"Model snapshots → {args.models_dir}/")

    round_logger = RoundLogger(args.log_csv)

    log.info("=" * 60)
    log.info("  FedVibroSense Aggregation Server  v1.1")
    log.info("=" * 60)
    log.info(f"  Port:    {args.port} @ {args.baud} baud")
    log.info(f"  Clients: {args.clients} per round")
    log.info(f"  Rounds:  {'∞' if args.rounds == 0 else args.rounds}")
    log.info(f"  Weights: {FL_WEIGHT_COUNT} float32 = {PACKET_SIZE} bytes/packet")
    log.info(f"  Est. transfer time per packet: "
             f"{PACKET_SIZE * 10 / args.baud * 1000:.0f} ms")
    log.info("=" * 60)

    # ── Open serial port ─────────────────────────────────────────────────────
    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=60,        # 60 s receive timeout per packet
            write_timeout=15,  # 15 s transmit timeout
        )
        log.info(f"Serial port {args.port} opened successfully")
    except serial.SerialException as e:
        log.error(f"Cannot open {args.port}: {e}")
        log.error("Tip: check  ls /dev/tty*  and  sudo raspi-config → Serial Port")
        sys.exit(1)

    # ── Main loop ────────────────────────────────────────────────────────────
    global_weights = None   # None until first round
    rnd = 0

    try:
        while True:
            rnd += 1
            if args.rounds > 0 and rnd > args.rounds:
                break

            log.info("")
            log.info(f"{'='*20}  FL Round {rnd}  {'='*20}")
            round_start = time.time()
            client_weights = []

            # ── Receive from each client ──────────────────────────────────
            for c in range(1, args.clients + 1):
                try:
                    weights = receive_packet(ser, c)
                    client_weights.append(weights)

                    # Send ACK immediately after successful receive
                    ser.write(ACK_BYTES)
                    ser.flush()
                    log.info(f"  [Client {c}] ACK sent")

                except RuntimeError as e:
                    log.error(f"  [Client {c}] Receive error: {e}")
                    # Send NACK so STM32 goes to ERROR state cleanly
                    ser.write(NACK_BYTES)
                    ser.flush()
                    log.warning(f"  [Client {c}] NACK sent — client will reset and retry")
                    # Skip this round if any client fails
                    client_weights = []
                    break

            if len(client_weights) != args.clients:
                log.warning(f"Round {rnd}: incomplete ({len(client_weights)}/{args.clients} clients) — skipping FedAvg")
                continue

            # ── FedAvg aggregation ────────────────────────────────────────
            global_weights = fedavg(client_weights)
            log.info(
                f"  FedAvg done — "
                f"W1_global: mean={global_weights[:8000].mean():.6f}  "
                f"std={global_weights[:8000].std():.6f}"
            )

            # ── Send global model back to all clients ─────────────────────
            for c in range(1, args.clients + 1):
                try:
                    send_packet(ser, global_weights, c)
                except serial.SerialTimeoutException as e:
                    log.error(f"  [Client {c}] TX timeout: {e}")

            # ── Logging & saving ──────────────────────────────────────────
            round_logger.log(rnd, args.clients, global_weights, client_weights)

            if args.save_models:
                model_path = os.path.join(args.models_dir, f"global_round_{rnd:04d}.npy")
                np.save(model_path, global_weights)
                log.info(f"  Model saved → {model_path}")

            elapsed = time.time() - round_start
            log.info(f"  Round {rnd} complete in {elapsed:.1f}s")

    except KeyboardInterrupt:
        log.info("\nServer stopped by user (Ctrl+C)")
    finally:
        ser.close()
        log.info(f"Serial port closed. Completed {rnd - 1} rounds.")
        if global_weights is not None:
            final_path = "final_global_model.npy"
            np.save(final_path, global_weights)
            log.info(f"Final global model saved → {final_path}")

if __name__ == '__main__':
    main()
