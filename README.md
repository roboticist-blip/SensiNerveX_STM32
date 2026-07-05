# SensiNerveX — STM32F405 Federated Learning Vibration Client

[![Platform](https://img.shields.io/badge/Platform-STM32F405RGT6-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32f405rg.html)
[![Architecture](https://img.shields.io/badge/Architecture-ARM%20Cortex--M4F-green)](https://developer.arm.com/Processors/Cortex-M4)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)
[![Build](https://img.shields.io/badge/Build-arm--none--eabi--gcc-orange)](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)

> **Research-grade embedded Federated Learning client for vibration anomaly detection.**  
> Fully on-device training, inference, and FL weight exchange on a 168 MHz Cortex-M4F — no external ML frameworks.

---

## Table of Contents

1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Hardware Requirements](#hardware-requirements)
4. [Project Structure](#project-structure)
5. [Memory Budget](#memory-budget)
6. [Quick Start](#quick-start)
7. [Build Instructions](#build-instructions)
8. [Flash Instructions](#flash-instructions)
9. [STM32CubeMX Setup](#stm32cubemx-setup)
10. [Runtime Operation](#runtime-operation)
11. [Federated Learning Protocol](#federated-learning-protocol)
12. [UART Command Interface](#uart-command-interface)
13. [Aggregation Server (Raspberry Pi)](#aggregation-server-raspberry-pi)
14. [Future Roadmap](#future-roadmap)
15. [Research Notes](#research-notes)
16. [Troubleshooting](#troubleshooting)

---

## Overview

FedVibroSense implements a complete **TinyML + Federated Learning** pipeline on the STM32F405RGT6 microcontroller. Each node:

- Samples an MPU-6050 IMU at **100 Hz**
- Fuses accelerometer and gyroscope data with a **complementary filter**
- Accumulates 1-second rolling windows into a **500-element feature vector**
- Trains a **hand-coded feedforward neural network** (500→16→3) using backpropagation
- Classifies vibration into three states: **Normal / Imbalance / Looseness**
- Serializes model weights and participates in **FedAvg** rounds over UART

All computation is performed with **zero dynamic memory allocation**, deterministic timing, and full **float32 FPU** utilization on the Cortex-M4F.

### Why no TensorFlow Lite Micro?

This project deliberately implements the neural network from scratch to:
- Maintain full control over memory layout and gradient flow
- Eliminate external framework dependencies for research reproducibility
- Enable custom federated learning weight serialization
- Demonstrate that embedded backpropagation is practical on Cortex-M4F
- Produce a self-contained research artifact suitable for IEEE publication

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    STM32F405RGT6 @ 168 MHz                   │
│                                                              │
│  MPU-6050 (I2C1, 400 kHz)                                   │
│       │                                                      │
│       ▼  100 Hz (TIM3 interrupt)                             │
│  ┌─────────────┐     ┌──────────────────┐                    │
│  │  MPU6050.c  │────▶│ ComplementaryFilter│                   │
│  │  Raw 6-DOF  │     │ pitch/roll/mag    │                   │
│  └─────────────┘     └────────┬─────────┘                    │
│                               │                              │
│                               ▼                              │
│                    ┌──────────────────┐                      │
│                    │ FeatureExtractor │                       │
│                    │ Ring buffer 128  │                       │
│                    │ Z-score 500-vec  │                       │
│                    └────────┬─────────┘                      │
│                             │  every 1 s                     │
│                             ▼                                │
│                    ┌──────────────────┐                      │
│                    │  NeuralNetwork   │                       │
│                    │  500→16→3 MLP    │                       │
│                    │  ReLU+Softmax    │                       │
│                    │  SGD Backprop    │                       │
│                    └────────┬─────────┘                      │
│                             │                                │
│           ┌─────────────────┴───────────────────┐            │
│           │         FederatedClient              │            │
│           │  IDLE→COLLECT→TRAIN→UPLOAD→DOWNLOAD  │           │
│           │  FedAvg weight serialization          │           │
│           └─────────────────┬───────────────────┘            │
│                             │                                │
│  UART1 (921600 baud) ◀──────┘──────▶ UART1                  │
└─────────────────────────────────────────────────────────────┘
         │                                      │
         ▼                                      ▼
  Raspberry Pi                          Raspberry Pi
  Aggregation Server                    (FedAvg, future)
  (Python FedAvg)
```

### Neural Network Topology

```
Input Layer      Hidden Layer       Output Layer
[500 neurons] →  [16 neurons]   →   [3 neurons]
   Linear           ReLU            Softmax

W1: 500×16 = 8000 params    Cross-Entropy Loss
b1:      16 params           SGD (lr=0.01)
W2:  16×3 =   48 params      Xavier Init
b2:       3 params
─────────────────────
Total:     8067 params = 31.5 KB (float32)
```

---

## Hardware Requirements

### Primary MCU Board
**WeAct Studio STM32F405RGT6 core board** (primary target for SDIO/SD
logging — pins below assume this board's onboard SDIO pinout). Also
tested without SD logging on:
- **STM32F4 Discovery** (STM32F407 variant — minor pinout adjustments needed)
- **Nucleo-F405RG** (direct pin mapping)
- Custom PCB with STM32F405RGT6

### Sensor
- **InvenSense MPU-6050** breakout module (GY-521 or equivalent)

### Storage
- microSD card, Class 10 or better, FAT32-formatted (or blank — `FF_USE_MKFS`
  is enabled so a first-boot format is possible, though pre-formatting is
  recommended)

### Wiring

| Signal       | STM32 Pin | MPU-6050 Pin | Notes                    |
|-------------|-----------|--------------|--------------------------|
| I2C1_SCL    | PB8       | SCL          | 4.7 kΩ pull-up to 3.3 V |
| I2C1_SDA    | PB9       | SDA          | 4.7 kΩ pull-up to 3.3 V |
| INT         | PA0       | INT          | Active low, pull-up      |
| 3.3 V       | 3.3 V     | VCC          | 100 nF decoupling cap    |
| GND         | GND       | GND          |                          |
| AD0         | GND       | AD0          | I2C address = 0x68       |

### SD Card (SDIO, 4-bit wide bus)

| Signal    | STM32 Pin | microSD Pin | Notes                          |
|-----------|-----------|-------------|---------------------------------|
| SDIO_D0   | PC8       | DAT0        | 10 kΩ pull-up to 3.3 V (usually on socket) |
| SDIO_D1   | PC9       | DAT1        | 10 kΩ pull-up to 3.3 V          |
| SDIO_D2   | PC10      | DAT2        | 10 kΩ pull-up to 3.3 V          |
| SDIO_D3   | PC11      | DAT3/CD     | 10 kΩ pull-up to 3.3 V          |
| SDIO_CK   | PC12      | CLK         | No pull needed (driven by MCU)  |
| SDIO_CMD  | PD2       | CMD         | 10 kΩ pull-up to 3.3 V          |
| 3.3 V     | 3.3 V     | VDD         |                                 |
| GND       | GND       | VSS         |                                 |

> Pin assignments are fixed by the STM32F405's SDIO peripheral — they
> cannot be remapped to other GPIOs. See `SDCard.h` for the full rationale.


### Debug / Programming
| Signal | STM32 Pin | Host Side      |
|--------|-----------|----------------|
| TX     | PA2       | USB-UART RX    |
| RX     | PA3       | USB-UART TX    |
| SWDIO  | PA13      | ST-Link / J-Link |
| SWDCLK | PA14      | ST-Link / J-Link |

### FL Communication (to Raspberry Pi / Server)
| Signal | STM32 Pin | Raspberry Pi Pin |
|--------|-----------|------------------|
| TX     | PA9       | RXD (GPIO15, Pin 10) |
| RX     | PA10      | TXD (GPIO14, Pin 8)  |
| GND    | GND       | GND (Pin 6)          |

> **Voltage warning**: STM32F405 GPIO is 3.3 V. Raspberry Pi GPIO is also 3.3 V — direct connection is safe. If using a 5 V UART adapter, use a level shifter.

---

## Project Structure

```
FedVibroSense_STM32/
│
├── Core/
│   ├── Inc/
│   │   ├── Config.h              ← All compile-time parameters (edit here)
│   │   ├── Utils.h               ← Debug macros, math helpers, CRC-16
│   │   ├── MPU6050.h             ← IMU driver API
│   │   ├── ComplementaryFilter.h ← Pitch/roll fusion API
│   │   ├── FeatureExtractor.h    ← Ring buffer + feature vector API
│   │   ├── NeuralNetwork.h       ← NN training/inference API
│   │   ├── FederatedClient.h     ← FL FSM API
│   │   ├── Serialization.h       ← Binary weight packet API
│   │   ├── AppModule.h           ← Module-table scheduler interface (NEW)
│   │   ├── SDCard.h              ← SDIO block driver API (NEW)
│   │   ├── DataLogger.h          ← SD/FatFs logging service API (NEW)
│   │   ├── diskio.h              ← FatFs disk-I/O interface (NEW)
│   │   └── ffconf.h              ← FatFs configuration (NEW)
│   │
│   └── Src/
│       ├── main.c                ← App_Init + AppModule-driven main loop
│       ├── MPU6050.c             ← I2C driver, calibration, burst read
│       ├── ComplementaryFilter.c ← alpha filter, angle estimation
│       ├── FeatureExtractor.c    ← Ring buffer, Z-score normalization
│       ├── NeuralNetwork.c       ← Forward/backward/SGD from scratch
│       ├── FederatedClient.c     ← FL state machine, UART comms
│       ├── Serialization.c       ← Binary pack/unpack + CRC-16
│       ├── Utils.c               ← UART log, LCG PRNG, timer, mem stats
│       ├── AppModule.c           ← Fixed-size module table impl (NEW)
│       ├── SDCard.c              ← SDIO/HAL_SD block driver (NEW)
│       ├── diskio.c              ← FatFs ↔ SDCard.c glue (NEW)
│       └── DataLogger.c          ← Non-blocking SD CSV logger (NEW)
│
├── Middlewares/Third_Party/FatFs/src/   ← NOT included — add separately
│   ├── ff.c / ff.h / ffunicode.c        ← See Core/Inc/ffconf.h header comment
│
├── STM32CubeMX/
│   ├── STM32F405RGTx_FLASH.ld   ← Linker script (Flash + SRAM + CCM)
│   └── CubeMX_Config_Guide.txt  ← Exact CubeMX peripheral settings (incl. SDIO)
│
├── cmake/
│   └── arm-none-eabi.cmake       ← CMake cross-compilation toolchain file
│
├── Makefile                      ← Primary terminal build system
├── CMakeLists.txt                ← CMake alternative (CLion/VS Code)
└── README.md                     ← This file
```

### Auto-Generated by CubeMX (do not edit)
```
Core/Src/stm32f4xx_hal_msp.c     ← GPIO/UART/I2C alt-function MSP init
Core/Src/stm32f4xx_it.c          ← IRQ handler stubs
Core/Src/system_stm32f4xx.c      ← SystemInit()
Core/Inc/main.h                  ← HAL handle externs
Core/Inc/stm32f4xx_hal_conf.h    ← HAL module enables (HAL_SD_MODULE_ENABLED now on)
Drivers/                         ← Full HAL + CMSIS (never edit)
```

---

## SD Card Logging (SDIO + FatFs)

Every inference window and every completed FL round is now persisted to
the microSD card as CSV, in addition to the existing UART debug log —
useful for offline analysis (loss curves, confusion matrices, inference
latency distributions) without needing a PC connected during a run.

**Hardware**: WeAct Studio STM32F405RGT6's SDIO peripheral, 4-bit wide bus.
Pins are fixed by silicon — see `SDCard.h` for the full map (PC8-11, PC12,
PD2). External 10 kΩ pull-ups on CMD/D0-D3 (most microSD sockets already
have these).

**Software layers** (bottom to top — each is independently swappable):

```
main.c → DataLogger.c → diskio.c (FatFs glue) → SDCard.c → HAL_SD → SDIO peripheral
                ↑
         ff.c (FatFs core — third-party, add separately)
```

- **`SDCard.c`** — thin, storage-agnostic block API (`SDCard_ReadBlocks`/
  `WriteBlocks`/`Init`) wrapping `HAL_SD`. Nothing above it knows HAL_SD
  exists.
- **`diskio.c`** — the standard FatFs `disk_read`/`disk_write`/`disk_ioctl`
  contract, translated to `SDCard.c` calls. This is the *only* file that
  bridges FatFs and our SDIO driver.
- **`DataLogger.c`** — the module everything else actually talks to.
  Owns session-file naming (`LOGS/SNX_00001.CSV`, auto-incrementing so
  reboots never clobber a previous run), periodic flush policy, and
  mount-failure retry. Critically, it fronts the SD card with a small RAM
  ring buffer (`SD_LOG_QUEUE_DEPTH`, Config.h) so the 100 Hz IMU loop and
  FL FSM never block on a slow card — `DataLogger_LogWindow()`/
  `LogFLRound()` just enqueue a formatted row (µs-scale) and
  `DataLogger_ModuleTick()` (run once per main-loop iteration) drains at
  most one row per call.

**Getting it building**: `ff.c`/`ff.h`/`ffunicode.c` (the FatFs core
engine) are third-party middleware, not part of the ST HAL package, so
they aren't in this repo. Add them either via STM32CubeMX (add the
"FATFS" middleware, choose a user-defined custom disk driver so it
doesn't overwrite `diskio.c`) or by downloading FatFs directly and
dropping the three files into `Middlewares/Third_Party/FatFs/src/`. See
the header comment in `Core/Inc/ffconf.h` for both paths. Until then,
`SD_LOGGING_ENABLE` in `Config.h` can be set to `0` to build everything
else — `DataLogger.c` compiles down to no-op stubs in that mode.

**New UART commands**: `l` prints logger stats (rows written/dropped,
write errors, remounts); `f` forces an immediate flush.

---

## Scalability Refactor (AppModule)

The original main loop hand-sequenced every subsystem inline. As soon as
SD logging needed to be added, that structure would have meant editing
the loop body again — and would again for the next addition (a second
sensor, a wireless transport, ...). `AppModule.h`/`.c` replaces that with
a fixed-size table of `{name, init, tick}` entries (see the header for
the full rationale). `main()`'s loop body is now:

```c
while (1) {
    AppModule_TickAll();
}
```

Each existing subsystem got a thin wrapper (`Mod_ImuSample_Tick`,
`Mod_FeatureWindow_Tick`, `Mod_FLC_Tick`, `Mod_UartCommand_Tick` in
`main.c`) so registration order still matches the original, timing-aware
sequence (IMU sampling first, UART commands last). `DataLogger` is a
first-class module in the same table — adding it required zero changes
to the loop itself, which is the point: the next new subsystem gets the
same treatment, one `Init`/`Tick` pair and one `AppModule_Register()`
call, and the loop stays exactly this small.

---

## Memory Budget

| Region   | Total   | Used (approx)  | Contents                          |
|----------|---------|----------------|-----------------------------------|
| Flash    | 1024 KB | ~65 KB         | Code + HAL + SDIO/FatFs + libm stubs |
| SRAM1    | 128 KB  | ~101 KB        | NN weights, FL buf, ring buf, ser buf, SD log queue |
| CCM      | 64 KB   | 0 KB (optional)| NN_Handle_t if moved to .ccmram   |
| Stack    | 2 KB    | ~800 B         | Main loop + ISR stack             |

### SRAM1 Static Allocation Detail

| Symbol              | Size      | Description                        |
|--------------------|-----------|------------------------------------|
| `s_nn.W1`          | 32,000 B  | Input→Hidden weights (500×16×4)    |
| `ser_packet_buf`   | 32,275 B  | FL serialization buffer            |
| `s_flc.train_buf`  | 20,010 B  | FL training window store           |
| `s_nn.dW1`         | 32,000 B  | W1 gradient buffer                 |
| `s_fe.ring`        | 2,560 B   | IMU ring buffer (128×5×4)          |
| `s_feature_vec`    | 2,000 B   | 500-float feature vector           |
| `s_queue` (DataLogger) | 2,560 B | SD log ring buffer (16×160 B)   |
| `s_fatfs`/`s_file` | ~600 B    | FatFs volume + file objects (FF_FS_TINY) |
| `s_nn.W2+b1+b2..`  | ~500 B    | Remaining NN parameters            |
| **Total**          | **~124 KB** | Within 128 KB SRAM1 limit       |

> **Note**: If using `dW1` and `W1` simultaneously with the `ser_packet_buf`, total peaks at ~86 KB active at once (not all simultaneously needed). If tight, move `s_nn` to CCM using `__attribute__((section(".ccmram")))`.

---

## Quick Start

### 1. Install Prerequisites (Ubuntu/Debian)

```bash
# ARM toolchain
sudo apt update
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi

# Verify toolchain
arm-none-eabi-gcc --version
# Expected: arm-none-eabi-gcc (GNU Arm Embedded Toolchain) 13.x.x

# OpenOCD for flashing
sudo apt install openocd

# st-flash alternative
sudo apt install stlink-tools

# For ST-Link USB access without root
sudo cp /usr/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 2. Generate CubeMX Base Project

1. Open STM32CubeMX
2. Follow `STM32CubeMX/CubeMX_Config_Guide.txt` exactly
3. Generate code with **Makefile** toolchain into this directory
4. CubeMX will populate `Drivers/`, `Core/Src/stm32f4xx_*.c`, etc.
5. **Do not** overwrite `Core/Src/main.c` — keep the provided version

### 3. Build

```bash
cd FedVibroSense_STM32
make all -j4
```

Expected output:
```
  CC   Core/Src/main.c
  CC   Core/Src/NeuralNetwork.c
  ...
  LD   build/FedVibroSense.elf

Linking target: build/FedVibroSense.elf
Memory region         Used Size  Region Size  %age Used
           FLASH:       54832 B         1 MB      5.23%
             RAM:       98240 B       128 KB     75.00%

--- Section sizes ---
   text    data     bss     dec     hex filename
  54832     412   97828  153072   25530 build/FedVibroSense.elf
```

### 4. Flash

```bash
# Via OpenOCD (preferred)
make flash

# Via st-flash
make flash_stl
```

### 5. Monitor Debug Output

```bash
# 115200 8N1 on USART2 (PA2/PA3)
minicom -D /dev/ttyUSB0 -b 115200

# Or with screen
screen /dev/ttyUSB0 115200
```

Expected startup output:
```
[INF] === FedVibroSense STM32F405 FL Client Started ===
[INF] FEATURE_VECTOR_SIZE=500, NN=500x16x3, FL_LOCAL_EPOCHS=10
[INF] TIM2 microsecond timer started
[INF] MPU6050 WHO_AM_I OK (0x68)
[INF] MPU6050 init OK — FS_accel=2g, FS_gyro=250°/s, SR=100Hz
[INF] Keep device STATIONARY for gyro calibration...
[INF] Gyro bias: X=0.0023 Y=-0.0011 Z=0.0008 °/s
[INF] App_Init complete. Waiting for 1 s of IMU data...
[INF] Window #1 | Pred=0 [N=0.921 I=0.052 L=0.027] | feat=1823 µs infer=48 µs
[INF] Window #2 | Pred=0 [N=0.934 I=0.041 L=0.025] | feat=1801 µs infer=47 µs
```

---

## Build Instructions

### Make (Primary)

```bash
make all          # Build ELF + BIN + HEX
make clean        # Remove build directory
make size         # Print section sizes only
make disasm       # Generate annotated disassembly → build/FedVibroSense.dis
make flash        # Flash via OpenOCD
make flash_stl    # Flash via st-flash
make debug        # Start OpenOCD + GDB session
```

### CMake (Alternative — CLion / VS Code)

```bash
mkdir build_cmake && cd build_cmake
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-none-eabi.cmake -G "Unix Makefiles"
make -j4
make flash
```

### Manual arm-none-eabi-gcc (Minimal Example)

```bash
# Compile one file manually (for troubleshooting)
arm-none-eabi-gcc \
  -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
  -DSTM32F405xx -DUSE_HAL_DRIVER \
  -ICore/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc \
  -IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
  -IDrivers/CMSIS/Include \
  -O2 -std=c11 -ffunction-sections -fno-strict-aliasing \
  -c Core/Src/NeuralNetwork.c -o build/NeuralNetwork.o
```

### Compiler Optimization Notes

| Flag    | Effect on NN forward pass          | Recommended? |
|---------|------------------------------------|--------------|
| `-O0`   | ~2.1 ms per forward pass           | Debug only   |
| `-Og`   | ~680 µs per forward pass           | Debug        |
| `-O2`   | ~48 µs per forward pass (FPU FMAC) | **Yes**      |
| `-O3`   | ~45 µs (marginal gain, larger code)| Optional     |
| `-Os`   | ~120 µs (size over speed)          | Not for NN   |

---

## Flash Instructions

### OpenOCD (Recommended)

```bash
# Flash and verify
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c "program build/FedVibroSense.elf verify reset exit"

# Flash with mass erase first (if device is protected)
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c "init; reset halt; flash erase_sector 0 0 11; \
        program build/FedVibroSense.elf verify reset exit"
```

### st-flash (stlink-tools)

```bash
# Write binary at flash origin
st-flash --reset write build/FedVibroSense.bin 0x8000000

# Verify only
st-flash verify build/FedVibroSense.bin 0x8000000

# Erase chip
st-flash erase
```

### STM32CubeProgrammer (GUI / CLI)

```bash
# CLI flashing
STM32_Programmer_CLI -c port=SWD -w build/FedVibroSense.elf -v -rst
```

### GDB via OpenOCD

```bash
# Terminal 1: Start OpenOCD server
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg

# Terminal 2: Connect GDB
arm-none-eabi-gdb build/FedVibroSense.elf \
    -ex "target remote localhost:3333" \
    -ex "monitor reset halt" \
    -ex "load" \
    -ex "monitor reset run"
```

---

## STM32CubeMX Setup

See `STM32CubeMX/CubeMX_Config_Guide.txt` for the complete step-by-step configuration. Key summary:

| Peripheral | Configuration                        |
|-----------|--------------------------------------|
| Clock     | HSE 8 MHz → PLL → 168 MHz SYSCLK    |
| FPU       | Enabled, Hard ABI                    |
| I2C1      | Fast mode 400 kHz (PB8/PB9)         |
| USART2    | 115200 baud (PA2/PA3) — Debug        |
| USART1    | 921600 baud (PA9/PA10) — FL comms    |
| TIM2      | 32-bit free-running @ 1 MHz          |
| TIM3      | 100 Hz interrupt (IRQ priority 1)    |
| SDIO      | 4-bit wide bus (PC8-11/PC12/PD2) — SD card |
| PA0       | EXTI0 falling edge (MPU-6050 INT)    |
| PD12–15   | GPIO Output (Status LEDs)            |

---

## Runtime Operation

### Main Loop Flow

```
Boot
 ├── HAL_Init() + SystemClock_Config() [168 MHz]
 ├── MX_GPIO/I2C/UART/TIM_Init()
 ├── Utils_TimerInit() — start TIM2
 ├── NN_Init() — Xavier weight initialization
 ├── FLC_Init() — FL client reset
 ├── CF_Init() — complementary filter reset
 ├── FE_Init() — ring buffer reset
 ├── MPU6050_Init() — verify WHO_AM_I, configure registers
 ├── MPU6050_Calibrate() — 2 s gyro bias collection (device stationary)
 ├── AppModule_Register() × 5 — IMUSample, FeatureWindow, FLClient,
 │      DataLogger, UARTCommand (see AppModule.h / Config.h)
 ├── AppModule_InitAll() — DataLogger mounts SD, opens LOGS/SNX_NNNNN.CSV
 └── Main Loop = AppModule_TickAll(), repeated forever:
      ├── [100 Hz]  Mod_ImuSample_Tick     — ReadScaled → CF_Update → FE_Push
      ├── [1 Hz]    Mod_FeatureWindow_Tick — FE_BuildFeatureVector → NN_Forward
      │                                      → DataLogger_LogWindow (enqueue)
      │                                      → FLC_SubmitSample (if labeled)
      ├── [event]   Mod_FLC_Tick           — FLC_Tick FSM step, logs round
      │                                      summary via DataLogger_LogFLRound
      ├── [tick]    DataLogger tick        — drains ≤1 queued SD row, flushes
      │                                      on SD_LOG_FLUSH_INTERVAL_MS
      └── [async]   Mod_UartCommand_Tick   — App_HandleUARTCommand
```

Adding a new subsystem means one `AppModule_Register()` call — this loop
body doesn't need to change again. See "Scalability Refactor" above.

### Timing Budget per 100 Hz Cycle (10 ms budget)

| Operation              | Typical Time | % of 10 ms budget |
|------------------------|-------------|-------------------|
| MPU6050_ReadScaled()   | ~120 µs     | 1.2%              |
| CF_Update()            | ~32 µs      | 0.3%              |
| FE_Push()              | ~2 µs       | 0.02%             |
| FE_BuildFeatureVector()| ~1,800 µs   | 18% (1 Hz only)   |
| NN_Forward()           | ~48 µs      | 0.5% (1 Hz only)  |
| **Total (per 10 ms)**  | **~154 µs** | **1.5%**          |

The CPU is idle ~98.5% of the time at 100 Hz sampling. This headroom accommodates UART logging, FL transmission, and future sensor additions.

### Training Step Timing

| Operation          | Time @ 168 MHz, -O2 |
|-------------------|---------------------|
| NN_Forward()       | ~48 µs              |
| NN_Backward()      | ~180 µs             |
| NN_UpdateWeights() | ~95 µs              |
| **Total TrainStep**| **~323 µs**         |

Full FL round (10 local epochs): ~3.2 ms training + ~352 ms upload = ~355 ms total.

---

## Federated Learning Protocol

### FL Round Sequence

```
STM32 Node                          Aggregation Server (RPi)
     │                                        │
     │── [COLLECTING] ──────────────────────► │
     │   10 labeled feature windows           │
     │                                        │
     │── [TRAINING] ────────────────────────► │
     │   10× NN_TrainStep() locally           │
     │                                        │
     │── [UPLOADING] ──────────────────────── │
     │   Binary FL packet (32,275 bytes)     ──► Receive
     │   UART1 @ 921600 baud (~352 ms)        │
     │                                        │   FedAvg(W_1...W_N)
     │◄─ ACK (0xAC 0xAC) ────────────────── ◄── Send ACK
     │                                        │
     │── [DOWNLOADING] ◄────────────────────  │
     │◄── Global model (32,275 bytes) ──────◄── Send global model
     │                                        │
     │── [IDLE] ─────────────────────────── ► │
     │   Apply global model, clear buf        │
     └────────────────────────────────────────┘
```

### Packet Format

```
 Byte  0- 1:  Magic     0xFE01 (upload) / 0xFE02 (download)  [uint16 LE]
 Byte  2   :  Version   0x01                                  [uint8]
 Byte  3- 4:  NumWeights 8067                                 [uint16 LE]
 Byte  5-32272: Payload  W1|b1|W2|b2 as float32 LE            [8067×4 B]
 Byte  32273-32274: CRC-16/CCITT over bytes 0–32272           [uint16 LE]
 ─────────────────────────────────────────────────────────────
 Total: 32,275 bytes
```

### Python Aggregation Server (Raspberry Pi)

```python
#!/usr/bin/env python3
"""
FedVibroSense aggregation server — Raspberry Pi reference implementation.
Performs FedAvg over N client uploads, sends global model back.

Install: pip3 install pyserial numpy
Usage:   python3 fed_server.py --port /dev/ttyAMA0 --clients 3
"""

import serial
import struct
import numpy as np
import argparse

MAGIC_UPLOAD   = 0xFE01
MAGIC_DOWNLOAD = 0xFE02
PROTOCOL_VER   = 0x01
FL_WEIGHT_COUNT = 8067
PACKET_SIZE    = 5 + FL_WEIGHT_COUNT * 4 + 2  # 32275

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
        crc &= 0xFFFF
    return crc

def receive_packet(ser: serial.Serial) -> np.ndarray:
    """Receive one FL packet, validate CRC, return weight array."""
    raw = ser.read(PACKET_SIZE)
    if len(raw) != PACKET_SIZE:
        raise RuntimeError(f"Incomplete packet: {len(raw)} bytes")
    magic, ver, n_w = struct.unpack_from('<HBH', raw, 0)
    assert magic == MAGIC_UPLOAD,    f"Bad magic: {magic:#06x}"
    assert ver == PROTOCOL_VER,      f"Bad version: {ver}"
    assert n_w == FL_WEIGHT_COUNT,   f"Bad weight count: {n_w}"
    payload = raw[5:5 + FL_WEIGHT_COUNT * 4]
    rx_crc  = struct.unpack_from('<H', raw, 5 + FL_WEIGHT_COUNT * 4)[0]
    calc_crc = crc16_ccitt(raw[:5 + FL_WEIGHT_COUNT * 4])
    assert rx_crc == calc_crc, f"CRC mismatch: {rx_crc:#06x} vs {calc_crc:#06x}"
    weights = np.frombuffer(payload, dtype=np.float32).copy()
    return weights

def send_packet(ser: serial.Serial, weights: np.ndarray):
    """Serialize and send global model with download magic."""
    header  = struct.pack('<HBH', MAGIC_DOWNLOAD, PROTOCOL_VER, FL_WEIGHT_COUNT)
    payload = weights.astype(np.float32).tobytes()
    crc_data = header + payload
    crc = crc16_ccitt(crc_data)
    packet = crc_data + struct.pack('<H', crc)
    assert len(packet) == PACKET_SIZE
    ser.write(packet)

def fedavg(weight_list: list) -> np.ndarray:
    """Simple FedAvg: element-wise mean over all clients."""
    return np.mean(np.stack(weight_list, axis=0), axis=0)

def main():
    parser = argparse.ArgumentParser(description='FedVibroSense Aggregation Server')
    parser.add_argument('--port',    default='/dev/ttyAMA0', help='UART port')
    parser.add_argument('--baud',    default=921600,         type=int)
    parser.add_argument('--clients', default=1,              type=int)
    parser.add_argument('--rounds',  default=100,            type=int)
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=30)
    print(f"Server listening on {args.port} @ {args.baud} baud")
    print(f"Expecting {args.clients} client(s), {args.rounds} FL rounds")

    for rnd in range(1, args.rounds + 1):
        print(f"\n=== FL Round {rnd}/{args.rounds} ===")
        client_weights = []

        for c in range(args.clients):
            print(f"  Waiting for client {c+1}/{args.clients}...")
            weights = receive_packet(ser)
            client_weights.append(weights)
            # Send ACK
            ser.write(bytes([0xAC, 0xAC]))
            ser.flush()
            print(f"  Client {c+1} received — ACK sent")

        # FedAvg aggregation
        global_weights = fedavg(client_weights)
        print(f"  FedAvg done. W1 mean={global_weights[:8000].mean():.6f}")

        # Send global model to all clients (broadcast)
        for c in range(args.clients):
            send_packet(ser, global_weights)
            print(f"  Global model sent to client {c+1}")

    print("\nAggregation server done.")
    ser.close()

if __name__ == '__main__':
    main()
```

---

## UART Command Interface

Connect a terminal to USART2 (PA2/PA3) at **115200 baud, 8N1**.

| Key | Action                                |
|-----|---------------------------------------|
| `0` | Set label = **NORMAL** (class 0)      |
| `1` | Set label = **IMBALANCE** (class 1)   |
| `2` | Set label = **LOOSENESS** (class 2)   |
| `u` | Trigger immediate FL weight upload    |
| `r` | Reset FL client to IDLE state         |
| `s` | Print FL client status summary        |
| `w` | Print NN weight statistics            |

### Example Session

```
> 0       ← Set label to NORMAL
[INF] Label set → NORMAL
[INF] FL: submitted window #5 with label=0 (1/10 buffered)
...
[INF] FL: submitted window #14 with label=0 (10/10 buffered)
[INF] FL: COLLECTING → TRAINING
[INF] FL: === LOCAL TRAINING START (round 1) ===
[INF] FL: train step 1/10 label=0 loss=1.0832
[INF] FL: train step 2/10 label=0 loss=0.9841
...
[INF] FL: train step 10/10 label=0 loss=0.4123
[INF] FL: training done, mean_loss=0.7210, total_samples=10
[INF] FL: UPLOADING weights (32275 bytes)...
[INF] SER: serialized 32275 bytes, magic=0xFE01, CRC=0x3A2F
[INF] FL: upload TX complete, waiting for ACK...
[INF] FL: ACK received — UPLOADING → DOWNLOADING
[INF] SER: deserialized global model OK, CRC=0x7B91
[INF] FL: === ROUND 1 COMPLETE ===
[INF] FL Status: state=IDLE round=1 samples=10 loss=0.7210 server=1
```

---

## Future Roadmap

### 1. INT8 Quantization

Replace `float32` weights with `int8` to reduce W1 from **32 KB → 8 KB** and enable 4× throughput on dot products:

```c
// Post-training quantization:
// scale = max(|W|) / 127.0f
// W_int8[i] = (int8_t)(W[i] / scale)

// Quantized MAC in forward pass:
int32_t acc = 0;
for (uint32_t i = 0; i < NN_INPUT_SIZE; i++) {
    acc += (int32_t)W1_int8[j * I + i] * (int32_t)x_int8[i];
}
z1[j] = (float)acc * scale_W1 * scale_x;
```

CMSIS-NN provides `arm_fully_connected_q7()` for this exact operation.

### 2. CMSIS-NN Acceleration

Replace the manual GEMV in `NN_Forward()` with CMSIS-NN:

```c
// Drop-in for quantized forward pass Layer 1:
arm_fully_connected_q7(
    x_q7,       // int8 input
    W1_q7,      // int8 weights
    NN_INPUT_SIZE, NN_HIDDEN_SIZE,
    bias_shift, output_shift,
    b1_q7,      // int32 bias
    a1_q7,      // int8 output
    vec_buffer  // scratch buffer
);
```

### 3. CMSIS-DSP Feature Extraction

Replace manual FFT-free feature computation with CMSIS-DSP:

```c
// Example: RMS of accelerometer magnitude using CMSIS-DSP
float32_t rms_result;
arm_rms_f32(accel_mag_window, FEATURE_WINDOW_SAMPLES, &rms_result);

// arm_mean_f32, arm_std_f32, arm_max_f32 — all vectorized for Cortex-M4
```

Consider adding **frequency-domain features** via `arm_rfft_fast_f32()`:
- FFT magnitude at peak frequency bin
- Spectral centroid
- Energy in 5–20 Hz (typical machine imbalance range)

### 4. Multi-Node FL Scaling

For N > 1 nodes on a shared aggregation server:

- Add node ID byte to packet header (1 byte, offset 2)
- Server collects N packets before computing FedAvg
- Use RS-485 half-duplex bus for multi-drop UART topology
- Alternative: add ESP32-S3 Wi-Fi co-processor on SPI/UART for TCP/IP transport

### 5. Power Optimization

| Technique                   | Estimated Saving |
|-----------------------------|-----------------|
| STOP mode between samples   | ~70% reduction  |
| RTC wakeup instead of TIM3  | Avoids HSI jitter |
| DMA I2C read (non-blocking) | Allows WFI during transfer |
| Reduce clock to 80 MHz      | ~50% dynamic power |
| MPU-6050 low-power mode     | 10 µA sensor sleep |

```c
// Example: enter STOP mode between 100 Hz samples
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        g_sample_flag = 1U;
    }
}
// In main loop, after processing:
__WFI();   // Sleep until next interrupt (TIM3 at 100 Hz)
```

### 6. Publication-Quality Extension

To convert this firmware into an IEEE Sensors Journal / IEEE IoT Journal paper platform:

1. **Testbed**: Mount MPU-6050 on a motor with known imbalance masses (add calibrated washers)
2. **Dataset**: Collect ≥500 labeled windows per class, stored on SD card or transmitted to PC
3. **Baseline**: Train a Python scikit-learn SVM on the same 500-float features for comparison
4. **Metrics**: Log training loss, inference accuracy, weight divergence (||W_local - W_global||₂) per FL round
5. **Ablation**: Vary FL_LOCAL_EPOCHS (1, 5, 10, 20) and measure convergence rounds
6. **Timing**: Record forward pass µs, backward pass µs, transfer ms — include in paper Table II
7. **Energy**: Measure current with Nordic PPK2 or Otii Arc during FL round
8. **Reproducibility**: Use fixed Utils_SeedRNG(0xABCD1234) and document in paper

---

## Research Notes

### Mathematical Correctness Checklist

- [x] Xavier uniform init with correct fan_in+fan_out denominator
- [x] Numerically stable Softmax (max-subtraction before exp)
- [x] Numerically stable CE loss (log(p + ε))
- [x] Combined Softmax + CE backward (δ2 = a2 - y_onehot)
- [x] ReLU derivative gated by pre-activation z1 (not a1)
- [x] CRC-16/CCITT with correct polynomial (0x1021), init (0xFFFF)
- [x] Ring buffer chronological reconstruction (head - N + s)
- [x] Z-score with sample std (n-1 denominator)
- [x] Complementary filter time constant τ = α·dt/(1-α) ≈ 0.49 s

### Known Limitations

1. **Online learning only**: No mini-batching — one sample per gradient step. For research, implement gradient accumulation over a mini-batch before calling `NN_UpdateWeights()`.
2. **No momentum / Adam**: SGD with constant learning rate. Add Adam with 3 extra float arrays (m, v, t) per weight tensor for faster convergence.
3. **Fixed α for CF**: The complementary filter α=0.98 is fixed. For mounting configurations with significant vibration, tune α or switch to Madgwick filter.
4. **Synchronous FL**: Only one client implemented. Multi-client requires asynchronous UART with node ID routing.
5. **Label latency**: Features are labeled with the label set at window collection time. Latency between condition change and label update = up to 1 s.

---

## Troubleshooting

### MPU-6050 Not Responding

```
[ERR] MPU6050 WHO_AM_I mismatch: got 0x00, expected 0x68
```

- Check SDA/SCL connections and 4.7 kΩ pull-ups
- Verify AD0 pin is tied to GND (not floating)
- Measure I2C bus with oscilloscope — should see 400 kHz pulses
- Try reducing I2C clock to 100 kHz: `hi2c1.Init.ClockSpeed = 100000U;`

### Calibration Drift

If pitch/roll drift after calibration:
- Ensure device is completely stationary during `MPU6050_Calibrate()`
- Increase `MPU6050_CALIBRATION_SAMPLES` from 200 to 500
- Reduce `CF_ALPHA` from 0.98 to 0.95 for more accelerometer weighting

### UART FL Transfer Timeout

```
[ERR] FL: ACK timeout or UART error (HAL=3)
```

- Verify Raspberry Pi UART is enabled: `sudo raspi-config → Interface → Serial`
- Check baud rate: both sides must be 921600
- Wire RX of one device to TX of the other (crossed)
- Try reducing to 460800 baud in `Config.h` → `UART_FL_BAUD`

### Build Error: `arm-none-eabi-gcc: not found`

```bash
# Ubuntu/Debian
sudo apt install gcc-arm-none-eabi

# Or download directly from ARM:
# https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
# Extract and add to PATH:
export PATH=$PATH:/opt/arm-gnu-toolchain-13.x/bin
```

### Flash Error: `Error: open failed`

```bash
# Check ST-Link is detected
lsusb | grep -i stm
# Expected: Bus xxx Device yyy: ID 0483:3748 STMicroelectronics ST-LINK/V2

# Check udev rules
ls /etc/udev/rules.d/ | grep openocd
# If missing:
sudo cp /usr/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
# Reconnect ST-Link USB
```

### Out of RAM: `region RAM overflowed`

Move the NN handle to CCM RAM in `main.c`:
```c
__attribute__((section(".ccmram"))) static NN_Handle_t s_nn;
```
This places the 64 KB W1+dW1 tensors in Core Coupled Memory, freeing ~64 KB of SRAM1.

---

## License

MIT License — see LICENSE file.  
Research use encouraged. Please cite this repository if used in publications.

---

## Authors

**FedVibroSense Project**  
Embedded Federated Learning for Vibration Anomaly Detection  
Target: IEEE Sensors Journal / IEEE Internet of Things Journal

*Built on STM32F405RGT6 + MPU-6050 + arm-none-eabi-gcc + OpenOCD*
