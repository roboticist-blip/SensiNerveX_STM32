/**
 * @file    AppModule.h
 * @brief   Minimal module interface for the main-loop scheduler.
 *
 * Problem this solves:
 *   The original main.c called each subsystem (MPU6050, FeatureExtractor,
 *   NeuralNetwork, FederatedClient, ...) directly, in a hand-written
 *   sequence, with subsystem-specific globals and state checks inlined
 *   into the loop body. That's fine for 4 subsystems; it gets unwieldy
 *   fast as more are added (SD logging, a second sensor, a Wi-Fi/BLE
 *   bridge, ...) because every addition means editing the loop itself
 *   and re-reasoning about ordering and timing budgets.
 *
 * What changes:
 *   Every subsystem exposes one AppModule_t: a name (for logging), an
 *   Init(), and a Tick() called once per main-loop iteration. main.c
 *   builds a fixed-size table (AppModule_Register) at boot and the loop
 *   body becomes a single "for each module: Tick()" — see main.c.
 *   Ordering is still explicit (registration order == tick order), so
 *   timing-critical subsystems (IMU sampling) can still be registered
 *   first; nothing here hides that, it just stops main.c from growing
 *   linearly with subsystem count.
 *
 * What this deliberately is NOT:
 *   A scheduler, an RTOS, or a pub/sub bus. It is the smallest possible
 *   abstraction that removes main.c as the bottleneck for adding new
 *   subsystems, without introducing dynamic dispatch overhead, heap
 *   allocation, or priority inversion risk on a bare-metal 100 Hz loop.
 *   Modules that need to talk to each other (e.g. FederatedClient wanting
 *   to log a round summary) still do so via a direct, typed function call
 *   (DataLogger_LogFLRound()) — not through this table — because that
 *   keeps call sites greppable and type-checked instead of hiding behind
 *   an event name.
 *
 * @author  SensiNerveX Project
 * @version 1.0.0
 */

#ifndef APPMODULE_H
#define APPMODULE_H

#include <stdint.h>
#include "Config.h"

typedef void (*AppModule_InitFn)(void);
typedef void (*AppModule_TickFn)(void);

typedef struct {
    const char       *name;   /**< For boot-time logging / fault reporting */
    AppModule_InitFn  init;   /**< May be NULL if the module needs no init */
    AppModule_TickFn  tick;   /**< May be NULL for init-only modules       */
} AppModule_t;

/**
 * @brief  Add a module to the boot-time table. Call during App_Init(),
 *         before AppModule_InitAll(). Registration order == tick order.
 * @return 1 on success, 0 if APP_MAX_MODULES is already full (checked
 *         with a LOG_ERR so a config mistake is visible at boot, not
 *         a silent no-op).
 */
uint8_t AppModule_Register(const AppModule_t *module);

/**
 * @brief  Call every registered module's init() in registration order.
 */
void AppModule_InitAll(void);

/**
 * @brief  Call every registered module's tick() in registration order.
 *         This is the entire body of main.c's steady-state loop.
 */
void AppModule_TickAll(void);

/**
 * @brief  Number of modules currently registered (for boot-time logging).
 */
uint8_t AppModule_Count(void);

#endif /* APPMODULE_H */
