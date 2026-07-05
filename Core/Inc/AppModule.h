/**
 * @file    AppModule.h
 * @brief   Minimal module interface for the main-loop scheduler.
 *
 *
 * @author  SensiNerveX Project
 * @version 2.0.0
 */

#ifndef APPMODULE_H
#define APPMODULE_H

#include <stdint.h>
#include "Config.h"

typedef void (*AppModule_InitFn)(void);
typedef void (*AppModule_TickFn)(void);

typedef struct {
    const char       *name;   
    AppModule_InitFn  init;   
    AppModule_TickFn  tick;  
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
