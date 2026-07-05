/**
 * @file    AppModule.c
 * @brief   Fixed-size module table implementation. See AppModule.h
 */

#include "AppModule.h"
#include "Utils.h"

static const AppModule_t *s_modules[APP_MAX_MODULES];
static uint8_t             s_count = 0U;

uint8_t AppModule_Register(const AppModule_t *module)
{
    if (s_count >= APP_MAX_MODULES) {
        LOG_ERR("AppModule: table full (%u), cannot register '%s' — "
                "raise APP_MAX_MODULES in Config.h",
                APP_MAX_MODULES, module->name ? module->name : "?");
        return 0U;
    }
    s_modules[s_count++] = module;
    LOG_INF("AppModule: registered '%s' (%u/%u)",
            module->name ? module->name : "?", s_count, APP_MAX_MODULES);
    return 1U;
}

void AppModule_InitAll(void)
{
    for (uint8_t i = 0U; i < s_count; i++) {
        if (s_modules[i]->init != NULL) {
            s_modules[i]->init();
        }
    }
}

void AppModule_TickAll(void)
{
    for (uint8_t i = 0U; i < s_count; i++) {
        if (s_modules[i]->tick != NULL) {
            s_modules[i]->tick();
        }
    }
}

uint8_t AppModule_Count(void)
{
    return s_count;
}
