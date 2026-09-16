#ifndef APP_LORA_H
#define APP_LORA_H

#include <stdint.h>

typedef enum {
    LORA_MODE_NORMAL,
    LORA_MODE_WAKEUP,
    LORA_MODE_LOWPOWER,
    LORA_MODE_SLEEP,
    LORA_MAX_MODE
} lora_mode_t;

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

void app_lora_init(void);

void app_lora_set_mode(lora_mode_t mode);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // APP_LORA_H