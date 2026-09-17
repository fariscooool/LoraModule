#ifndef APP_MAIN_H
#define APP_MAIN_H

#include "bsp_gpio.h"   /* bsp_sig_ch_t 等类型 */

#ifdef __cplusplus
extern "C" {
#endif

void app_init(void);

void app_deinit(void);

void app_task(void);


#ifdef __cplusplus
}
#endif

#endif // APP_MAIN_H



