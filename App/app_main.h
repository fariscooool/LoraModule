#ifndef APP_MAIN_H
#define APP_MAIN_H

#include "bsp_gpio.h"   /* bsp_sig_ch_t 等类型 */

typedef enum 
{
    STATE_IDLE,
    STATE_MATCHING,
    STATE_RUNNING,
    STATE_ERROR
}app_status_t;

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



