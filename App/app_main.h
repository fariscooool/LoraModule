#ifndef APP_MAIN_H
#define APP_MAIN_H

#include "bsp_gpio.h"   /* bsp_sig_ch_t 等类型 */

#ifdef __cplusplus
extern "C" {
#endif

void app_init(void);

void app_deinit(void);

void app_task(void);

/* ---------------- 供串口命令模块(app_cmd)等调用 ---------------- */

/**
 * @brief 组上报帧并通过 UART1(LPUART1) 下发给 LoRa 模块
 * @note  业务功能,发布版也保留;内部调试打印受 APP_DEBUG_ENABLE 控制
 */
void app_sig_report(bsp_sig_ch_t ch);

/**
 * @brief 读取某通道当前“参数”(默认来自 app_config.h 的宏,可被命令修改)
 */
uint8_t app_param_get(bsp_sig_ch_t ch);

/**
 * @brief 修改某通道运行时“参数”(0~255)
 */
void app_param_set(bsp_sig_ch_t ch, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif // APP_MAIN_H



