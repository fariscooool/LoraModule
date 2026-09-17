#ifndef APP_LORA_PROCOTOL_H
#define APP_LORA_PROCOTOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "app_lora.h"

/**
 * @brief LoRa 模块周期处理函数
 * @note  在主循环里周期性调用:
 *        唤醒事件 -> 延迟等待数据到达(约 5~10ms) -> 拆包解析 -> 主动发送
 */
void app_lora_update(void);

#ifdef __cplusplus
}
#endif

#endif // APP_LORA_PROCOTOL_H