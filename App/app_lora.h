#ifndef APP_LORA_H
#define APP_LORA_H

#include <stdint.h>
#include "bsp_gpio.h"
#include "bsp_lora_uart.h"

typedef enum {
    LORA_MODE_NORMAL,
    LORA_MODE_WAKEUP,
    LORA_MODE_LOWPOWER,
    LORA_MODE_SLEEP,
    LORA_MAX_MODE
} lora_mode_t;

typedef enum {
    LORA_IDLE,
    LORA_RECEIVING,
    LORA_SENDING
} lora_state_t;

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/**
 * @brief LoRa唤醒回调函数类型定义，用于模块唤醒事件的处理。
**/
typedef void (*lora_wake_cb_t)(void);

/**
 * @brief 初始化 LoRa 模块
**/
void app_lora_init(void);

/**
 * @brief 设置 LoRa 模块的工作模式
 * @param  mode LoRa 模块的工作模式
**/
void app_lora_set_mode(lora_mode_t mode);

/**
 * @brief 发送字节数据到 LoRa 模块
 * @param  data 数据缓冲区首地址
 * @param  len  数据长度(字节)
 * @retval None
**/
void app_lora_send_bytes(const uint8_t *data, uint16_t len);

/**
 * @brief 等待一帧数据到达 LoRa 模块
 * @note  阻塞执行:内部会延迟等待数据到达(约 5~10ms)
**/
void app_lora_wait_rx(void);

/**
 * @brief 读取字节数据从 LoRa 模块
 * @param  data 数据缓冲区首地址
 * @param  len  数据长度(字节)
**/
uint8_t app_lora_read_bytes(uint8_t *data, uint16_t len);

/**
 * @brief 读取单字节数据从 LoRa 模块
 * @param  data 数据缓冲区首地址
 * @retval 读取到的字节数(0=无数据)
**/
uint8_t app_lora_read_byte(uint8_t *data);

/**
 * @brief 注册 LoRa 模块唤醒回调函数
 * @param  cb 唤醒回调函数
**/
void app_lora_register_wake_callback(lora_wake_cb_t cb);

/**
 * @brief 获取 LoRa 模块唤醒标志
 * @return LoRa 模块唤醒标志
**/
uint8_t app_lora_wakeup_flag(void);

/**
 * @brief 清除 LoRa 模块唤醒标志
 * @retval None
**/
void app_lora_wakeup_clear(void);

/**
 * @brief 获取 LoRa 模块当前状态
 * @return LoRa 模块当前状态
**/
lora_state_t app_lora_get_status(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // APP_LORA_H