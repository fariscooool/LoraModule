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

// 1. 定义枚举，方便阅读和赋值
typedef enum {
    PARITY_8N1 = 0, // 00
    PARITY_8O1 = 1, // 01
    PARITY_8E1 = 2, // 10
    PARITY_8N1_2 = 3 // 11 (等同00)
} parity_t;

typedef enum {
    BAUD_1200 = 0,  // 000
    BAUD_2400 = 1,  // 001
    BAUD_4800 = 2,  // 010
    BAUD_9600 = 3,  // 011 (默认)
    BAUD_19200 = 4, // 100
    BAUD_38400 = 5, // 101
    BAUD_57600 = 6, // 110
    BAUD_115200 = 7 // 111
} ttl_baud_t;

typedef enum {
    AIR_0_3K = 0,   // 000
    AIR_1_2K = 1,   // 001
    AIR_2_4K = 2,   // 010 (默认)
    AIR_4_8K = 3,   // 011
    AIR_9_6K = 4,   // 100
    AIR_19_2K = 5,  // 101
    AIR_19_2K_2 = 6,// 110
    AIR_19_2K_3 = 7 // 111
} air_rate_t;

typedef enum {
    WAKEUP_250MS = 0,  // 000
    WAKEUP_500MS = 1,  // 001
    WAKEUP_750MS = 2,  // 010
    WAKEUP_1000MS = 3,  // 011
    WAKEUP_1250MS = 4,  // 100
    WAKEUP_1500MS = 5,  // 101
    WAKEUP_1750MS = 6,  // 110
    WAKEUP_2000MS = 7   // 111
}wakeup_time_t;

typedef enum {
    IO_DRV_MODE_PUSH_PULL = 0,  // 推挽输出
    IO_DRV_MODE_OPEN_DRAIN = 1  // 开漏输出
} io_drv_mode_t;

typedef enum {
    FIXED_POINT_TRANS_DISABLE = 0,  // 定点传输禁用
    FIXED_POINT_TRANS_ENABLE  = 1   // 定点传输使能
} fixed_point_trans_t;

typedef union {
    uint8_t data[5]; /* 原始数据帧, 5字节 */
    struct {
        uint8_t addr_high;   /* 地址高 */
        uint8_t addr_low;    /* 地址低 */
        union {
            uint8_t raw;
            struct {
                uint8_t air_rate : 3;  /* 空中速率, 3位 */
                uint8_t ttl_rate : 3;
                uint8_t parity : 2;  /* 校验位, 2位 */
            }bits;
        }sped;
        uint8_t channel;       /* 信道 7～0位，对应（410MHz+CHAN *1MHz），默认3CH（470MHz）00H-73H，对应410～525MHz*/
        union {
            uint8_t raw;
            struct {
                uint8_t reserved : 2;       /* 保留位, 2位 */
                uint8_t wakeup_time : 3;    /* 唤醒时间, 3位 */
                uint8_t io_drv_mode : 1;    /* IO 驱动模式 */
                uint8_t fixed_point_trans : 1; /* 定点传输使能 */
            }bits;
        }option;
    } detail;
}lora_reg_parm_cfg_t;

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
 * @brief 查看接收 FIFO 中的一个字节(不移动读指针)
 * @param  data 数据缓冲区首地址
 * @retval 1=成功  0=空
 */
uint8_t app_lora_peek_byte(uint8_t *data);

/**
 * @brief 获取接收 FIFO 中可读字节数
 * @retval 可读字节数
 */
uint8_t app_lora_rx_available(void);

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