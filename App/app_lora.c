/**
 * @file app_lora.c
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief LoRa 当前有广播、监听等功能。当前设备作为挂接头，主负责从设备的数据上传更新。
 *        需要将主设备地址设置为0xFFFF，信道需要约定一致。
 *        当前从设备要先设置好地址例如0x0001，信道设置为0x04。
 * @version 0.1
 * @date 2026-09-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */

 #include <stdbool.h>
 #include <stddef.h>

#include "app_lora.h"


#include "main.h"   /* HAL_Delay / HAL_GetTick */

/* 模块唤醒后数据的到达时间:AUX 唤醒标志置位后,数据还要再等约 5~10ms 才从 UART 出来 */
#define APP_LORA_RX_WAIT_MS      10U
/* 等待数据的总超时:固定等待后仍没数据,最多再等到此时间,防止流程卡死 */
#define APP_LORA_RX_TIMEOUT_MS   50U

// LoRa 状态和M0 M1 引脚电平映射
static struct {
    bsp_gpio_level_t m0_level;
    bsp_gpio_level_t m1_level;
} app_lora_status_t[LORA_MAX_MODE] = {
    [LORA_MODE_NORMAL]   = { BSP_GPIO_LOW, BSP_GPIO_LOW },
    [LORA_MODE_WAKEUP]   = { BSP_GPIO_HIGH, BSP_GPIO_LOW },
    [LORA_MODE_LOWPOWER] = { BSP_GPIO_LOW, BSP_GPIO_HIGH },
    [LORA_MODE_SLEEP]    = { BSP_GPIO_HIGH, BSP_GPIO_HIGH },
};

// LoRa 驱动接口结构体
static struct 
{
    void (*lora_send)(const uint8_t *data, uint16_t len);     /* 发送(DMA + 阻塞语义) */
    uint32_t (*lora_rx_available)(void);                      /* 可读字节数(与 bsp_lora_uart 一致) */
    uint8_t (*lora_rx_get)(uint8_t *b);                       /* 取一个字节 */
    uint8_t (*lora_rx_peek)(uint8_t *b);
    uint8_t (*lora_rx_get_bytes)(uint8_t *buf, uint16_t len); /* 取多字节 */

} lora_drv = {
    .lora_send = lora_send,
    .lora_rx_available = lora_rx_available,
    .lora_rx_get = lora_rx_get,
    .lora_rx_peek = lora_rx_peek,
    .lora_rx_get_bytes = lora_rx_get_bytes,
};

typedef struct {
    uint16_t lora_address;   /* 当前设备地址 */

    uint8_t lora_channel;     /* 当前设备信道 */

    lora_mode_t current_mode;

    lora_state_t current_state;
    
    uint8_t lora_initialized;
    
} app_lora_t;

/*********************************
 * 静态全局变量
 *********************************/
static app_lora_t lora;
volatile uint8_t lora_wakeup;                  /* 模块唤醒标志 */
static lora_wake_cb_t lora_wakeup_cb = NULL;    /* 模块唤醒回调函数 */
uint8_t lora_rx_buf[24];


/*********************************
 * 回调函数
 *********************************/
static void lora_callback(bsp_sig_ch_t GPIO_Pin)
{
    (void)GPIO_Pin;
    lora_wakeup = true;                 /* 标志式:业务在主循环里取       */

    if (lora_wakeup_cb != NULL)
    {
        lora_wakeup_cb();              /* 回调式:中断上下文,只能短动作 */
    }
}


/**
 * 对外输出接口
 * app_lora_init 初始化 LoRa 模块
 * app_lora_set_mode 设置 LoRa 模块的工作模式
 * app_lora_send_bytes 发送字节数据到 LoRa 模块
 * app_lora_wait_rx 等待一帧数据到达
 * app_lora_read_bytes 读取字节数据从 LoRa 模块
 * app_lora_register_wake_callback 注册 LoRa 模块唤醒回调函数
 * app_lora_wakeup_flag 获取 LoRa 模块唤醒标志
 * app_lora_wakeup_clear 清除 LoRa 模块唤醒标志
 * app_lora_get_status 获取 LoRa 模块当前状态。
 */
void app_lora_init(void)
{
    //DRV init
    //UART+GPIO已集中完成初始化

    lora_wakeup = false;
    lora.current_mode = LORA_MODE_NORMAL;
    lora.current_state = LORA_IDLE;
    app_lora_set_mode(LORA_MODE_NORMAL);

    // 注册 LoRa 回调函数
    bsp_gpio_register_exti_callback(BSP_LORA_AUX, lora_callback);
    
}

void app_lora_set_mode(lora_mode_t mode)
{
    if(mode < LORA_MAX_MODE)
    {
        lora.current_mode = mode;
        // 设置 LoRa 模块的 M0 和 M1 引脚电平
        bsp_gpio_set_level(BSP_LORA_M0, app_lora_status_t[mode].m0_level);
        bsp_gpio_set_level(BSP_LORA_M1, app_lora_status_t[mode].m1_level);
    }
}

void app_lora_send_bytes(const uint8_t *data, uint16_t len)
{
    if ((data != NULL) && (len > 0U))
    {
        lora_drv.lora_send(data, len);
    }
}

void app_lora_wait_rx(void)
{
    uint32_t t0 = HAL_GetTick();

    HAL_Delay(APP_LORA_RX_WAIT_MS);
    while ((lora_drv.lora_rx_available() == 0U) &&
           ((HAL_GetTick() - t0) < APP_LORA_RX_TIMEOUT_MS))
    {
        HAL_Delay(1U);
    }
}

uint8_t app_lora_read_bytes(uint8_t *data, uint16_t len)
{
    if ((data != NULL) && (len > 0U))
    {
        return lora_drv.lora_rx_get_bytes(data, len);
    }
    return 0U;
}

uint8_t app_lora_read_byte(uint8_t *data)
{
    if (data != NULL)
    {
        return lora_drv.lora_rx_get(data);
    }
    return 0U;
}

uint8_t app_lora_peek_byte(uint8_t *data)
{
    if (data != NULL)
    {
        return lora_drv.lora_rx_peek(data);
    }
    return 0U;
}

uint8_t app_lora_rx_available(void)
{
    return lora_drv.lora_rx_available();
}

void app_lora_register_wake_callback(lora_wake_cb_t cb)
{
    lora_wakeup_cb = cb;
}

uint8_t app_lora_wakeup_flag(void)
{
    return lora_wakeup;
}

void app_lora_wakeup_clear(void)
{
    lora_wakeup = false;
}

lora_state_t app_lora_get_status(void)
{
    return lora.current_state;
}
