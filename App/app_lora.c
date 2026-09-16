/**
 * @file app_lora.c
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-09-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */

 #include <stdbool.h>

#include "app_lora.h"
#include "bsp_gpio.h"

typedef struct {
    lora_mode_t current_mode;
    uint8_t lora_initialized;

    uint8_t lora_wakeup;
} app_lora_t;

// LoRa 状态和M0 M1 引脚电平映射
static struct {
    bsp_gpio_level_t m0_level;
    bsp_gpio_level_t m1_level;
} app_lora_status_t[LORA_MAX_MODE] = {
    [LORA_MODE_NORMAL] = { BSP_GPIO_LOW, BSP_GPIO_LOW },
    [LORA_MODE_WAKEUP] = { BSP_GPIO_HIGH, BSP_GPIO_LOW},
    [LORA_MODE_LOWPOWER] = { BSP_GPIO_LOW, BSP_GPIO_HIGH },
    [LORA_MODE_SLEEP] = { BSP_GPIO_HIGH, BSP_GPIO_HIGH },
};

static app_lora_t lora;

/*********************************
 * 回调函数
 *********************************/

static void lora_callback(bsp_sig_ch_t GPIO_Pin)
{
    if(GPIO_Pin == BSP_LORA_AUX)
        lora.lora_wakeup = true;
}

void app_lora_init(void)
{
    //DRV init
    //UART+GPIO已集中完成初始化

    lora.lora_wakeup = false;
    lora.current_mode = LORA_MODE_NORMAL;
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

