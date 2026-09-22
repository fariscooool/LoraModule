/**
 * @file bsp_gpio.h
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief  外部信号检测 GPIO 驱动接口
 * @version 0.1
 * @date 2026-09-05
 *
 * @note   三路外部信号:
 *           CH0 -> PB0
 *           CH1 -> PB1
 *           CH2 -> PB3
 *         平时外部信号线为高电平(内部上拉),出现低电平即认为有事件,
 *         并配置成 EXTI 下降沿中断用于把 MCU 从低功耗模式唤醒。
 *         电平本身由应用层去读,驱动层只负责初始化/读取/记录唤醒事件。
 *
 * @note   这三路各占一条独立的 EXTI 线(CH0->EXTI0/CH1->EXTI1/CH2->EXTI3),
 *         STM32 的每条 EXTI 线只能属于一个端口,所以不能有别的引脚再占同一线号。
 *         LoRa AUX(PA0)只做普通输入,不配 EXTI(否则会抢走 PB0 的 EXTI0)。
 */
#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * 类型定义
 *============================================================================*/

typedef enum
{
    BSP_SIG_CH0 = 0,    /* PB0 */
    BSP_SIG_CH1,        /* PB1 */
    BSP_SIG_CH2,        /* PB3 */

    BSP_LORA_AUX,       /* PA0 */
    BSP_LORA_M0,        /* PA7 */
    BSP_LORA_M1,        /* PA8 */
    BSP_SIG_CH_MAX
} bsp_sig_ch_t;

/* 真正的外部信号通道个数(CH0/CH1/CH2)。
 * BSP_LORA_AUX/M0/M1 与信号共用了同一张引脚映射表,但它们不是"信号通道",
 * 遍历信号时只能用 BSP_SIG_CH_SIGNAL_MAX,否则会把输出为低的 M0/M1 当成事件。 */
#define BSP_SIG_CH_SIGNAL_MAX   (3U)

typedef enum
{
    BSP_GPIO_LOW  = 0,
    BSP_GPIO_HIGH
} bsp_gpio_level_t;

typedef void (*bsp_gpio_exti_callback_t)(bsp_sig_ch_t GPIO_Pin);

/*==============================================================================
 * 接口
 *============================================================================*/

/**
 * @brief 初始化三路信号输入:
 *        - 输入 + 内部上拉(平时为高)
 *        - EXTI 下降沿中断(用于从 Stop 模式唤醒)
 * @note  在 MX_GPIO_Init() 之后调用,会覆盖 PB0/PB1/PB3 的原始配置
 */
void bsp_gpio_init(void);

/**
 * @brief 读取某一路信号的当前电平
 */
bsp_gpio_level_t bsp_gpio_sig_level(bsp_sig_ch_t ch);

/**
 * @brief 设置某一路信号的电平
 * @note  仅对配置为输出的引脚有效
 */
void bsp_gpio_set_level(bsp_sig_ch_t ch, bsp_gpio_level_t level);

/**
 * @brief 注册某一路信号的 EXTI 回调
 * @param ch       通道号
 * @param callback 回调函数; 传 NULL 表示注销
 * @note  仅对配置了 EXTI 中断的通道有效(现在只有 CH0/CH1/CH2)
 */
void bsp_gpio_register_exti_callback(bsp_sig_ch_t ch, bsp_gpio_exti_callback_t callback);

/**
 * @brief 获取“从低功耗唤醒”时触发过的信号位图(bit0=CH0,bit1=CH1,bit2=CH2)
 * @note  由 EXTI 中断里置位,应用层处理后可用 bsp_gpio_clear_wake_events() 清掉
 */
uint32_t bsp_gpio_get_wake_events(void);

/**
 * @brief 清除唤醒事件位图
 */
void bsp_gpio_clear_wake_events(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_GPIO_H */
