/**
 * @file bsp_system.h
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief  系统级驱动:低功耗(Stop)模式、低功耗定时器(LPTIM)周期唤醒、看门狗喂狗
 * @version 0.1
 * @date 2026-09-05
 *
 * @note   低功耗设计(详见实现):
 *         1. IWDG 在 Stop 模式下依然计数(LSI 继续运行),
 *            因此用 LPTIM(时钟源=LSI)约每 1s 唤醒一次来喂狗,保证能无限期休眠。
 *         2. PB0/PB1/PB3 的 EXTI 下降沿会随时把 MCU 从 Stop 唤醒(事件优先)。
 *         3. 从 Stop 唤醒后系统时钟会回到默认 MSI,需要在 bsp 里重建主时钟。
 */
#ifndef BSP_SYSTEM_H
#define BSP_SYSTEM_H

#include "main.h"   /* 引入 HAL,以及 LPTIM_HandleTypeDef 等类型 */

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * 类型/常量
 *============================================================================*/

/* LPTIM 句柄(供 stm32l0xx_it.c 的 LPTIM1_IRQHandler 使用) */
extern LPTIM_HandleTypeDef hlptim;

/* LPTIM 周期唤醒间隔(约 1s,LSI 标称 37kHz;ARR 最大 65535) */
#define BSP_LPTIM_WAKE_PERIOD_MS   1000U
#define BSP_LPTIM_AUTORELOAD       ((37000UL * BSP_LPTIM_WAKE_PERIOD_MS) / 1000UL)

/*==============================================================================
 * 接口
 *============================================================================*/

/**
 * @brief 初始化低功耗相关外设:
 *        - 打开 PWR 时钟
 *        - 把 LSI 选作 LPTIM1 的时钟源
 *        - 启动 LPTIM1 连续计数(周期唤醒)
 * @note  在 bsp_gpio_init() 之后、进入主循环前调用一次
 */
void bsp_system_init(void);

/**
 * @brief 喂独立看门狗(IWDG)
 * @note  应在主循环里周期性调用;从低功耗唤醒后也要尽快喂一次
 */
void bsp_feed_wdg(void);

/**
 * @brief 进入 Stop 低功耗模式,直到被 EXTI(信号)或 LPTIM(周期)唤醒
 * @note  阻塞函数:唤醒返回后已重建主时钟(24MHz PLL),可继续正常运行
 */
void bsp_power_enter_stop(void);

/**
 * @brief 低功耗定时器是否产生过周期唤醒(供调试/心跳打印)
 * @retval 1=是(内部已清除) 0=否
 */
uint8_t bsp_power_lptim_tick(void);

/**
 * @brief 使能/关闭“低功耗下仍可被调试器连接”(仅供调试)
 * @param enable 1=打开 0=关闭
 * @note  通过设置 DBGMCU_CR 的 DBG_SLEEP/DBG_STOP 位,让 CPU 进入
 *        Sleep/Stop 后调试时钟不被关闭;代价是低功耗电流会上升。
 *        正式发布固件请不要调用(或传 0)。
 */
void bsp_system_debug_enable_stop_watch(uint8_t enable);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SYSTEM_H */
