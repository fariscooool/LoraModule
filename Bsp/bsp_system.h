/**
 * @file bsp_system.h
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief  系统级驱动:低功耗(Stop)模式
 * @version 0.1
 * @date 2026-09-05
 *
 * @note   低功耗设计(详见实现):
 *         1. 只有 PB0/PB1/PB3 的 EXTI 下降沿会把 MCU 从 Stop 唤醒;
 *            醒来后由应用层读电平、消抖、主动上报,处理完继续睡。
 *         2. 已取消 LPTIM/RTC 周期唤醒,也没有看门狗:LPTIM 代码已删除,
 *            LSI 已在 SystemClock_Config() 中关闭,Stop 下无低频时钟。
 *         3. 从 Stop 唤醒后系统时钟会回到默认 MSI,需要在 bsp 里重建主时钟。
 */
#ifndef BSP_SYSTEM_H
#define BSP_SYSTEM_H

#include "main.h"   /* 引入 HAL 类型(GPIO_TypeDef 等) */

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * 接口
 *============================================================================*/

/**
 * @brief  进入 Stop 低功耗模式,直到被外部信号(PB0/PB1/PB3)的 EXTI 唤醒
 * @note   阻塞函数:唤醒返回后已重建主时钟(24MHz PLL),可继续正常运行
 */
void bsp_power_enter_stop(void);

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
