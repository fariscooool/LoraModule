/**
 * @file bsp_system.c
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief  系统级驱动实现:低功耗(Stop)模式(纯 EXTI 事件唤醒)
 * @version 0.1
 * @date 2026-09-05
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp_system.h"

/* main.c 中由 CubeMX 生成的系统时钟配置(从 Stop 唤醒后需要重建主时钟) */
void SystemClock_Config(void);

/*==============================================================================
 * 接口实现
 *============================================================================*/

void bsp_power_enter_stop(void)
{
    /* SysTick 在 Stop 下会停走,HAL tick 先挂起,唤醒后再恢复 */
    HAL_SuspendTick();

    /*
     * 进入 Stop 模式:
     *  - 低功耗调节器(PWR_LOWPOWERREGULATOR_ON)
     *  - 唯一唤醒源: PB0/PB1/PB3 的 EXTI 下降沿
     *  - 进入后高频时钟全部停止;LSI 已在 SystemClock_Config() 里关闭,
     *    且没有 IWDG/LPTIM/RTC,所以 Stop 期间没有任何低频时钟在跑
     */
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

    /* ---------- 唤醒回来(EXTI 中断已执行完,唤醒位图已置位) ---------- */

    /* Stop 唤醒后系统时钟回到默认 MSI,重建 24MHz(PLL)主时钟 */
    SystemClock_Config();

    /* 恢复 SysTick 时间基准 */
    HAL_ResumeTick();
}

void bsp_system_debug_enable_stop_watch(uint8_t enable)
{
    if (enable)
    {
        /* 让 CPU 在 Sleep/Stop 模式下仍保持调试时钟,调试器才能连接/看停点
         * 注意:会略微增加低功耗电流,仅供调试,正式发布不要调用 */
        DBGMCU->CR |= (DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP);
    }
    else
    {
        DBGMCU->CR &= ~(DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP);
    }
}



