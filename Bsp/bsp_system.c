/**
 * @file bsp_system.c
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief  系统级驱动实现:低功耗(Stop)模式、LPTIM 周期唤醒、看门狗喂狗
 * @version 0.1
 * @date 2026-09-05
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp_system.h"

#include "iwdg.h"    /* hiwdg */

/* main.c 中由 CubeMX 生成的系统时钟配置(从 Stop 唤醒后需要重建主时钟) */
void SystemClock_Config(void);

/*==============================================================================
 * 局部变量
 *============================================================================*/

LPTIM_HandleTypeDef hlptim;

static volatile uint8_t s_lptim_tick = 0U;   /* LPTIM 周期唤醒标志 */

/*==============================================================================
 * HAL MSP: LPTIM1 底层初始化(开时钟/复位/使能 NVIC)
 *   HAL_LPTIM_Init() 里在句柄首次使用时自动回调本函数
 *============================================================================*/

void HAL_LPTIM_MspInit(LPTIM_HandleTypeDef *h)
{
    if (h->Instance == LPTIM1)
    {
        __HAL_RCC_LPTIM1_CLK_ENABLE();
        __HAL_RCC_LPTIM1_FORCE_RESET();
        __HAL_RCC_LPTIM1_RELEASE_RESET();

        /* LPTIM 中断:用于周期唤醒(优先级低于 EXTI 信号中断) */
        HAL_NVIC_SetPriority(LPTIM1_IRQn, 2U, 0U);
        HAL_NVIC_EnableIRQ(LPTIM1_IRQn);
    }
}

/*==============================================================================
 * HAL 弱回调覆盖: LPTIM 每次计数到 0(AutoReload Match)触发
 *   这里只做一个标志,真正的“喂狗/回到休眠”由主循环处理
 *============================================================================*/

void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *h)
{
    (void)h;
    s_lptim_tick = 1U;
}

/*==============================================================================
 * 接口实现
 *============================================================================*/

void bsp_system_init(void)
{
    RCC_PeriphCLKInitTypeDef pclk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();

    /* 把 LSI 选作 LPTIM1 的时钟源(LSI 因 IWDG 已开启,Stop 下仍运行) */
    pclk.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1;
    pclk.LptimClockSelection  = RCC_LPTIM1CLKSOURCE_LSI;
    if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK)
    {
        Error_Handler();
    }

    /* LPTIM1 初始化:内部时钟计数、软件触发、立即更新 */
    hlptim.Instance                 = LPTIM1;
    hlptim.Init.Clock.Source        = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
    hlptim.Init.Clock.Prescaler     = LPTIM_PRESCALER_DIV1;
    hlptim.Init.Trigger.Source      = LPTIM_TRIGSOURCE_SOFTWARE;
    hlptim.Init.OutputPolarity      = LPTIM_OUTPUTPOLARITY_HIGH;
    hlptim.Init.UpdateMode          = LPTIM_UPDATE_IMMEDIATE;
    hlptim.Init.CounterSource       = LPTIM_COUNTERSOURCE_INTERNAL;
    if (HAL_LPTIM_Init(&hlptim) != HAL_OK)
    {
        Error_Handler();
    }

    /* 连续计数模式 + 中断:每到 0 自动重载并产生周期唤醒 */
    if (HAL_LPTIM_Counter_Start_IT(&hlptim, BSP_LPTIM_AUTORELOAD) != HAL_OK)
    {
        Error_Handler();
    }
}

void bsp_feed_wdg(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}

void bsp_power_enter_stop(void)
{
    /* 进低功耗前先喂一次狗,保证从“上一次喂狗”开始有足够的窗口 */
    bsp_feed_wdg();

    /* SysTick 在 Stop 下会停走,HAL tick 先挂起,唤醒后再恢复 */
    HAL_SuspendTick();

    /*
     * 进入 Stop 模式:
     *  - 低功耗调节器(PWR_LOWPOWERREGULATOR_ON)
     *  - 唤醒源: PB0/PB1/PB3 EXTI 下降沿、LPTIM1 周期(约1s)
     *  - 进入后所有高频时钟停止,LSI/IWDG/LPTIM 继续运行
     */
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

    /* ---------- 唤醒回来 ---------- */

    /* Stop 唤醒后系统时钟回到默认 MSI,重建 24MHz(PLL)主时钟 */
    SystemClock_Config();

    /* 恢复 SysTick 时间基准 */
    HAL_ResumeTick();

    /* 刚唤醒,尽快喂一次狗(若因 EXTI 事件被叫醒,主循环会继续处理) */
    bsp_feed_wdg();
}

uint8_t bsp_power_lptim_tick(void)
{
    uint8_t t;
    t = s_lptim_tick;
    s_lptim_tick = 0U;
    return t;
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



