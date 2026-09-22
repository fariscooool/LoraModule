/**
 * @file app_main.c
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief  应用层主逻辑:
 *         - 初始化各驱动
 *         - 主循环:喂狗 -> 检测外部信号(PB0/PB1/PB3 低电平)并上报 LoRa
 *                  -> 空闲时进入低功耗(Stop)
 * @version 0.1
 * @date 2026-09-05
 *
 * @note   分层约定:
 *           App/   应用层(本文件):业务流程、报文内容、低功耗策略
 *           Bsp/   驱动层(bsp_*):只提供外设能力,不含业务
 */

/* 设备/协议/调试等配置集中放在 app_config.h,务必最先包含
 * (其中 APP_DEBUG_ENABLE 会覆盖 bsp_dbg_uart.h 的默认值) */
#include "app_config.h"

#include "app_main.h"
#include "app_lora_procotol.h"

#include "bsp_dbg_uart.h"
#include "bsp_gpio.h"
#include "bsp_lora_uart.h"
#include "bsp_system.h"

/* 编译期一致性检查:配置里的信号通道数必须与 Bsp 层的信号通道数一致 */
#if (APP_SIG_CH_COUNT != BSP_SIG_CH_SIGNAL_MAX)
#error "APP_SIG_CH_COUNT 必须等于 BSP_SIG_CH_SIGNAL_MAX(3)"
#endif

/*==============================================================================
 * 内部函数
 *============================================================================*/

/**
 * @brief 消抖确认:连续 DEBOUNCE_MS 毫秒内稳定为低电平才算有效
 * @retval 1=确认低电平  0=抖动/已恢复高
 */
static uint8_t app_sig_debounce_confirm(bsp_sig_ch_t ch)
{
    uint32_t cnt = 0U;
    uint32_t t;

    for (t = 0U; t < APP_SIG_DEBOUNCE_MS; t++)
    {
        if (bsp_gpio_sig_level(ch) == BSP_GPIO_LOW)
        {
            cnt++;
        }
        HAL_Delay(1U);          /* 每隔 1ms 采样一次,运行态下 SysTick 有效 */
    }
    return (cnt >= (APP_SIG_DEBOUNCE_MS - 2U)) ? 1U : 0U;
}


/*==============================================================================
 * 对外接口(供 main.c 调用)
 *============================================================================*/

void app_init(void)
{
    dbg_printf("\r\n==== %s boot (FW %s) ====\r\n", APP_DEVICE_NAME, APP_FW_VERSION);

    /* 驱动层初始化顺序:
     * 1) 串口驱动(需要 CubeMX 已 MX_xxx_Init)
     * 2) 信号输入 GPIO + EXTI 唤醒(PB0/PB1/PB3)
     * 3) LoRa 应用层
     * 注:LPTIM 周期唤醒已取消,不需要 bsp_system_init() */

    /* 调试串口:发布版(APP_DEBUG_ENABLE/CMD 都为 0)时不使能,省一个外设/中断 */
#if ((APP_DEBUG_ENABLE == 1) || (APP_DEBUG_CMD_ENABLE == 1))
    dbg_uart_init();
#endif
    lora_uart_init();
    bsp_gpio_init();
    app_lora_init();        /* LoRa 应用层:模式初始化 */

    // LORA 参数配置
    lora_reg_parm_cfg_t lora_cfg = {
        .detail.addr_high = 0x00,
        .detail.addr_low = 0x01,
        .detail.sped.bits.air_rate = AIR_2_4K,
        .detail.sped.bits.ttl_rate = BAUD_9600,
        .detail.sped.bits.parity = PARITY_8N1,
        .detail.channel = 0x3C,   /* 根据实际情况初始化 */
        .detail.option.bits.reserved = 0,
        .detail.option.bits.wakeup_time = WAKEUP_250MS,
        .detail.option.bits.io_drv_mode = IO_DRV_MODE_PUSH_PULL,
        .detail.option.bits.fixed_point_trans = FIXED_POINT_TRANS_DISABLE,
    };

    if(app_lora_cfg_reg_verify(lora_cfg, sizeof(lora_cfg.data)))
    {
        dbg_printf("LoRa configuration verified successfully.\r\n");
    }
    else
    {
        dbg_printf("LoRa configuration verification failed.\r\n");
    }



#if (APP_DEBUG_ENABLE == 1)
    /* 调试期:让 MCU 进入 Sleep/Stop 后调试器仍能连接/打断点
     * (代价是低功耗电流略增;正式发布版不会走到这里) */
    bsp_system_debug_enable_stop_watch(1U);
#endif

#if (APP_LOWPOWER_ENABLE == 1)
    dbg_printf("Init done, keep run %d ms then low power.\r\n", (int)APP_BOOT_KEEP_RUN_MS);
#else
    dbg_printf("Init done, LOW POWER DISABLED (debug).\r\n");
#endif
}

void app_deinit(void)
{
    /* 暂无需要释放的资源 */
}

// uint8_t debug_gpio_signal[BSP_SIG_CH_SIGNAL_MAX] = {0U};
void app_task(void)
{
    static uint8_t s_prev_low[BSP_SIG_CH_SIGNAL_MAX] = {0U};   /* 上一轮电平,用于检测下降沿 */
    uint32_t wake;
    uint8_t ch;

    /* 0) 先快照并清掉唤醒事件位图(EXTI 中断里置位,这里消费掉) */
    wake = bsp_gpio_get_wake_events();
    bsp_gpio_clear_wake_events();

    /* 1) 检测外部信号:出现“高 -> 低”的下降沿且消抖确认,则上报 LoRa */
    for (ch = 0U; ch < BSP_SIG_CH_SIGNAL_MAX; ch++)
    {
        /* 只看本通道自己的事件位:否则任一通道有事件就会让三路全部误判 */
        if ((bsp_gpio_sig_level((bsp_sig_ch_t)ch) == BSP_GPIO_LOW) ||
            ((wake & (1UL << ch)) != 0U))
        {
            if (s_prev_low[ch] == 0U)          /* 之前是高,现在变低 = 事件 */
            {
                s_prev_low[ch] = 1U;
                if (app_sig_debounce_confirm((bsp_sig_ch_t)ch))
                {
                    app_lora_signal((uint8_t)ch);   /* 上报信号帧 */
                    dbg_printf("App lora signal: %d\r\n", (int)ch);
                }
            }
        }
        else
        {
            s_prev_low[ch] = 0U;               /* 恢复高电平,等待下一次事件 */
        }
    }
    // debug_gpio_signal[0] = bsp_gpio_sig_level((bsp_sig_ch_t)BSP_SIG_CH0);
    // debug_gpio_signal[1] = bsp_gpio_sig_level((bsp_sig_ch_t)BSP_SIG_CH1);
    // debug_gpio_signal[2] = bsp_gpio_sig_level((bsp_sig_ch_t)BSP_SIG_CH2);
    /* 2) LoRa 主动处理(被动唤醒已取消:AUX 不再是唤醒源) */
    app_lora_process();

    /* 3) 没有任务可做 -> 进入低功耗(Stop)等下一次信号 EXTI */
#if (APP_LOWPOWER_ENABLE == 1)
    /* 上电后前 APP_BOOT_KEEP_RUN_MS 毫秒保持运行(不睡),
     * 方便调试器连接 / 按复位追赶;窗口结束之后才进入 Stop。 */
    if (HAL_GetTick() < (uint32_t)APP_BOOT_KEEP_RUN_MS)
    {
        HAL_Delay(5U);              /* 短暂延时,期间照常喂狗/响应命令 */
    }
    else
    {
        dbg_printf("Entering low power (Stop) mode.\r\n");
        bsp_power_enter_stop();     /* 正常低功耗(阻塞,直到被唤醒) */
    }
#else
    HAL_Delay(250U);                /* 调试:不进低功耗,一直运行 */
#endif

    /* 唤醒后回到循环顶部:喂狗、处理事件、再休眠 */
}



