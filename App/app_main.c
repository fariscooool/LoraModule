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
#include "bsp_adc_pwr.h"

/* 编译期一致性检查:配置里的信号通道数必须与 Bsp 层的信号通道数一致 */
#if (APP_SIG_CH_COUNT != BSP_SIG_CH_SIGNAL_MAX)
#error "APP_SIG_CH_COUNT 必须等于 BSP_SIG_CH_SIGNAL_MAX(3)"
#endif

static uint8_t for_debug_time = 0;

/*==============================================================================
 * 内部函数
 *============================================================================*/

/**
 * @brief 通用消抖:不区分高/低电平,只判断"当前电平是否已经稳定"
 *
 *        每 1ms 采样一次;只有当电平连续 APP_SIG_DEBOUNCE_MS 毫秒保持同一种
 *        状态时,才认为消抖完成,并通过 level 返回该稳定电平。
 *        期间一旦电平变化,稳定计时清零并重新开始。
 *
 * @param ch    信号通道
 * @param level [out] 消抖后确认的电平(高/低),仅在返回 1 时有效
 * @retval 1=电平已稳定  0=一直在抖动(超过 APP_SIG_DEBOUNCE_MAX_MS)
 * @note   阻塞函数,最长耗时 APP_SIG_DEBOUNCE_MAX_MS 毫秒;
 *         依赖 SysTick(HAL_Delay),只能在运行态调用,不要放到中断里。
 */
static uint8_t app_sig_debounce(bsp_sig_ch_t ch, bsp_gpio_level_t *level)
{
    bsp_gpio_level_t last;
    bsp_gpio_level_t cur;
    uint32_t stable_ms = 0U;
    uint32_t waited_ms = 0U;

    if (level == NULL)
    {
        return 0U;
    }

    last = bsp_gpio_sig_level(ch);      /* 先取当前电平作为基准 */

    while (waited_ms < APP_SIG_DEBOUNCE_MAX_MS)
    {
        HAL_Delay(1U);                  /* 每 1ms 采样一次 */
        waited_ms++;

        cur = bsp_gpio_sig_level(ch);
        if (cur != last)
        {
            last = cur;                 /* 电平变了 -> 稳定计时重新开始 */
            stable_ms = 0U;
            continue;
        }

        stable_ms++;
        if (stable_ms >= APP_SIG_DEBOUNCE_MS)
        {
            *level = cur;               /* 已连续稳定够久,消抖完成 */
            return 1U;
        }
    }

    return 0U;                          /* 超时:电平一直在抖 */
}

/**
 * @brief 读取三路信号的当前状态:每路都先消抖,再拼成一个状态字
 *
 *        状态字位定义(与 LORA_FUN_SIGNAL 帧的数据字节一致):
 *          bit0 = CH0(PB0)   bit1 = CH1(PB1)   bit2 = CH2(PB3)
 *          位值 = 该路电平:1 = 高电平(无信号),0 = 低电平(有信号)
 *        如果你的协议约定 bit=1 表示"触点闭合(低电平)",把下面的
 *        (uint8_t)level 取反即可。
 *
 * @retval 状态字(bit0 ~ bit2 有效)
 * @note   阻塞函数,最坏耗时 BSP_SIG_CH_SIGNAL_MAX * APP_SIG_DEBOUNCE_MAX_MS;
 *         某一路一直在抖时退回该路的即时电平,保证这一帧仍能上报。
 */
static uint8_t app_sig_state_read(void)
{
    uint8_t state = 0U;
    uint8_t ch;

    for (ch = 0U; ch < BSP_SIG_CH_SIGNAL_MAX; ch++)
    {
        bsp_gpio_level_t level = BSP_GPIO_HIGH;

        if (app_sig_debounce((bsp_sig_ch_t)ch, &level) == 0U)
        {
            /* 消抖超时(一直在抖):退回即时电平,避免这一帧什么都不报 */
            level = bsp_gpio_sig_level((bsp_sig_ch_t)ch);
        }

        state |= (uint8_t)((uint8_t)level << ch);
    }

    return state;
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
    /* 电池电压自检:上电测一次并打印(正式上报可放在 app_task 里按需调用) */
    {
        uint32_t bat_mv = 0U;

        if (bsp_adc_pwr_init() != BSP_ADC_PWR_OK)
        {
            dbg_printf("Battery: ADC init failed.\r\n");
        }
        else if (bsp_adc_pwr_read_mv(&bat_mv) != BSP_ADC_PWR_OK)
        {
            dbg_printf("Battery: ADC read failed.\r\n");
        }
        else
        {
            dbg_printf("Battery: %u mV, %u%%, %s\r\n",
                       (unsigned int)bat_mv,
                       (unsigned int)bsp_adc_pwr_percent(bat_mv),
                       bsp_adc_pwr_is_low(bat_mv) ? "LOW" : "OK");
        }
    }
#endif /* APP_DEBUG_ENABLE */



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
    uint32_t wake;
    uint8_t sig_state = 0U;

    /* 0) 先快照并清掉唤醒事件位图(EXTI 中断里置位,这里消费掉) */
    wake = bsp_gpio_get_wake_events();
    bsp_gpio_clear_wake_events();

    /* 1) 任一路信号出现跳变(EXTI 唤醒位)=> 三路信号全部重新消抖读取,
     *    拼成一个状态字一次性上报:主机看到的是同一时刻的三相快照 */
    if (wake != 0U)
    {
        sig_state = app_sig_state_read();

        app_lora_signal(sig_state);     /* 上报信号帧(数据域 = 三相状态字) */
        dbg_printf("App lora signal: state=0x%02X\r\n", (unsigned int)sig_state);
    }
    /* 电池电压监测:每轮循环都测一次 */
    // debug_gpio_signal[0] = bsp_gpio_sig_level((bsp_sig_ch_t)BSP_SIG_CH0);
    // debug_gpio_signal[1] = bsp_gpio_sig_level((bsp_sig_ch_t)BSP_SIG_CH1);
    // debug_gpio_signal[2] = bsp_gpio_sig_level((bsp_sig_ch_t)BSP_SIG_CH2);
    /* 2) LoRa 主动处理(被动唤醒已取消:AUX 不再是唤醒源) */
    app_lora_process();

    /* 3) 没有任务可做 -> 进入低功耗(Stop)等下一次信号 EXTI */
#if (APP_LOWPOWER_ENABLE == 1)
    /* 上电后前 APP_BOOT_KEEP_RUN_MS 毫秒保持运行(不睡),
     * 方便调试器连接 / 按复位追赶;窗口结束之后才进入 Stop。 */
    if (HAL_GetTick() < (uint32_t)APP_BOOT_KEEP_RUN_MS && for_debug_time == 0)
    {
        HAL_Delay(5U);              /* 短暂延时,期间照常喂狗/响应命令 */
        for_debug_time = 1;
    }
    else
    {
/* 电池电压监测:每轮循环都测一次 */
#if (APP_DEBUG_ENABLE == 1)
        {
            uint32_t bat_mv = 0U;

            if (bsp_adc_pwr_read_mv(&bat_mv) == BSP_ADC_PWR_OK)
            {
                dbg_printf("Battery: %u mV, %u%%, %s\r\n",
                        (unsigned int)bat_mv,
                        (unsigned int)bsp_adc_pwr_percent(bat_mv),
                        bsp_adc_pwr_is_low(bat_mv) ? "LOW" : "OK");
                app_lora_power((uint8_t)bsp_adc_pwr_percent(bat_mv));
            }
        }
#endif
        dbg_printf("Entering low power (Stop) mode.\r\n");
        bsp_power_enter_stop();     /* 正常低功耗(阻塞,直到被唤醒) */
    }
#else
    HAL_Delay(250U);                /* 调试:不进低功耗,一直运行 */
#endif

    /* 唤醒后回到循环顶部:喂狗、处理事件、再休眠 */
}



