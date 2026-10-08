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
static app_status_t app_status = STATE_IDLE;
static uint8_t matched_state = 0U;

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
    app_status = STATE_IDLE;
    matched_state = 1U;

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
        .detail.addr_high = (uint8_t)((APP_DEVICE_ADDR >> 8) & 0xFFU),
        .detail.addr_low = (uint8_t)(APP_DEVICE_ADDR & 0xFFU),
        .detail.sped.bits.air_rate = AIR_2_4K,
        .detail.sped.bits.ttl_rate = BAUD_9600,
        .detail.sped.bits.parity = PARITY_8N1,
        .detail.channel = APP_LORA_FREQ_CH,   /* 根据实际情况初始化 */
        .detail.option.bits.power = TX_POWER_20DBM,
        .detail.option.bits.fec = FEC_DISABLE,
        .detail.option.bits.wakeup_time = WAKEUP_250MS,
        .detail.option.bits.io_drv_mode = IO_DRV_MODE_PUSH_PULL,
        .detail.option.bits.fixed_point_trans = FIXED_POINT_TRANS_DISABLE,
    };

    if(app_lora_cfg_reg_verify(lora_cfg))
    {
        dbg_printf("LoRa configuration verified successfully.\r\n");
    }
    else
    {
        dbg_printf("LoRa configuration verification failed.\r\n");
    }

    dbg_printf("\r\n==== %s boot (FW %s) ====\r\n", APP_DEVICE_NAME, APP_FW_VERSION);

    /* 电池电压 ADC:上报帧要带电量,正式版也必须初始化(不是调试专属) */
    if (bsp_adc_pwr_init() != BSP_ADC_PWR_OK)
    {
        dbg_printf("Battery: ADC init failed.\r\n");
    }

#if (APP_DEBUG_ENABLE == 1)
    /* 上电自检:测一次并打印 */
    {
        uint32_t bat_mv = 0U;

        if (bsp_adc_pwr_read_mv(&bat_mv) != BSP_ADC_PWR_OK)
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

    switch(app_status)
    {
    case STATE_IDLE:

    {
        /* 如果尚未匹配,则进入配对状态 */
        if(matched_state == 0U)
        {
            app_status = STATE_MATCHING;
        }
        /* 上电后前 APP_BOOT_KEEP_RUN_MS 毫秒保持运行(不睡),
         * 方便调试器连接 / 按复位追赶;窗口结束之后才进入 Stop。 */
        if (HAL_GetTick() < (uint32_t)APP_BOOT_KEEP_RUN_MS && for_debug_time == 0)
        {
            HAL_Delay(5U);              /* 短暂延时,期间照常响应命令 */
            for_debug_time = 1;
            dbg_printf("Boot keep run time elapsed, entering low power soon.\r\n");
            app_status = STATE_RUNNING;
        }
        /* 空闲状态下的处理逻辑 */
    }break;

    case STATE_MATCHING:

    {
        /* 配对状态下的处理逻辑 */
    }break;

    case STATE_RUNNING:

    {
        /* 运行状态下的处理逻辑:
         * 事件 -> 读三相状态 + 测电量 -> 提交一轮上报(信号帧+电量帧);
         * 方案B2:两帧连发,一个窗口收双 ACK,缺帧重发;
         * 发送/等ACK/重试/超时由上行事务在 app_lora_process() 里推进;
         * 两帧都确认(或重试用尽)之前不进 Stop。 */

        /* 0) 先快照并清掉唤醒事件位图(EXTI 中断里置位,这里消费掉) */
        wake = bsp_gpio_get_wake_events();
        bsp_gpio_clear_wake_events();

        /* 1) 任一路信号出现跳变(EXTI 唤醒位)=> 三路信号全部重新消抖读取:
        *    全部为低(状态字=0)表示挂接完成 -> 0xAA,否则 0x55;
        *    与最新电量一起提交,由后台事务负责发送/确认/重试 */
        if (wake != 0U)
        {
            uint32_t bat_mv = 0U;
            uint8_t  pwr_pct = 0xFFU;   /* 0xFF = 电量读取失败/无效(主机按无效处理) */

            sig_state = app_sig_state_read();

            if (bsp_adc_pwr_read_mv(&bat_mv) == BSP_ADC_PWR_OK)
            {
                pwr_pct = (uint8_t)bsp_adc_pwr_percent(bat_mv);
                dbg_printf("Battery: %u mV, %u%%, %s\r\n",
                        (unsigned int)bat_mv,
                        (unsigned int)pwr_pct,
                        bsp_adc_pwr_is_low(bat_mv) ? "LOW" : "OK");
            }
            else
            {
                dbg_printf("Battery: read failed\r\n");
            }

            dbg_printf("App lora signal: state=0x%02X\r\n", (unsigned int)sig_state);

            app_lora_uplink_status((sig_state == 0U) ? (uint8_t)APP_SIG_HOOKED_OK
                                                     : (uint8_t)APP_SIG_HOOKED_FAIL,
                                   pwr_pct);
        }

        /* 2) 交付事务未完成(等 ACK/重发中)不进低功耗,空转 1ms 让下一轮继续推进 */


#if (APP_LOWPOWER_ENABLE == 1)
        if (app_lora_uplink_busy() != 0U)
        {
            HAL_Delay(1U);              /* 交付未完成:保持清醒,下一轮继续推进 */
        }
        else
        {
            dbg_printf("Entering low power (Stop) mode.\r\n");
            bsp_power_enter_stop();     /* 正常低功耗(阻塞,直到被唤醒) */
            dbg_printf("Exited low power (Stop) mode.\r\n");
        }
#else
    HAL_Delay(250U);                /* 调试:不进低功耗,一直运行 */
#endif 
    } break;

    case STATE_ERROR:

    {
        /* 错误状态下的处理逻辑 */
    }break;

    default:

        break;

    }

    app_lora_process();

    /* 唤醒后回到循环顶部:处理事件、再休眠 */
}



