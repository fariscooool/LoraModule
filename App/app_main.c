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
#include "app_cmd.h"

#include "bsp_dbg_uart.h"
#include "bsp_gpio.h"
#include "bsp_lora_uart.h"
#include "bsp_system.h"

/*==============================================================================
 * 上报帧结构(协议宏定义见 app_config.h 的“上报帧协议”一节)
 *   [0] [1] 帧头    FRM_HEAD0 FRM_HEAD1
 *   [2]      类型    FRM_TYPE_REPORT
 *   [3]      通道    0/1/2
 *   [4]      参数    该通道运行时参数(默认=app_config.h 的 APP_SIG_CHx_PARAM)
 *   [5]      校验    前 5 字节累加和(取低 8 位)
 *   [6]      帧尾    FRM_TAIL
 * 总长 7 字节。
 *============================================================================*/

typedef struct
{
    uint8_t head0;
    uint8_t head1;
    uint8_t type;
    uint8_t ch;
    uint8_t param;
    uint8_t sum;
    uint8_t tail;
} sig_report_frame_t;                     /* 长度 7 */

/* 通道 -> 运行时“参数”表(默认取自 app_config.h 的宏;
 * 运行中可由调试串口命令 set 修改,重启恢复默认) */
static uint8_t g_ch_param[BSP_SIG_CH_MAX] =
{
    APP_SIG_CH0_PARAM,
    APP_SIG_CH1_PARAM,
    APP_SIG_CH2_PARAM,
};

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

/**
 * @brief 组上报帧并通过 UART1(LPUART1) 下发给 LoRa 模块
 * @note  对外导出:既供信号检测调用,也供调试串口命令手动触发
 */
void app_sig_report(bsp_sig_ch_t ch)
{
    sig_report_frame_t frm;
    uint8_t i;

    if (ch >= BSP_SIG_CH_MAX)
    {
        return;
    }

    frm.head0 = FRM_HEAD0;
    frm.head1 = FRM_HEAD1;
    frm.type  = FRM_TYPE_REPORT;
    frm.ch    = (uint8_t)ch;
    frm.param = g_ch_param[ch];

    /* 累加和 = 前 5 字节低 8 位 */
    frm.sum = 0U;
    {
        const uint8_t *p = (const uint8_t *)&frm;
        for (i = 0U; i < 5U; i++)
        {
            frm.sum = (uint8_t)(frm.sum + p[i]);
        }
    }
    frm.tail = FRM_TAIL;

    dbg_printf("[APP] CH%d low -> send to LoRa: ", (int)ch);
    for (i = 0U; i < sizeof(frm); i++)
    {
        dbg_printf("%02X ", ((const uint8_t *)&frm)[i]);
    }
    dbg_printf("\r\n");

    /* 驱动层负责把这一帧完整发出去(阻塞,发完才返回) */
    lora_send((const uint8_t *)&frm, (uint16_t)sizeof(frm));
}

/**
 * @brief 读取某通道运行时参数
 */
uint8_t app_param_get(bsp_sig_ch_t ch)
{
    if (ch >= BSP_SIG_CH_MAX)
    {
        return 0U;
    }
    return g_ch_param[ch];
}

/**
 * @brief 修改某通道运行时参数(0~255)
 */
void app_param_set(bsp_sig_ch_t ch, uint8_t value)
{
    if (ch >= BSP_SIG_CH_MAX)
    {
        return;
    }
    g_ch_param[ch] = value;
}

/**
 * @brief 处理 LoRa 模块返回的数据(这里简单地以 HEX 打印便于观察)
 */
static void app_service_lora_rx(void)
{
    uint8_t b;
    uint8_t n = 0U;

    while (lora_rx_available() > 0U)
    {
        if (lora_rx_get(&b))
        {
            if (n == 0U)
            {
                dbg_printf("[APP] LoRa reply: ");
            }
            dbg_printf("%02X ", b);
            n++;
            if (n >= 32U)
            {
                break;
            }
        }
    }
    if (n > 0U)
    {
        dbg_printf("\r\n");
    }
}

/*==============================================================================
 * 对外接口(供 main.c 调用)
 *============================================================================*/

void app_init(void)
{
    dbg_printf("\r\n==== %s boot (FW %s) ====\r\n", APP_DEVICE_NAME, APP_FW_VERSION);

    /* 驱动层初始化顺序:
     * 1) 串口驱动(需要 CubeMX 已 MX_xxx_Init)
     * 2) 信号输入 GPIO + EXTI 唤醒
     * 3) 低功耗相关(LPTIM 周期唤醒/时钟源) */

    /* 调试串口:发布版(APP_DEBUG_ENABLE/CMD 都为 0)时不使能,省一个外设/中断 */
#if ((APP_DEBUG_ENABLE == 1) || (APP_DEBUG_CMD_ENABLE == 1))
    dbg_uart_init();
#endif
    lora_uart_init();
    bsp_gpio_init();
    bsp_system_init();

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

void app_task(void)
{
    static uint8_t s_prev_low[BSP_SIG_CH_MAX] = {0U};   /* 上一轮电平,用于检测下降沿 */
    uint8_t ch;

    /* 1) 主循环周期性喂狗(防止运行期被复位) */
    bsp_feed_wdg();

    /* 2) 检测外部信号:出现“高 -> 低”的下降沿且消抖确认,则上报 LoRa */
    for (ch = 0U; ch < BSP_SIG_CH_MAX; ch++)
    {
        if (bsp_gpio_sig_level((bsp_sig_ch_t)ch) == BSP_GPIO_LOW)
        {
            if (s_prev_low[ch] == 0U)          /* 之前是高,现在变低 = 事件 */
            {
                s_prev_low[ch] = 1U;
                if (app_sig_debounce_confirm((bsp_sig_ch_t)ch))
                {
                    bsp_gpio_clear_wake_events();
                    app_sig_report((bsp_sig_ch_t)ch);
                }
            }
        }
        else
        {
            s_prev_low[ch] = 0U;               /* 恢复高电平,等待下一次事件 */
        }
    }

    /* 3) 处理 LoRa 模块返回的数据(简单以 HEX 打印,便于观察) */
    app_service_lora_rx();

    /* 4) 调试串口命令解析(help/info/read/report/set,
     *    发布时由 app_config.h 中 APP_DEBUG_CMD_ENABLE=0 整段关闭) */
    app_cmd_poll();

#if APP_DEBUG_HEARTBEAT
    /* 5) LPTIM 周期唤醒一次就打一个点,方便刚开始观察“休眠-唤醒-休眠” */
    if (bsp_power_lptim_tick())
    {
        dbg_printf("[APP] lptim wake\r\n");
    }
#endif

    /* 6) 没有任务可做 -> 进入低功耗(Stop)等下一次事件(信号 EXTI 或 LPTIM 周期) */
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
        // bsp_power_enter_stop();     /* 正常低功耗(阻塞,直到被唤醒) */
    }
#else
    HAL_Delay(250U);                /* 调试:不进低功耗,一直运行 */
#endif

    /* 唤醒后回到循环顶部:喂狗、处理事件、再休眠 */
}



