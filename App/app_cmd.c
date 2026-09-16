/**
 * @file app_cmd.c
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief  调试串口(UART2)命令解析实现
 * @version 0.1
 * @date 2026-09-05
 *
 * @note   命令(大小写不敏感,以回车换行结束):
 *           help             列出帮助
 *           info             打印设备信息(来自 app_config.h)
 *           read <ch>        读取通道电平与当前参数
 *           report <ch>      手动触发一次该通道上报(无需真实信号)
 *           set  <ch> <val>  修改该通道运行时参数(0~255,重启恢复默认)
 *         通道 ch: 0/1/2 对应 PB0/PB1/PB3
 */

#include "app_cmd.h"

#include "app_config.h"      /* 必须最先包含,定义 APP_DEBUG_CMD_ENABLE 等 */
#include "app_main.h"
#include "bsp_dbg_uart.h"
#include "bsp_gpio.h"

#include <stdlib.h>   /* strtol */
#include <string.h>   /* strlen 等 */

#if (APP_DEBUG_CMD_ENABLE == 1)

/*==============================================================================
 * 局部数据
 *============================================================================*/

#define CMD_BUF_SIZE    48U
#define CMD_ARG_MAX     4U

static char    s_line[CMD_BUF_SIZE];     /* 命令行缓冲  */
static uint8_t s_len = 0U;               /* 当前长度    */

/* 通道显示名(与 app_config.h 中顺序一致) */
static const char *const s_ch_name[APP_SIG_CH_COUNT] =
{
    APP_SIG_CH0_NAME,
    APP_SIG_CH1_NAME,
    APP_SIG_CH2_NAME,
};

/*==============================================================================
 * 内部小工具
 *============================================================================*/

static char app_cmd_tolower(char c)
{
    return (c >= 'A' && c <= 'Z') ? (char)(c + ('a' - 'A')) : c;
}

/* 忽略大小写比较(要求 a 全部小写亦可) */
static int app_cmd_ieq(const char *a, const char *b)
{
    while ((*a != '\0') && (*b != '\0'))
    {
        if (app_cmd_tolower(*a) != app_cmd_tolower(*b))
        {
            return 0;
        }
        a++;
        b++;
    }
    return (*a == '\0') && (*b == '\0');
}

/* 把 s_line 按空格/tab 拆成 token */
static uint8_t app_cmd_tokenize(char *argv[], uint8_t max)
{
    uint8_t argc = 0U;
    uint8_t in_word = 0U;
    char   *p = s_line;

    while (*p != '\0')
    {
        if ((*p == ' ') || (*p == '\t'))
        {
            if (in_word)
            {
                *p = '\0';
                in_word = 0U;
            }
        }
        else
        {
            if (!in_word)
            {
                if (argc >= max)
                {
                    break;
                }
                argv[argc++] = p;
                in_word = 1U;
            }
        }
        p++;
    }
    return argc;
}

/* 从 UART2 接收 FIFO 攒一行(直到收到 \r 或 \n) */
static uint8_t app_cmd_getline(void)
{
    uint8_t b;

    while (dbg_rx_available() > 0U)
    {
        if (dbg_rx_get(&b) == 0U)
        {
            continue;
        }

#if APP_CMD_ECHO
        dbg_putchar(b);                 /* 可选:回显输入 */
#endif

        if ((b == '\r') || (b == '\n'))
        {
            if (s_len > 0U)             /* 攒到一整行 */
            {
                s_line[s_len] = '\0';
                s_len = 0U;
                return 1U;
            }
            /* 空行忽略 */
        }
        else if (s_len < (CMD_BUF_SIZE - 1U))
        {
            s_line[s_len++] = (char)b;
        }
    }
    return 0U;
}

/* 打印用法错误 */
static void app_cmd_usage(const char *cmd)
{
    dbg_printf("usage: %s\r\n", cmd);
}

/*==============================================================================
 * 命令实现
 *============================================================================*/

static void app_cmd_help(void)
{
    dbg_printf("Commands:\r\n");
    dbg_printf("  help                 list commands\r\n");
    dbg_printf("  info                 show device info\r\n");
    dbg_printf("  read <ch>            read channel level & param\r\n");
    dbg_printf("  report <ch>          trigger report manually\r\n");
    dbg_printf("  set <ch> <val>       set channel param (0..255)\r\n");
}

static void app_cmd_info(void)
{
    uint8_t ch;

    dbg_printf("Device : %s\r\n", APP_DEVICE_NAME);
    dbg_printf("Model  : %s\r\n", APP_DEVICE_MODEL);
    dbg_printf("FW     : %s\r\n", APP_FW_VERSION);
    dbg_printf("Addr   : 0x%02X\r\n", APP_DEVICE_ADDR);
    dbg_printf("Ch num : %d\r\n", (int)APP_SIG_CH_COUNT);
    dbg_printf("Debug  : %s\r\n", APP_DEBUG_ENABLE ? "ON" : "OFF");

    for (ch = 0U; ch < (uint8_t)APP_SIG_CH_COUNT; ch++)
    {
        const char *lvl = (bsp_gpio_sig_level((bsp_sig_ch_t)ch) == BSP_GPIO_LOW)
                          ? "LOW" : "HIGH";
        dbg_printf("  %-8s param=0x%02X level=%s\r\n",
                   s_ch_name[ch], app_param_get((bsp_sig_ch_t)ch), lvl);
    }
}

static void app_cmd_read(uint8_t argc, char *argv[])
{
    long  ch;
    char *end;

    if (argc < 2U)
    {
        app_cmd_usage("read <ch>");
        return;
    }
    ch = strtol(argv[1], &end, 0);
    if ((*end != '\0') || (ch < 0L) || (ch >= (long)APP_SIG_CH_COUNT))
    {
        dbg_printf("bad ch (0..%d)\r\n", (int)APP_SIG_CH_COUNT - 1);
        return;
    }

    {
        const char *lvl = (bsp_gpio_sig_level((bsp_sig_ch_t)ch) == BSP_GPIO_LOW)
                          ? "LOW" : "HIGH";
        dbg_printf("%s level=%s param=0x%02X\r\n",
                   s_ch_name[ch], lvl, app_param_get((bsp_sig_ch_t)ch));
    }
}

static void app_cmd_report(uint8_t argc, char *argv[])
{
    long  ch;
    char *end;

    if (argc < 2U)
    {
        app_cmd_usage("report <ch>");
        return;
    }
    ch = strtol(argv[1], &end, 0);
    if ((*end != '\0') || (ch < 0L) || (ch >= (long)APP_SIG_CH_COUNT))
    {
        dbg_printf("bad ch (0..%d)\r\n", (int)APP_SIG_CH_COUNT - 1);
        return;
    }

    dbg_printf("manual report %s\r\n", s_ch_name[ch]);
    app_sig_report((bsp_sig_ch_t)ch);
}

static void app_cmd_set(uint8_t argc, char *argv[])
{
    long  ch, val;
    char *end;

    if (argc < 3U)
    {
        app_cmd_usage("set <ch> <val>");
        return;
    }
    ch = strtol(argv[1], &end, 0);
    if ((*end != '\0') || (ch < 0L) || (ch >= (long)APP_SIG_CH_COUNT))
    {
        dbg_printf("bad ch (0..%d)\r\n", (int)APP_SIG_CH_COUNT - 1);
        return;
    }
    val = strtol(argv[2], &end, 0);
    if ((*end != '\0') || (val < 0L) || (val > 255L))
    {
        dbg_printf("bad val (0..255)\r\n");
        return;
    }

    app_param_set((bsp_sig_ch_t)ch, (uint8_t)val);
    dbg_printf("%s param -> 0x%02X\r\n", s_ch_name[ch], app_param_get((bsp_sig_ch_t)ch));
}

/*==============================================================================
 * 对外接口
 *============================================================================*/

void app_cmd_poll(void)
{
    char  *argv[CMD_ARG_MAX];
    uint8_t argc;

    if (!app_cmd_getline())
    {
        return;                     /* 还没有一整行命令 */
    }

    argc = app_cmd_tokenize(argv, CMD_ARG_MAX);
    if (argc == 0U)
    {
        return;
    }

    if (app_cmd_ieq(argv[0], "help") || app_cmd_ieq(argv[0], "?"))
    {
        app_cmd_help();
    }
    else if (app_cmd_ieq(argv[0], "info"))
    {
        app_cmd_info();
    }
    else if (app_cmd_ieq(argv[0], "read"))
    {
        app_cmd_read(argc, argv);
    }
    else if (app_cmd_ieq(argv[0], "report"))
    {
        app_cmd_report(argc, argv);
    }
    else if (app_cmd_ieq(argv[0], "set"))
    {
        app_cmd_set(argc, argv);
    }
    else
    {
        dbg_printf("unknown cmd: %s (help for list)\r\n", argv[0]);
    }
}

#else  /* APP_DEBUG_CMD_ENABLE == 0 */

void app_cmd_poll(void)
{
    /* 发布版:命令解析已由 app_config.h 关闭 */
}

#endif /* APP_DEBUG_CMD_ENABLE */
