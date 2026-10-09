/**
 * @file app_lora_procotol.c
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief LoRa 协议处理:数据拆包解析 / 装包发送(待自己实现)
 * @version 0.1
 * @date 2026-09-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#include <stdint.h>
#include <string.h>
#include <stddef.h>

#include "app_config.h"
#include "app_lora.h"
#include "app_lora_procotol.h"
#include "bsp_dbg_uart.h"   /* dbg_printf(发布版编译为空操作) */
#include "main.h"

/*==============================================================================
 * 三、上报帧协议(v1.6:合并快照帧 + UID32 设备标识 + ACK,详见 app_config.h "四")
 *
 * 上行帧(设备 -> 主机,状态快照:信号+电量合并):
 *   [0] 0x5A   [1] 本机地址   [2] LEN=0x0C(12)
 *   [3] FUN=0x03   [4..7] UID32(小端)   [8] SEQ   [9] SIG   [10] PWR   [11] CRC8
 *
 * 下行帧(主机 -> 设备):
 *   ACK : [0]0x5A [1]目标设备地址 [2]LEN=0x0B [3]FUN|0x80 [4..7]目标UID32
 *         [8]CORR [9]CODE [10]CRC8
 *   命令: [0]0x5A [1]目标设备地址 [2]LEN=0x09 [3]FUN [4..7]目标UID32 [8]CRC8
 *============================================================================*/

static uint8_t *signal_status = NULL; // ABC相开关状态
static uint8_t *power_status = NULL; // 电池电量

/*********************************
 * 静态函数
 *********************************/
// CRC 8-CCITT (多项式 0x07)
static uint8_t lora_crc8_cal(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x07; /* CRC-8-CCITT 多项式 */
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* 设备唯一标识短码:96bit UID 经 FNV-1a 派生的 32bit(首次调用时读取并缓存)
 * 用途:随每帧下发,主机建立"本套设备白名单",同信道其他套设备的帧直接过滤 */
uint32_t app_lora_uid32(void)
{
    static uint32_t cached = 0U;
    static uint8_t  valid  = 0U;

    if (valid == 0U)
    {
        uint32_t w[3];
        uint32_t h = 2166136261UL;      /* FNV-1a 32bit 初值 */
        uint8_t  i;
        uint8_t  k;

        w[0] = HAL_GetUIDw0();
        w[1] = HAL_GetUIDw1();
        w[2] = HAL_GetUIDw2();

        for (i = 0U; i < 3U; i++)
        {
            for (k = 0U; k < 4U; k++)
            {
                h ^= (uint8_t)((w[i] >> (8U * k)) & 0xFFU);
                h *= 16777619UL;        /* FNV-1a 32bit 质数 */
            }
        }

        cached = h;
        valid  = 1U;
    }

    return cached;
}

/*********************************
 * 二、上行交付事务(一帧快照 + ACK 确认 + 超时重发)
 *
 * 注:本章节行为由 app_config.h 的 APP_LORA_ACK_ENABLE 控制。
 *     =0(简化版,当前交付):快照帧发出即结束,不等 ACK/不重发/不占事务;
 *     =1(完整版):下述状态机全流程生效。代码始终保留,仅编译期裁剪。
 *
 * 一轮上报 = 1 帧状态快照(0x03:信号+电量+UID32+SEQ);
 * 主机回 ACK(FUN=0x83,CORR=被确认的 SEQ,CODE=0 为成功);
 *   - 窗口内收到  -> 本轮交付成功
 *   - 窗口超时    -> 换新 SEQ 重发,最多 APP_LORA_RETRY_MAX 次
 *   - 重试用尽/被拒 -> 记失败统计,不影响下一次上报
 * 事务进行中再次提交快照 -> 记为"最新快照",本轮结束后自动补发一轮
 *********************************/

typedef enum
{
    UPLINK_IDLE = 0,    /* 无进行中的事务      */
    UPLINK_BACKOFF,     /* 退避中,到期发送帧   */
    UPLINK_WAIT_ACK,    /* 已发送,窗口内等 ACK */
} uplink_state_t;

static struct
{
    uplink_state_t state;
    uint8_t  round_sig;       /* 本轮快照(发送用:触发信号) */
    uint8_t  round_pwr;       /* 本轮快照(发送用:电量百分比) */
    uint8_t  last_sig;        /* 最近一次已知值(供点名应答) */
    uint8_t  last_pwr;
    uint8_t  latest_sig;      /* 事务进行中收到的新快照 */
    uint8_t  latest_pwr;
    uint8_t  reshoot;         /* 1=本轮结束后立即再发一轮新快照 */
    uint8_t  seq;             /* 在途帧的 SEQ(等 ACK 用) */
    uint8_t  retry_left;      /* 剩余重发次数 */
    uint8_t  backoff_ms;      /* 本次退避时长(ms) */
    uint32_t tick;            /* 进入当前状态/开始窗口的时刻 */
    uint8_t  seq_next;        /* 共享 SEQ 计数器:每发一帧 +1 */

    app_lora_uplink_stats_t stats;
} s_uplink;

/* 组一帧状态快照(0x03:地址 + UID32 + SEQ + 信号 + 电量)并立即发送
 * 注:lora_send 为阻塞语义(DMA 发完才返回),tx 放栈上是安全的 */
static void uplink_send_snapshot(uint8_t seq, uint8_t sig, uint8_t pwr)
{
    uint8_t  tx[LORA_FRAME_UPLINK_LEN];
    uint32_t uid = app_lora_uid32();

    tx[0]  = LORA_FRAME_HEAD;
    tx[1]  = (uint8_t)APP_DEVICE_ADDR;
    tx[2]  = (uint8_t)LORA_FRAME_UPLINK_LEN;
    tx[3]  = LORA_FUN_STATUS;
    tx[4]  = (uint8_t)(uid & 0xFFU);
    tx[5]  = (uint8_t)((uid >> 8U) & 0xFFU);
    tx[6]  = (uint8_t)((uid >> 16U) & 0xFFU);
    tx[7]  = (uint8_t)((uid >> 24U) & 0xFFU);
    tx[8]  = seq;
    tx[9]  = sig;
    tx[10] = pwr;
    tx[11] = lora_crc8_cal(tx, 11U);

    app_lora_send_bytes(tx, (uint16_t)sizeof(tx));
}

#if (APP_LORA_ACK_ENABLE == 0)
/* 简化版:一轮 = 一帧快照,发出即结束(不等 ACK、不重发、不占事务) */
static void uplink_send_round_now(uint8_t sig, uint8_t power)
{
    s_uplink.round_sig = sig;
    s_uplink.round_pwr = power;
    s_uplink.state     = UPLINK_IDLE;       /* 不占事务:发完即空闲,可进 Stop */

    s_uplink.seq = s_uplink.seq_next++;
    uplink_send_snapshot(s_uplink.seq, sig, power);

    dbg_printf("[UPLINK] tx STATUS seq=%u sig=0x%02X pwr=%u (ACK disabled)\r\n",
               (unsigned int)s_uplink.seq, (unsigned int)sig, (unsigned int)power);
}
#endif

#if (APP_LORA_ACK_ENABLE == 1)
/* 开始一轮:快照待发,重试额度重置 */
static void uplink_begin_round(uint8_t sig, uint8_t power)
{
    s_uplink.round_sig = sig;
    s_uplink.round_pwr = power;
    s_uplink.retry_left = (uint8_t)APP_LORA_RETRY_MAX;
    s_uplink.backoff_ms = (uint8_t)APP_LORA_TX_BACKOFF_MS;
    s_uplink.tick = HAL_GetTick();
    s_uplink.state = UPLINK_BACKOFF;

    dbg_printf("[UPLINK] round start: sig=0x%02X pwr=%u\r\n",
               (unsigned int)sig, (unsigned int)power);
}

/* 发送本轮快照(每次发送都换新 SEQ),然后进入等待窗口 */
static void uplink_send_round(void)
{
    s_uplink.seq = s_uplink.seq_next++;
    uplink_send_snapshot(s_uplink.seq, s_uplink.round_sig, s_uplink.round_pwr);

    /* 记下实际发出的 SEQ:主机 ACK 的 CORR 必须与之相等 */
    dbg_printf("[UPLINK] tx STATUS seq=%u sig=0x%02X pwr=%u (等 %u ms)\r\n",
               (unsigned int)s_uplink.seq, (unsigned int)s_uplink.round_sig,
               (unsigned int)s_uplink.round_pwr, (unsigned int)APP_LORA_ACK_TIMEOUT_MS);

    s_uplink.tick = HAL_GetTick();          /* 窗口从发出后开始计 */
    s_uplink.state = UPLINK_WAIT_ACK;
}

/* 结束本轮:若事务期间来过新快照,立即补发一轮 */
static void uplink_finish_round(void)
{
    if (s_uplink.reshoot != 0U)
    {
        s_uplink.reshoot = 0U;
        dbg_printf("[UPLINK] new snapshot pending, re-uplink\r\n");
        uplink_begin_round(s_uplink.latest_sig, s_uplink.latest_pwr);
    }
    else
    {
        s_uplink.state = UPLINK_IDLE;
    }
}

/* 窗口超时:还有额度就换新 SEQ 重发,额度用尽记失败 */
static void uplink_window_timeout(void)
{
    if (s_uplink.retry_left > 0U)
    {
        s_uplink.retry_left--;
        s_uplink.stats.retry_cnt++;
        s_uplink.backoff_ms = (uint8_t)APP_LORA_RETRY_BACKOFF_MS;    /* 重发前退避,错开碰撞 */
        s_uplink.tick = HAL_GetTick();
        s_uplink.state = UPLINK_BACKOFF;
    }
    else
    {
        s_uplink.stats.round_fail++;
        dbg_printf("[UPLINK] retry exhausted, not acked\r\n");
        uplink_finish_round();
    }
}

/* ACK 到达:必须是 0x83 且 CORR==在途 SEQ */
static void uplink_ack_rx(uint8_t fun, uint8_t corr, uint8_t code)
{
    if (s_uplink.state != UPLINK_WAIT_ACK)
    {
        dbg_printf("[UPLINK] ACK(fun=0x%02X corr=%u) 被丢:当前 state=%u,不在等 ACK 窗口\r\n",
                   (unsigned int)fun, (unsigned int)corr, (unsigned int)s_uplink.state);
        return;                     /* 不在等 ACK(迟到/重复包):忽略 */
    }

    if ((fun != LORA_FUN_STATUS) || (corr != s_uplink.seq))
    {
        dbg_printf("[UPLINK] ACK fun=0x%02X corr=%u 被丢:在等 fun=0x%02X corr=%u\r\n",
                   (unsigned int)fun, (unsigned int)corr,
                   (unsigned int)LORA_FUN_STATUS, (unsigned int)s_uplink.seq);
        return;                     /* 不是当前在等的那一帧:忽略 */
    }

    if (code == LORA_ACK_CODE_OK)
    {
        s_uplink.stats.round_ok++;
        dbg_printf("[UPLINK] acked (seq=%u)\r\n", (unsigned int)corr);
    }
    else
    {
        /* 主机明确拒收:重发没有意义,直接记拒绝 */
        s_uplink.stats.rejected++;
        dbg_printf("[UPLINK] rejected, code=%u\r\n", (unsigned int)code);
    }

    uplink_finish_round();          /* 该帧已有结论:收尾 */
}

/* 事务状态机:由 app_lora_process() 每轮调用推进 */
static void uplink_poll(void)
{
    uint32_t now = HAL_GetTick();

    switch (s_uplink.state)
    {
    case UPLINK_IDLE:
        break;

    case UPLINK_BACKOFF:
        if ((now - s_uplink.tick) >= (uint32_t)s_uplink.backoff_ms)
        {
            uplink_send_round();
        }
        break;

    case UPLINK_WAIT_ACK:
        if ((now - s_uplink.tick) >= APP_LORA_ACK_TIMEOUT_MS)
        {
            uplink_window_timeout();    /* 缺 ACK:重发或记失败 */
        }
        break;

    default:
        break;
    }
}
#endif /* APP_LORA_ACK_ENABLE == 1:完整交付状态机 */

/* ---- 对外接口 ---- */

void app_lora_uplink_status(uint8_t sig, uint8_t power)
{
    s_uplink.last_sig = sig;
    s_uplink.last_pwr = power;

#if (APP_LORA_ACK_ENABLE == 1)
    if (s_uplink.state == UPLINK_IDLE)
    {
        uplink_begin_round(sig, power);
    }
    else if (s_uplink.state == UPLINK_BACKOFF)
    {
        /* 帧还没发出去(或正准备重发):直接刷新本轮快照 */
        s_uplink.round_sig = sig;
        s_uplink.round_pwr = power;
    }
    else
    {
        /* 等 ACK 中:存为最新快照,本轮结束后自动补发一轮 */
        s_uplink.latest_sig = sig;
        s_uplink.latest_pwr = power;
        s_uplink.reshoot = 1U;
    }
#else
    /* 简化版:不关心事务状态,来一次就立即把快照送出去(不等确认) */
    uplink_send_round_now(sig, power);
#endif
}

uint8_t app_lora_uplink_busy(void)
{
#if (APP_LORA_ACK_ENABLE == 1)
    return (s_uplink.state != UPLINK_IDLE) ? 1U : 0U;
#else
    return 0U;          /* 简化版:帧已发完,允许立刻进 Stop */
#endif
}

void app_lora_uplink_get_stats(app_lora_uplink_stats_t *out)
{
    if (out != NULL)
    {
        *out = s_uplink.stats;
    }
}

/*********************************
 * 三、主机自定义通信发送以及解析(点名应答 + 收包状态机)
 * 逐字节收包 -> 收满整帧 -> CRC8 校验 -> ACK 匹配 / 功能码 switch 落地
 *********************************/
void app_lora_set_signal(uint8_t *signal)
{
    signal_status = signal;
}

void app_lora_set_power(uint8_t *power)
{
    power_status = power;
}

/* 取小端 32bit */
static uint32_t lora_u32_le(const uint8_t *p)
{
    return ((uint32_t)p[0]) | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* lora 接收消息解析 */
/* ---- 整帧校验通过后:身份过滤 -> ACK 匹配 / 功能码 switch;其余丢弃 ---- */
static void lora_rx_parse(const uint8_t *f, uint8_t len)
{
    uint8_t  fun       = f[3];
    uint32_t uid       = lora_u32_le(&f[4]);
    uint32_t my_uid    = app_lora_uid32();
    uint8_t  directed  = ((f[1] == (uint8_t)APP_DEVICE_ADDR) && (uid == my_uid)) ? 1U : 0U;
    uint8_t  broadcast = ((f[1] == LORA_ADDR_BROADCAST) && (uid == LORA_UID32_BROADCAST)) ? 1U : 0U;

    /* 收到一帧 CRC 正确的帧:先记一笔(这一行出现即证明“字节到了 + 帧完整 + CRC 对”) */
    dbg_printf("[LORA RX] frame fun=0x%02X addr=0x%02X uid=0x%08lX len=%u\r\n",
               (unsigned int)fun, (unsigned int)f[1], (unsigned long)uid, (unsigned int)len);

    /* 身份过滤:定向帧必须"地址 + UID32"都指向本机;广播帧地址=0xFF 且 UID32=0 */
    if ((directed == 0U) && (broadcast == 0U))
    {
        dbg_printf("[LORA RX] drop: 不是发给本机(本机 addr=0x%02X uid=0x%08lX)\r\n",
                   (unsigned int)APP_DEVICE_ADDR, (unsigned long)my_uid);
        return;
    }

    /* ACK:FUN bit7=1,[8]=CORR(被确认帧的SEQ) [9]=CODE。
     * ACK 必须精确指向本机(地址+UID32 都对):广播 ACK 一律忽略。 */
    if ((fun & LORA_FRAME_ACK_BIT) != 0U)
    {
#if (APP_LORA_ACK_ENABLE == 1)
        if ((directed != 0U) && (len >= LORA_FRAME_ACK_LEN))
        {
            uplink_ack_rx((uint8_t)(fun & (uint8_t)~LORA_FRAME_ACK_BIT), f[8], f[9]);
        }
        else
        {
            dbg_printf("[LORA RX] drop ACK: directed=%u len=%u(要求 directed=1, len>=%u)\r\n",
                       (unsigned int)directed, (unsigned int)len,
                       (unsigned int)LORA_FRAME_ACK_LEN);
        }
#else
        /* 简化版:主机仍会回 ACK,这里一律忽略(帧格式不变,只是不等确认) */
        dbg_printf("[LORA RX] ack ignored (APP_LORA_ACK_ENABLE=0)\r\n");
#endif
        return;
    }

    switch (fun)
    {
    case LORA_FUN_GET_POWER:                        /* 点名:取电量 */
    case LORA_FUN_GET_SIGNAL:                       /* 点名:取信号 */
        /* 应答:立即回一帧最新状态快照(带新 SEQ,不等 ACK;主机按请求自行关联) */
        uplink_send_snapshot(s_uplink.seq_next++,
                             (signal_status != NULL) ? *signal_status : s_uplink.last_sig,
                             (power_status != NULL) ? *power_status : s_uplink.last_pwr);
        break;
    default:                                        /* 未定义功能码:丢弃 */
        break;
    }
}

/* ---- 接收:找帧头 -> 按 LEN 收满 -> CRC 校验 -> 解析 ---- */
static uint8_t  s_rx_buf[LORA_FRAME_BUF_MAX];
static uint8_t  s_rx_idx;       /* 0 = 正在找帧头,其余为已收字节数 */
static uint32_t s_rx_tick;      /* 最近一个字节的时刻,用于字节间超时 */

/* 把一段原始字节按十六进制打出来:定位“字节到底有没有到 MCU”这一层 */
#if (APP_DEBUG_ENABLE == 1)
static void lora_rx_dump(const uint8_t *p, uint8_t n)
{
    uint8_t i;

    dbg_printf("[LORA RX] raw %uB:", (unsigned int)n);
    for (i = 0U; i < n; i++)
    {
        dbg_printf(" %02X", (unsigned int)p[i]);
    }
    dbg_printf("\r\n");
}
#endif

/* 喂一个字节:无阻塞、无忙等,有多少吃多少 */
static void lora_rx_byte(uint8_t b)
{
    s_rx_tick = HAL_GetTick();

    if (s_rx_idx == 0U)                     /* 找帧头:非帧头字节直接丢 */
    {
        if (b == LORA_FRAME_HEAD)
        {
            s_rx_buf[0] = b;
            s_rx_idx    = 1U;
        }
        return;
    }

    s_rx_buf[s_rx_idx] = b;
    s_rx_idx++;

    if (s_rx_idx == 3U)                     /* 刚收完 LEN([2]):先做范围检查 */
    {
        uint8_t len = s_rx_buf[2];

        if ((len < LORA_FRAME_MIN_LEN) || (len > (uint8_t)sizeof(s_rx_buf)))
        {
            s_rx_idx = 0U;                  /* 长度非法:丢帧重找帧头 */
        }
        return;
    }

    if (s_rx_idx == s_rx_buf[2])            /* 整帧收满:CRC 对才分发 */
    {
        uint8_t len = s_rx_buf[2];

        if (lora_crc8_cal(s_rx_buf, (uint16_t)(len - 1U)) == s_rx_buf[len - 1U])
        {
            lora_rx_parse(s_rx_buf, len);
        }
        else
        {
            dbg_printf("[LORA RX] crc err len=%u\r\n", (unsigned int)len);
        }
        s_rx_idx = 0U;
    }
}

void app_lora_process(void)
{
    uint8_t b;
#if (APP_DEBUG_ENABLE == 1)
    uint8_t burst[24];
    uint8_t n = 0U;
    uint8_t i;
    lora_rx_diag_t diag;
#endif

    /* 0) 接收自愈:挂载失败/被接收错误打断时补挂,避免“永久收不到但不报错” */
#if (APP_DEBUG_ENABLE == 1)
    if (lora_uart_rx_ensure_armed(&diag) != 0U)
    {
        dbg_printf("[LORA RX] re-armed (bytes=%lu arm_fail=%u rearm=%u)\r\n",
                   (unsigned long)diag.rx_bytes, (unsigned int)diag.arm_fail,
                   (unsigned int)diag.rearm);
    }
#else
    (void)lora_uart_rx_ensure_armed(NULL);
#endif

    /* 1) 收:把接收 FIFO 里的字节全部喂给状态机(非阻塞,有多少吃多少)。
     *    注:不再依赖“模块唤醒标志”——AUX(PA0) 没有 EXTI(要让出 EXTI0 给 PB0),
     *    该标志不会置位;直接抽干 FIFO,ACK/下行命令才能被处理。
     *    调试期先把这一批字节按十六进制打出来:确认“字节到底有没有到 MCU”。 */
#if (APP_DEBUG_ENABLE == 1)
    while ((n < (uint8_t)sizeof(burst)) && (app_lora_read_byte(&burst[n]) != 0U))
    {
        n++;
    }
    if (n != 0U)
    {
        lora_rx_dump(burst, n);
        for (i = 0U; i < n; i++)
        {
            lora_rx_byte(burst[i]);
        }
    }
#endif

    while (app_lora_read_byte(&b) != 0U)
    {
        lora_rx_byte(b);
    }

    /* 2) 半帧超时保护:字节间超过 LORA_FRAME_GAP_MS 就丢弃重找帧头 */
    if ((s_rx_idx != 0U) && ((HAL_GetTick() - s_rx_tick) > LORA_FRAME_GAP_MS))
    {
        s_rx_idx = 0U;
    }

    /* 3) 推进上行交付事务(退避 -> 发送 -> 等ACK -> 重发/收尾) */
#if (APP_LORA_ACK_ENABLE == 1)
    uplink_poll();
#endif
}
