/**
 * @file app_lora_net.c
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief LoRa 组网层实现:唯一识别码 / 定点传输 / 帧协议 v2 / ACK+重传 / 接收窗口
 * @version 0.2
 * @date 2026-09-30
 *
 * @note 分层关系:
 *         app_lora.c(h)       —— 模块能力层:模式(M0/M1)、收发字节、配置读写
 *         app_lora_procotol.c —— 旧的 v1 协议(单机/单播, 保留兼容)
 *         app_lora_net.c(本文件) —— 组网层:多对多寻址 + v2 帧 + 交付确认
 *
 * @note 关键设计:
 *       1) 唯一识别码的唯一真源 = 模块地址寄存器。开机读回 -> 内存节点号,
 *          所以现场改地址后重启即生效,不需要重烧固件。
 *       2) 定点传输(REG2 bit7)必须开:发送前加 3 字节 [目标ADDH][目标ADDL][目标信道],
 *          接收端由模块硬件按地址过滤,别人的包根本不会吐给 MCU。
 *       3) "主机要确认收到" = 上行帧等 ACK(CORR 匹配),超时随机退避重传。
 *          设备平时休眠, 只有在上行后的接收窗口(APP_LORA_NET_ACK_TIMEOUT_MS)里能收下行。
 *          要做到"随时可被主机叫醒",必须把 AUX 接回一根空闲 EXTI(硬件改动)或加周期唤醒。
 */
#include <string.h>

#include "app_config.h"
#include "app_lora_net.h"
#include "app_lora_procotol.h"   /* app_lora_cfg_read / write / reg_verify */

#include "bsp_dbg_uart.h"
#include "main.h"                /* HAL_GetTick / HAL_Delay */

/*==============================================================================
 * 一、内部常量
 *============================================================================*/

#define NET_HEAD            0x5AU
#define NET_VER             0x02U

#define NET_HDR_LEN         8U      /* 头+LEN+VER+FUN+SRC(2)+GRP+SEQ */
#define NET_DATA_MAX        16U     /* DATA 上限(CORR 1B + 负载 15B) */
#define NET_BUF_MAX         (NET_HDR_LEN + NET_DATA_MAX + 1U)   /* 25 */
#define NET_ADDR_HDR_LEN    3U      /* 定点传输包头 ADDH ADDL CHAN */
#define NET_TX_BUF_MAX      (NET_ADDR_HDR_LEN + NET_BUF_MAX)

/* 帧下标 */
#define NET_IDX_HEAD        0U
#define NET_IDX_LEN         1U
#define NET_IDX_VER         2U
#define NET_IDX_FUN         3U
#define NET_IDX_SRC_L       4U
#define NET_IDX_SRC_H       5U
#define NET_IDX_GRP         6U
#define NET_IDX_SEQ         7U
#define NET_IDX_DATA        8U

#define NET_MIN_LEN         (NET_HDR_LEN + 1U + 1U)     /* 只有 CORR、无负载 */

#define NET_FUN_ACK         0x80U
#define NET_FUN_MASK        0x7FU

/* 退避上限为 0 会导致后面取模除零, 编译期就拦住 */
#if (APP_LORA_NET_BACKOFF_MS == 0U)
#error "APP_LORA_NET_BACKOFF_MS 不能为 0"
#endif

/* 地址空间合法性: 地址 = [组号][节点序号] */
#if ((APP_LORA_NET_GROUP_ID > 0xFEU) || ((APP_LORA_NET_ADDR_FROM_GROUP == 1U) && (APP_LORA_NET_GROUP_ID == 0U)))
#error "APP_LORA_NET_GROUP_ID 必须落在 0x01~0xFE(0x00/0xFF 保留)"
#endif
/* 注意: 这里用 APP_DEVICE_ADDR 而不是 APP_LORA_NET_NODE_INDEX,
 *       因为后者带 (uint8_t) 强制转换, 在 #if 里会被预处理器判为语法错误 */
#if (APP_LORA_NET_ADDR_FROM_GROUP == 1U)
#if ((APP_DEVICE_ADDR < 1) || (APP_DEVICE_ADDR > 0xFE))
#error "APP_DEVICE_ADDR(节点序号) 必须是 0x01~0xFE(0x00=主机, 0xFF 保留)"
#endif
#endif
#if (APP_LORA_NET_CHANNEL > APP_LORA_NET_CHANNEL_MAX)
#error "APP_LORA_NET_CHANNEL 超出本型号信道范围(会被模块截断)"
#endif

/*==============================================================================
 * 二、内部状态
 *============================================================================*/

typedef enum
{
    NET_ST_IDLE = 0,    /* 空闲(模块休眠)      */
    NET_ST_BACKOFF,     /* 随机退避中, 到点发送 */
    NET_ST_WAIT_ACK,    /* 已发出, 等主机 ACK   */
    NET_ST_LISTEN       /* 接收窗口            */
} net_state_t;

static uint16_t s_node_id  = 0U;                        /* 本机节点号 = [组号][节点序号] */
static uint8_t  s_group    = 0U;                        /* 组号                   */
static uint8_t  s_channel  = 0U;                        /* 信道(定点包头用)       */
static uint16_t s_host_addr = 0U;                       /* 本组主机地址(上行目标) */
static uint8_t  s_seq      = 0U;                        /* 本机发送序号           */

static uint8_t  s_awake    = 0U;                        /* 1=模块处于正常模式     */
static net_state_t s_state = NET_ST_IDLE;
static uint32_t s_deadline = 0U;

static uint8_t  s_pending[NET_BUF_MAX];                 /* 待发帧                 */
static uint8_t  s_pending_len     = 0U;
static uint8_t  s_pending_fun     = 0U;
static uint8_t  s_pending_need_ack = 0U;

static uint8_t  s_inflight_seq = 0U;                    /* 等 ACK 的帧序号        */
static uint8_t  s_inflight_fun = 0U;
static uint8_t  s_retry        = 0U;
static uint8_t  s_ack_code     = APP_LORA_NET_CODE_OK;

static uint8_t  s_last_signal = 0U;                     /* 最近一次上报值(缓存)   */
static uint8_t  s_last_power  = 0U;
static uint8_t *s_signal_ptr  = NULL;
static uint8_t *s_power_ptr   = NULL;

static app_lora_net_result_cb_t s_result_cb = NULL;
static app_lora_net_stats_t     s_stats;

static uint32_t s_rand = 0x12345678U;

/* 接收状态机 */
static uint8_t  s_rx[NET_BUF_MAX];
static uint8_t  s_rx_idx  = 0U;
static uint32_t s_rx_tick = 0U;

/*==============================================================================
 * 三、通用工具
 *============================================================================*/

/* CRC 8-CCITT(多项式 0x07),与旧协议一致 */
static uint8_t net_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00U;
    uint16_t i;
    uint8_t  j;

    for (i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for (j = 0U; j < 8U; j++)
        {
            crc = (uint8_t)((crc & 0x80U) ? ((crc << 1) ^ 0x07U) : (crc << 1));
        }
    }
    return crc;
}

/* xorshift32:只用它打散退避相位, 不用于加密 */
static uint32_t net_rand(void)
{
    s_rand ^= (s_rand << 13);
    s_rand ^= (s_rand >> 17);
    s_rand ^= (s_rand << 5);
    return s_rand;
}

/* 到期判断, 处理 tick 回绕 */
static uint8_t net_expired(uint32_t deadline)
{
    return ((int32_t)(HAL_GetTick() - deadline) >= 0) ? 1U : 0U;
}

/* 由组号推导 上行目标地址(主机地址) / 可选信道
 * 地址约定: [组号 8bit][节点序号 8bit], 主机 = 组号 << 8 */
static void net_host_addr_sync(void)
{
    s_host_addr = (uint16_t)((uint16_t)s_group << 8);

#if (APP_LORA_NET_CHANNEL_FROM_GROUP == 1U)
    if (s_group != 0U)
    {
        s_channel = (uint8_t)(s_group % ((uint8_t)APP_LORA_NET_CHANNEL_MAX + 1U));
    }
#endif
}

uint32_t app_lora_net_uid32(void)
{
#if defined(UID_BASE)
    const uint32_t *uid = (const uint32_t *)UID_BASE;   /* STM32L0 96bit UID */
    return (uid[0] ^ uid[1] ^ uid[2]);
#else
    return 0x5A5AA5A5U;
#endif
}

uint16_t app_lora_net_node_id(void)
{
    return s_node_id;
}

uint16_t app_lora_net_host_addr(void)
{
    return s_host_addr;
}

uint8_t app_lora_net_group(void)
{
    return s_group;
}

uint8_t app_lora_net_channel(void)
{
    return s_channel;
}

/*==============================================================================
 * 四、E32 寄存器编解码(位序以模块手册为准)
 *
 *   REG0(0x02) = [7:5]空速 | [4:3]校验 | [2:0]波特率
 *   REG1(0x03) = 信道
 *   REG2(0x04) = [7]定点传输 | [6]IO驱动 | [5:3]唤醒时间 | [2]FEC | [1:0]发射功率
 *============================================================================*/

static void net_e32_encode(const app_lora_e32_cfg_t *c, uint8_t raw[5])
{
    raw[0] = c->addr_h;
    raw[1] = c->addr_l;
    raw[2] = (uint8_t)(((c->air_rate & 0x07U) << 5) |
                       ((c->parity   & 0x03U) << 3) |
                        (c->baud     & 0x07U));
    raw[3] = c->channel;
    raw[4] = (uint8_t)(((c->fixed_point        & 0x01U) << 7) |
                       ((c->io_drv_open_drain  & 0x01U) << 6) |
                       ((c->wakeup_time        & 0x07U) << 3) |
                       ((c->fec_enable         & 0x01U) << 2) |
                        (c->tx_power          & 0x03U));
}

static void net_e32_decode(const uint8_t raw[5], app_lora_e32_cfg_t *c)
{
    c->addr_h = raw[0];
    c->addr_l = raw[1];
    c->air_rate = (uint8_t)((raw[2] >> 5) & 0x07U);
    c->parity   = (uint8_t)((raw[2] >> 3) & 0x03U);
    c->baud     = (uint8_t)( raw[2]       & 0x07U);
    c->channel  = raw[3];
    c->fixed_point       = (uint8_t)((raw[4] >> 7) & 0x01U);
    c->io_drv_open_drain = (uint8_t)((raw[4] >> 6) & 0x01U);
    c->wakeup_time       = (uint8_t)((raw[4] >> 3) & 0x07U);
    c->fec_enable        = (uint8_t)((raw[4] >> 2) & 0x01U);
    c->tx_power          = (uint8_t)( raw[4]       & 0x03U);
}

void app_lora_net_cfg_defaults(app_lora_e32_cfg_t *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    cfg->addr_h            = (uint8_t)((APP_LORA_NET_NODE_ID >> 8) & 0xFFU);
    cfg->addr_l            = (uint8_t)( APP_LORA_NET_NODE_ID       & 0xFFU);
    cfg->air_rate          = (uint8_t)APP_LORA_NET_AIR_RATE;
    cfg->parity            = (uint8_t)APP_LORA_NET_PARITY;
    cfg->baud              = (uint8_t)APP_LORA_NET_BAUD;
    cfg->channel           = (uint8_t)APP_LORA_NET_CHANNEL;
    cfg->wakeup_time       = (uint8_t)APP_LORA_NET_WAKEUP_TIME;
    cfg->io_drv_open_drain = 0U;
    cfg->fec_enable        = (uint8_t)APP_LORA_NET_FEC_ENABLE;
    cfg->tx_power          = (uint8_t)APP_LORA_NET_TX_POWER;
    cfg->fixed_point       = 1U;    /* 定点传输: 组网必须开 */
}

#if (APP_DEBUG_ENABLE == 1)
static const char *const net_air_rate_name[8] =
{ "0.3k", "1.2k", "2.4k", "4.8k", "9.6k", "19.2k", "38.4k", "62.5k" };
static const char *const net_baud_name[8] =
{ "1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200" };
static const char *const net_parity_name[4] =
{ "8N1", "8O1", "8E1", "8N1" };

void app_lora_net_dump_cfg(const app_lora_e32_cfg_t *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    dbg_printf("[NET] E32 ADDR=0x%02X%02X CHAN=0x%02X(%uMHz)\r\n",
               (unsigned int)cfg->addr_h, (unsigned int)cfg->addr_l,
               (unsigned int)cfg->channel,
               (unsigned int)(APP_LORA_NET_CHANNEL_BASE_MHZ + cfg->channel));
    dbg_printf("[NET]     air=%s parity=%s baud=%s\r\n",
               net_air_rate_name[cfg->air_rate & 0x07U],
               net_parity_name[cfg->parity & 0x03U],
               net_baud_name[cfg->baud & 0x07U]);
    dbg_printf("[NET]     fixed_point=%u io=%s wor_idx=%u fec=%u pwr_idx=%u\r\n",
               (unsigned int)cfg->fixed_point,
               cfg->io_drv_open_drain ? "open-drain" : "push-pull",
               (unsigned int)cfg->wakeup_time,
               (unsigned int)cfg->fec_enable,
               (unsigned int)cfg->tx_power);
}
#endif /* APP_DEBUG_ENABLE */

uint8_t app_lora_net_cfg_read_raw(uint8_t raw[5])
{
    lora_reg_parm_cfg_t tmp;

    if (raw == NULL)
    {
        return 0U;
    }

    if (app_lora_cfg_read(&tmp, 5U) == 0U)
    {
        return 0U;
    }

    memcpy(raw, tmp.data, 5U);
    s_awake = 1U;       /* app_lora_cfg_read 内部退出配置模式后停在正常模式 */
    return 1U;
}

uint8_t app_lora_net_cfg_apply(const app_lora_e32_cfg_t *cfg)
{
    uint8_t raw[5];
    lora_reg_parm_cfg_t tmp;

    if (cfg == NULL)
    {
        return 0U;
    }

    net_e32_encode(cfg, raw);
    memcpy(tmp.data, raw, 5U);

    if (app_lora_cfg_reg_verify(tmp, 5U) == 0U)
    {
        s_awake = 1U;   /* 校验流程无论如何都退出到正常模式了 */
        return 0U;
    }

    s_awake = 1U;
    return 1U;
}

uint8_t app_lora_net_cfg_selftest(void)
{
    app_lora_e32_cfg_t cur;
    app_lora_e32_cfg_t want;
    uint8_t raw[5];
    uint8_t raw_want[5];
    uint8_t ok;

    app_lora_net_cfg_defaults(&want);
    net_e32_encode(&want, raw_want);

    if (app_lora_net_cfg_read_raw(raw) == 0U)
    {
        dbg_printf("[NET] cfg: read FAIL, apply defaults\r\n");
        return app_lora_net_cfg_apply(&want);
    }

    net_e32_decode(raw, &cur);

#if (APP_DEBUG_ENABLE == 1)
    app_lora_net_dump_cfg(&cur);
#endif

    if (cur.channel > (uint8_t)APP_LORA_NET_CHANNEL_MAX)
    {
        dbg_printf("[NET] cfg: CHAN 0x%02X 超出该型号范围(0x00~0x%02X), 会被模块截断!\r\n",
                   (unsigned int)cur.channel, (unsigned int)APP_LORA_NET_CHANNEL_MAX);
    }

    ok = (memcmp(raw, raw_want, 5U) == 0U) ? 1U : 0U;
    if (ok == 0U)
    {
        dbg_printf("[NET] cfg: mismatch -> rewrite\r\n");
        ok = app_lora_net_cfg_apply(&want);
        dbg_printf("[NET] cfg: rewrite %s\r\n", ok ? "OK" : "FAIL");
    }
    else
    {
        dbg_printf("[NET] cfg: match expected\r\n");
    }

    return ok;
}

uint8_t app_lora_net_set_node_id(uint16_t node_id)
{
    app_lora_e32_cfg_t cfg;
    uint8_t raw[5];

    if ((node_id == 0U) || (node_id == APP_LORA_NET_BROADCAST_ADDR))
    {
        return 0U;      /* 0 和 0xFFFF 是保留值 */
    }

    if (app_lora_net_cfg_read_raw(raw) == 0U)
    {
        return 0U;
    }

    net_e32_decode(raw, &cfg);
    cfg.addr_h = (uint8_t)(node_id >> 8);
    cfg.addr_l = (uint8_t)(node_id & 0xFFU);

    if (app_lora_net_cfg_apply(&cfg) == 0U)
    {
        return 0U;
    }

    s_node_id = node_id;
    s_group   = (uint8_t)(node_id >> 8);        /* 组号 = 地址高字节 */
    net_host_addr_sync();

    dbg_printf("[NET] node id -> 0x%04X (grp=%u, saved in module)\r\n",
               (unsigned int)node_id, (unsigned int)s_group);
    return 1U;
}

/* 分配 组号 + 节点序号 = 完整重配地址(批量生产/上位机远程下发用) */
uint8_t app_lora_net_set_group(uint8_t group, uint8_t node_index)
{
    app_lora_e32_cfg_t cfg;
    uint8_t  raw[5];
    uint16_t addr;

    if ((group == 0U) || (group > 0xFEU) || (node_index == 0U) || (node_index == 0xFFU))
    {
        return 0U;      /* 组号 0x00/0xFF 保留; 节点序号 0x00=主机, 0xFF 保留 */
    }

    if (app_lora_net_cfg_read_raw(raw) == 0U)
    {
        return 0U;
    }
    net_e32_decode(raw, &cfg);

    addr       = (uint16_t)(((uint16_t)group << 8) | (uint16_t)node_index);
    cfg.addr_h = (uint8_t)(addr >> 8);
    cfg.addr_l = (uint8_t)(addr & 0xFFU);

    s_group = group;                    /* 先更新组号, 才能推导信道 */
    net_host_addr_sync();

#if (APP_LORA_NET_CHANNEL_FROM_GROUP == 1U)
    cfg.channel = s_channel;            /* 按组号散频时同步把信道写进模块 */
#endif

    if (app_lora_net_cfg_apply(&cfg) == 0U)
    {
        return 0U;
    }

    s_node_id = addr;

    dbg_printf("[NET] addr -> 0x%04X (grp=%u node=%u ch=0x%02X, saved in module)\r\n",
               (unsigned int)addr, (unsigned int)s_group,
               (unsigned int)node_index, (unsigned int)s_channel);
    return 1U;
}

/*==============================================================================
 * 五、模块模式与发送
 *============================================================================*/

static void net_wake(void)
{
    if (s_awake != 0U)
    {
        return;
    }
    app_lora_set_mode(LORA_MODE_NORMAL);
    HAL_Delay(APP_LORA_NET_MODE_SETTLE_MS);
    s_awake = 1U;
}

static void net_sleep(void)
{
    if (s_awake == 0U)
    {
        return;
    }
    app_lora_set_mode(LORA_MODE_SLEEP);     /* M0=M1=1, 模块 µA 级 */
    HAL_Delay(APP_LORA_NET_MODE_SETTLE_MS);
    s_awake = 0U;
}

#if (APP_DEBUG_ENABLE == 1)
static void net_dump_frame(const char *tag, const uint8_t *buf, uint8_t len)
{
    uint8_t i;

    dbg_printf("[NET] %s(%u):", tag, (unsigned int)len);
    for (i = 0U; i < len; i++)
    {
        dbg_printf(" %02X", (unsigned int)buf[i]);
    }
    dbg_printf("\r\n");
}
#endif

/* 定点传输发送: [目标ADDH][目标ADDL][目标CHAN] + 帧 */
static uint8_t net_send_frame(uint16_t dst, const uint8_t *frame, uint8_t len)
{
    uint8_t tx[NET_TX_BUF_MAX];

    if ((frame == NULL) || (len == 0U) || (len > NET_BUF_MAX))
    {
        return 0U;
    }

    net_wake();

    tx[0] = (uint8_t)((dst >> 8) & 0xFFU);
    tx[1] = (uint8_t)( dst       & 0xFFU);
    tx[2] = s_channel;
    memcpy(&tx[NET_ADDR_HDR_LEN], frame, (size_t)len);

    app_lora_send_bytes(tx, (uint16_t)(NET_ADDR_HDR_LEN + len));
    s_stats.tx_frames++;

#if (APP_DEBUG_ENABLE == 1)
    net_dump_frame("TX>air", tx, (uint8_t)(NET_ADDR_HDR_LEN + len));
#endif

    return 1U;
}

/* 组帧: DATA = [CORR][payload...] */
static uint8_t net_build_frame(uint8_t *buf, uint8_t fun, uint8_t corr,
                               const uint8_t *payload, uint8_t plen,
                               uint16_t src, uint8_t seq)
{
    uint8_t len;
    uint8_t i;

    if ((buf == NULL) || (plen > (NET_DATA_MAX - 1U)))
    {
        return 0U;
    }

    buf[NET_IDX_HEAD]  = NET_HEAD;
    buf[NET_IDX_VER]   = NET_VER;
    buf[NET_IDX_FUN]   = fun;
    buf[NET_IDX_SRC_L] = (uint8_t)( src       & 0xFFU);
    buf[NET_IDX_SRC_H] = (uint8_t)((src >> 8) & 0xFFU);
    buf[NET_IDX_GRP]   = s_group;
    buf[NET_IDX_SEQ]   = seq;
    buf[NET_IDX_DATA]  = corr;

    for (i = 0U; i < plen; i++)
    {
        buf[NET_IDX_DATA + 1U + i] = payload[i];
    }

    len = (uint8_t)(NET_HDR_LEN + 1U + plen + 1U);
    buf[NET_IDX_LEN] = len;
    buf[len - 1U]    = net_crc8(buf, (uint16_t)(len - 1U));

    return len;
}

/* 开接收窗口:模块保持正常模式 ms 毫秒 */
static void net_listen(uint32_t ms)
{
    if (ms == 0U)
    {
        s_state = NET_ST_IDLE;
        net_sleep();
        return;
    }

    net_wake();
    s_state    = NET_ST_LISTEN;
    s_deadline = HAL_GetTick() + ms;
}

/* 把一帧放进待发槽并启动随机退避 */
static void net_uplink_request(uint8_t fun, uint8_t corr,
                               const uint8_t *payload, uint8_t plen,
                               uint8_t need_ack)
{
    uint8_t len;

    s_seq++;
    len = net_build_frame(s_pending, fun, corr, payload, plen, s_node_id, s_seq);
    if (len == 0U)
    {
        return;
    }

    if (s_state == NET_ST_WAIT_ACK)
    {
        s_stats.tx_retry++;     /* 上一帧还没等到 ACK 就被新数据顶掉 */
    }

    s_pending_len      = len;
    s_pending_fun      = fun;
    s_pending_need_ack = need_ack;
    s_state            = NET_ST_BACKOFF;
    s_deadline         = HAL_GetTick() + (net_rand() % (uint32_t)APP_LORA_NET_BACKOFF_MS);
}

/* 立即回一条 ACK(不占用待发槽) */
static uint8_t net_send_ack(uint16_t dst, uint8_t cmd_fun, uint8_t corr, uint8_t code)
{
    uint8_t frame[NET_BUF_MAX];
    uint8_t payload[1];
    uint8_t len;

    payload[0] = code;
    s_seq++;
    len = net_build_frame(frame, (uint8_t)(cmd_fun | NET_FUN_ACK), corr,
                          payload, 1U, s_node_id, s_seq);
    if (len == 0U)
    {
        return 0U;
    }

    return net_send_frame(dst, frame, len);
}

/* 一帧的交付结束(成功/失败) */
static void net_finish(uint8_t ok)
{
    if (ok != 0U)
    {
        s_stats.ack_ok++;
    }
    else if (s_pending_need_ack != 0U)
    {
        s_stats.tx_fail++;
    }

    if (s_result_cb != NULL)
    {
        s_result_cb(s_pending_fun, ok);
    }

#if (APP_DEBUG_ENABLE == 1)
    dbg_printf("[NET] fun=0x%02X %s retry=%u code=%u\r\n",
               (unsigned int)s_pending_fun, ok ? "delivered" : "FAILED",
               (unsigned int)s_retry, (unsigned int)s_ack_code);
#endif

    s_retry       = 0U;
    s_pending_len = 0U;

    if (ok != 0U)
    {
        net_listen(APP_LORA_NET_TAIL_MS);
    }
    else
    {
        s_state = NET_ST_IDLE;
        net_sleep();
    }
}

/* 退避到点:真正发出去 */
static void net_tx_go(void)
{
    if (net_send_frame(s_host_addr, s_pending, s_pending_len) == 0U)
    {
        s_state = NET_ST_IDLE;
        net_sleep();
        return;
    }

    s_inflight_seq = s_pending[NET_IDX_SEQ];
    s_inflight_fun = s_pending[NET_IDX_FUN];

    if (s_pending_need_ack != 0U)
    {
        s_state    = NET_ST_WAIT_ACK;
        s_deadline = HAL_GetTick() + (uint32_t)APP_LORA_NET_ACK_TIMEOUT_MS;
    }
    else
    {
        net_listen(APP_LORA_NET_TAIL_MS);
    }
}

/* 等 ACK 超时 -> 换新序号重传, 重传耗尽则判失败 */
static void net_ack_timeout(void)
{
    s_stats.ack_timeout++;
    s_retry++;

    if (s_retry <= (uint8_t)APP_LORA_NET_RETRY_MAX)
    {
        s_stats.tx_retry++;

        s_seq++;
        s_pending[NET_IDX_SEQ] = s_seq;
        s_pending[s_pending_len - 1U] = net_crc8(s_pending, (uint16_t)(s_pending_len - 1U));

        s_state    = NET_ST_BACKOFF;
        s_deadline = HAL_GetTick() + 50U +
                     (net_rand() % (uint32_t)APP_LORA_NET_BACKOFF_MS);
    }
    else
    {
        net_finish(0U);
    }
}

/*==============================================================================
 * 六、接收解析
 *============================================================================*/

static void net_frame_handle(const uint8_t *f)
{
    uint8_t  len  = f[NET_IDX_LEN];
    uint8_t  ver  = f[NET_IDX_VER];
    uint8_t  fun  = f[NET_IDX_FUN];
    uint8_t  grp  = f[NET_IDX_GRP];
    uint8_t  seq  = f[NET_IDX_SEQ];
    uint8_t  corr = f[NET_IDX_DATA];
    uint8_t  dlen = (uint8_t)(len - NET_HDR_LEN - 1U);
    const uint8_t *pl = &f[NET_IDX_DATA + 1U];
    uint16_t src  = (uint16_t)(f[NET_IDX_SRC_L] | ((uint16_t)f[NET_IDX_SRC_H] << 8));

    /* --- 过滤 --- */
    if (ver != NET_VER)
    {
        s_stats.rx_filtered++;
        return;
    }
    if (src == s_node_id)                       /* 自发自收/中继回环 */
    {
        s_stats.rx_filtered++;
        return;
    }
    if ((grp != 0U) && (grp != s_group))        /* 帧内组号与本机不符 */
    {
        s_stats.rx_filtered++;
        return;
    }
#if (APP_LORA_NET_ADDR_FROM_GROUP == 1U)
    if ((uint8_t)(src >> 8) != s_group)         /* 源地址的组号不是本组 */
    {
        s_stats.rx_filtered++;
        return;
    }
#endif

#if (APP_DEBUG_ENABLE == 1)
    dbg_printf("[NET] RX< src=0x%04X fun=0x%02X seq=%u corr=%u grp=%u\r\n",
               (unsigned int)src, (unsigned int)fun, (unsigned int)seq,
               (unsigned int)corr, (unsigned int)grp);
#endif

    /* --- ACK:匹配本机在等的那一帧 --- */
    if ((fun & NET_FUN_ACK) != 0U)
    {
        uint8_t code = (dlen >= 2U) ? pl[0] : APP_LORA_NET_CODE_OK;

        if ((s_state == NET_ST_WAIT_ACK) &&
            (corr == s_inflight_seq) &&
            ((uint8_t)(fun & NET_FUN_MASK) == s_inflight_fun))
        {
            s_ack_code = code;
            net_finish((code == APP_LORA_NET_CODE_OK) ? 1U : 0U);
        }
        return;
    }

    /* --- 下行命令 --- */
    s_stats.cmd_recv++;

    switch (fun)
    {
    case APP_LORA_NET_FUN_GET_POWER:
    {
        uint8_t power = (s_power_ptr != NULL) ? *s_power_ptr : s_last_power;
        net_uplink_request(APP_LORA_NET_FUN_POWER, seq, &power, 1U, 0U);
        break;
    }

    case APP_LORA_NET_FUN_GET_SIGNAL:
    {
        uint8_t sig = (s_signal_ptr != NULL) ? *s_signal_ptr : s_last_signal;
        net_uplink_request(APP_LORA_NET_FUN_SIGNAL, seq, &sig, 1U, 0U);
        break;
    }

    case APP_LORA_NET_FUN_QUERY_CFG:
    {
        uint8_t raw[5];

        if (app_lora_net_cfg_read_raw(raw) != 0U)
        {
            net_uplink_request(APP_LORA_NET_FUN_CFG, seq, raw, 5U, 0U);
        }
        else
        {
            (void)net_send_ack(src, fun, seq, APP_LORA_NET_CODE_BUSY);
        }
        break;
    }

    case APP_LORA_NET_FUN_SET_NODE:
    {
#if (APP_LORA_NET_ALLOW_REMOTE_SET_NODE == 1U)
        uint8_t plen = (uint8_t)(dlen - 1U);

        if (plen != 2U)
        {
            (void)net_send_ack(src, fun, seq, APP_LORA_NET_CODE_BAD_PARAM);
        }
        else
        {
            uint16_t new_id = (uint16_t)(pl[0] | ((uint16_t)pl[1] << 8));
            uint8_t code = app_lora_net_set_node_id(new_id)
                           ? APP_LORA_NET_CODE_OK : APP_LORA_NET_CODE_BAD_PARAM;
            (void)net_send_ack(src, fun, seq, code);
        }
#else
        (void)net_send_ack(src, fun, seq, APP_LORA_NET_CODE_DENIED);
#endif
        break;
    }

    default:
        (void)net_send_ack(src, fun, seq, APP_LORA_NET_CODE_UNSUPPORTED);
        break;
    }
}

/* 字节喂入:找帧头 -> 收满 LEN -> CRC -> 解析 */
static void net_rx_byte(uint8_t b)
{
    uint8_t len;

    s_rx_tick = HAL_GetTick();

    if (s_rx_idx == 0U)
    {
        if (b == NET_HEAD)
        {
            s_rx[0] = b;
            s_rx_idx = 1U;
        }
        return;
    }

    s_rx[s_rx_idx] = b;
    s_rx_idx++;

    if (s_rx_idx == 2U)                     /* 刚拿到 LEN */
    {
        len = s_rx[NET_IDX_LEN];
        if ((len < NET_MIN_LEN) || (len > NET_BUF_MAX))
        {
            s_rx_idx = 0U;
        }
        return;
    }

    if (s_rx_idx >= s_rx[NET_IDX_LEN])      /* 整帧收满 */
    {
        len = s_rx[NET_IDX_LEN];

        if (net_crc8(s_rx, (uint16_t)(len - 1U)) == s_rx[len - 1U])
        {
            s_stats.rx_frames++;
            net_frame_handle(s_rx);
        }
        else
        {
            s_stats.rx_crc_err++;
        }

        s_rx_idx = 0U;
    }
}

/*==============================================================================
 * 七、对外接口
 *============================================================================*/

void app_lora_net_init(void)
{
    app_lora_e32_cfg_t want;
    app_lora_e32_cfg_t cur;
    uint8_t raw[5];
    uint8_t raw_want[5];

    memset(&s_stats, 0, sizeof(s_stats));
    memset(s_pending, 0, sizeof(s_pending));

    s_rand    = app_lora_net_uid32() | 1U;      /* 每台设备的退避相位天然不同 */
    s_group   = (uint8_t)APP_LORA_NET_GROUP_ID;
    s_channel = (uint8_t)APP_LORA_NET_CHANNEL;
    s_node_id = (uint16_t)APP_LORA_NET_NODE_ID;
    s_host_addr = (uint16_t)APP_LORA_NET_HOST_ADDR;
    s_state   = NET_ST_IDLE;
    s_seq     = 0U;
    s_rx_idx  = 0U;
    s_awake   = 1U;                             /* app_lora_init 已把模块设为正常模式 */

    app_lora_net_cfg_defaults(&want);
    net_e32_encode(&want, raw_want);

    /* 唯一识别码的真源 = 模块寄存器 */
    if (app_lora_net_cfg_read_raw(raw) != 0U)
    {
        net_e32_decode(raw, &cur);

        if ((cur.addr_h != 0xFFU) && !((cur.addr_h == 0U) && (cur.addr_l == 0U)))
        {
            s_node_id = (uint16_t)(((uint16_t)cur.addr_h << 8) | cur.addr_l);
            s_group   = cur.addr_h;         /* 组号真源 = 地址高字节(上位机改地址即改组) */
        }
        s_channel = cur.channel;            /* 信道以模块实际配置为准 */
        net_host_addr_sync();               /* 由组号推导上行目标地址/(可选)信道 */

        dbg_printf("[NET] init: node=0x%04X grp=%u ch=0x%02X host=0x%04X\r\n",
                   (unsigned int)s_node_id, (unsigned int)s_group,
                   (unsigned int)s_channel, (unsigned int)s_host_addr);

        if (memcmp(raw, raw_want, 5U) != 0U)
        {
            dbg_printf("[NET] init: module cfg != expected, 调 app_lora_net_cfg_selftest() 修正\r\n");
        }
    }
    else
    {
        dbg_printf("[NET] init: cfg read failed, use compile-time defaults\r\n");
    }

    dbg_printf("[NET] uid=0x%08X\r\n", (unsigned int)app_lora_net_uid32());

    net_sleep();                                /* 空闲即休眠 */

#if (APP_LORA_NET_BOOT_RX_WINDOW_MS > 0U)
    net_listen(APP_LORA_NET_BOOT_RX_WINDOW_MS); /* 上电给上位机一个下发窗口 */
#endif
}

void app_lora_net_set_signal(uint8_t *signal)
{
    s_signal_ptr = signal;
}

void app_lora_net_set_power(uint8_t *power)
{
    s_power_ptr = power;
}

void app_lora_net_set_result_cb(app_lora_net_result_cb_t cb)
{
    s_result_cb = cb;
}

uint8_t app_lora_net_uplink_signal(uint8_t state)
{
    s_last_signal = state;
    net_uplink_request(APP_LORA_NET_FUN_SIGNAL, 0U, &state, 1U,
                       (uint8_t)APP_LORA_NET_ACK_FOR_SIGNAL);
    return 1U;
}

uint8_t app_lora_net_uplink_power(uint8_t percent)
{
    s_last_power = percent;
    net_uplink_request(APP_LORA_NET_FUN_POWER, 0U, &percent, 1U,
                       (uint8_t)APP_LORA_NET_ACK_FOR_POWER);
    return 1U;
}

uint8_t app_lora_net_uplink_heartbeat(void)
{
    net_uplink_request(APP_LORA_NET_FUN_HB, 0U, NULL, 0U, 0U);
    return 1U;
}

void app_lora_net_open_rx_window(uint32_t ms)
{
    net_listen(ms);
}

uint8_t app_lora_net_busy(void)
{
    return (s_state != NET_ST_IDLE) ? 1U : 0U;
}

void app_lora_net_task(void)
{
    uint8_t b;

    /* 1) 把接收 FIFO 里的字节全部喂给解析器(非阻塞) */
    while (app_lora_read_byte(&b) != 0U)
    {
        net_rx_byte(b);
    }

    /* 2) 半帧超时保护 */
    if ((s_rx_idx != 0U) && ((HAL_GetTick() - s_rx_tick) > (uint32_t)APP_LORA_NET_FRAME_GAP_MS))
    {
        s_rx_idx = 0U;
    }

    /* 3) 状态机推进 */
    switch (s_state)
    {
    case NET_ST_BACKOFF:
        if (net_expired(s_deadline) != 0U)
        {
            net_tx_go();
        }
        break;

    case NET_ST_WAIT_ACK:
        if (net_expired(s_deadline) != 0U)
        {
            net_ack_timeout();
        }
        break;

    case NET_ST_LISTEN:
        if (net_expired(s_deadline) != 0U)
        {
            s_state = NET_ST_IDLE;
            net_sleep();                        /* 窗口关了就休眠, 否则模块 10mA 级待机 */
        }
        break;

    default:
        break;
    }
}

const app_lora_net_stats_t *app_lora_net_get_stats(void)
{
    return &s_stats;
}
