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
 * 三、上报帧协议(v1.5:带 SEQ 与 ACK,详见 app_config.h "四" 节)
 *
 * 上行帧(设备 -> 主机,信号帧/电量帧各一帧,背靠背发出):
 *   [0] 0x5A   [1] 本机地址   [2] LEN=7(整帧字节数,含帧头与 CRC)
 *   [3] FUN    [4] SEQ(每发一帧 +1,重发换新)   [5] 数据域   [6] CRC8(对 [0..5])
 *
 * 下行帧(主机 -> 设备;ACK 与点名命令同布局):
 *   [0] 0x5A   [1] 目标设备地址   [2] LEN=7(命令无数据域时为 5)
 *   [3] FUN|0x80(ACK)/FUN(命令)   [4] CORR 或命令SEQ   [5] CODE 或数据   [6] CRC8
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

/* 清空接收 FIFO 残留(交易前后用) */
static void lora_cfg_flush_rx(void)
{
    uint8_t b;

    while (app_lora_read_byte(&b) != 0U)
    {
        /* 丢弃 */
    }
}

/*********************************
 * 一、配置包打包以及接收
 * 亿佰特 E22/E220 系列芯片配置参数需要在休眠模式下:
 *   写: C0 + 5字节工作参数，共6字节（掉电保存）
 *   读: 主机发：C1C1C1 模块回：已配置的参数.
 *       主机发：C3C3C3 模块回：版本信息
 *       主机发：C4C4C4 模块复位
 *********************************/
/* 进入配置模式:切 M0/M1 + 等模块稳定 + 等 AUX 就绪 */
static void lora_cfg_mode_enter(void)
{
    uint32_t t0;

    app_lora_set_mode(LORA_MODE_SLEEP);         /* M1=1,M0=1:配置模式 */
    HAL_Delay(LORA_CFG_SWITCH_MS);

    /* 空闲时 AUX 为高;超时也继续,按最快路径尝试 */
    t0 = HAL_GetTick();
    while ((bsp_gpio_sig_level(BSP_LORA_AUX) == BSP_GPIO_LOW) &&
           ((HAL_GetTick() - t0) < LORA_CFG_READY_MS))
    {
        HAL_Delay(1U);
    }
}

/* 退出配置模式:切回正常传输模式 */
static void lora_cfg_mode_exit(void)
{
    app_lora_set_mode(LORA_MODE_NORMAL);
    HAL_Delay(LORA_CFG_SWITCH_MS);
}

/* 发一帧厂商帧并等回包(收满 rx_len 字节)
 * @retval 1=收满  0=超时 */
static uint8_t lora_cfg_transaction(const uint8_t *tx, uint8_t tx_len,
                                    uint8_t *rx, uint8_t rx_len)
{
    uint32_t t0;
    uint8_t  got = 0U;

    lora_cfg_flush_rx();
    app_lora_send_bytes(tx, tx_len);

    // 如果不需要接收回包,直接返回成功
    if(rx_len == 0U || rx == NULL)
    {
        return 1U;
    }

    t0 = HAL_GetTick();
    while ((got < rx_len) && ((HAL_GetTick() - t0) < LORA_CFG_REPLY_MS))
    {
        if (app_lora_read_byte(&rx[got]) != 0U)
        {
            got++;
            t0 = HAL_GetTick();     /* 每收到一个字节续期,容忍慢回包 */
        }
    }
    return (got == rx_len) ? 1U : 0U;
}

void app_lora_enter_sleep_mode(void)
{
    lora_cfg_mode_enter();
}

uint8_t app_lora_cfg_write(const lora_reg_parm_cfg_t cfg)
{
    uint8_t tx[1U + APP_LORA_CFG_DATA_MAX];
    uint8_t ok = 0U;
    uint8_t i;

    tx[0] = LORA_CFG_CMD_WRITE;

    for (i = 0U; i < APP_LORA_CFG_DATA_MAX; i++)
    {
        tx[1U + i] = cfg.data[i];
    }

    lora_cfg_mode_enter();
    if (lora_cfg_transaction(tx, (uint8_t)(1U + APP_LORA_CFG_DATA_MAX), NULL, 0) != 0U)
    {
        ok = 1U;
    }
    lora_cfg_mode_exit();

    return ok;
}


uint8_t app_lora_cfg_read(lora_reg_parm_cfg_t *buf)
{
    uint8_t tx[3U];
    uint8_t rx[APP_LORA_CFG_DATA_MAX + 1]; //返回C0 + 数据域(5Byte)
    uint8_t ok = 0U;
    uint8_t i;

    if (buf == NULL)
    {
        return 0U;
    }

    tx[0] = LORA_CFG_CMD_READ;
    tx[1] = LORA_CFG_CMD_READ;
    tx[2] = LORA_CFG_CMD_READ;

    lora_cfg_mode_enter();

    // 虽然用DMA，但是实际接口为阻塞发送，因此tx为局部变量是安全的，但最好用静态变量以防止未来接口修改为非阻塞模式
    if (lora_cfg_transaction(tx, 3U, rx, (uint8_t)(sizeof(rx))) != 0U)
    {
        for (i = 0U; i < APP_LORA_CFG_DATA_MAX; i++)
        {
            buf->data[i] = rx[i + 1];
        }
        ok = 1U;
    }
    lora_cfg_mode_exit();

    return ok;
}

/**
 * @brief 一次配置模式下完成 "写 + 读回" 并比对
 * @retval 1=写回显与读回都一致 0=失败
 */
static uint8_t lora_cfg_write_read_verify(const lora_reg_parm_cfg_t val)
{
    uint8_t tx[1U + APP_LORA_CFG_DATA_MAX];
    lora_reg_parm_cfg_t rx;
    uint8_t ok = 0U;
    uint8_t i;

    lora_cfg_mode_enter();

    /* 1) 写:模块应原样回显 */
    tx[0] = LORA_CFG_CMD_WRITE;
    for (i = 0U; i < APP_LORA_CFG_DATA_MAX; i++)
    {
        tx[1U + i] = val.data[i];
    }
    // 虽然用DMA，但是实际接口为阻塞发送，因此tx为局部变量是安全的，但最好用静态变量以防止未来接口修改为非阻塞模式
    if (lora_cfg_transaction(tx, (uint8_t)(1U + APP_LORA_CFG_DATA_MAX), NULL, 0) != 0U)
    {
        ok = 1U;
    }

    /* 2) 读回数据,解析后与期望值逐个比对 */
    if(app_lora_cfg_read(&rx) != 0U)
    {
        for (i = 0U; i < APP_LORA_CFG_DATA_MAX; i++)
        {
            if (rx.data[i] != val.data[i])
            {
                ok = 0U;
                break;
            }
        }
    }

    lora_cfg_mode_exit();

    return ok;
}

uint8_t app_lora_cfg_reg_verify(lora_reg_parm_cfg_t val)
{
    return lora_cfg_write_read_verify(val);
}


/*********************************
 * 二、上行交付事务(两帧连发 + 单窗口收双 ACK + 缺帧重发)
 *
 * 一轮上报 = 信号帧(0x01) + 电量帧(0x02) 背靠背发出,两帧各带 SEQ;
 * 主机对两帧分别回 ACK(FUN|0x80,CORR=被确认的 SEQ,CODE=0 为成功);
 * 设备在一个窗口内收两个 ACK:
 *   - 收齐      -> 本轮交付成功
 *   - 窗口超时  -> 只补发缺 ACK 的那一帧(换新 SEQ),最多重发 APP_LORA_RETRY_MAX 次
 *   - 重试用尽  -> 该帧记失败统计,不影响另一帧与下一次上报
 * 事务进行中再次提交快照 -> 记为"最新快照",本轮结束后自动补发一轮
 *********************************/

#define UPLINK_BIT_SIG   0x01U      /* 位图:信号帧 */
#define UPLINK_BIT_PWR   0x02U      /* 位图:电量帧 */
#define UPLINK_BIT_ALL   (UPLINK_BIT_SIG | UPLINK_BIT_PWR)

typedef enum
{
    UPLINK_IDLE = 0,    /* 无进行中的事务      */
    UPLINK_BACKOFF,     /* 退避中,到期发送帧   */
    UPLINK_WAIT_ACK,    /* 已发送,窗口内等 ACK */
} uplink_state_t;

static struct
{
    uplink_state_t state;
    uint8_t  round_sig;       /* 本轮快照(本轮所有发送都用它,保证一轮数据一致) */
    uint8_t  round_pwr;
    uint8_t  latest_sig;      /* 事务进行中收到的新快照 */
    uint8_t  latest_pwr;
    uint8_t  reshoot;         /* 1=本轮结束后立即再发一轮新快照 */
    uint8_t  pending_mask;    /* 当前窗口还在等哪些帧的 ACK */
    uint8_t  ack_mask;        /* 已确认的帧 */
    uint8_t  seq_sig;         /* 信号帧当前 SEQ */
    uint8_t  seq_pwr;         /* 电量帧当前 SEQ */
    uint8_t  retry_left_sig;  /* 信号帧剩余重发次数 */
    uint8_t  retry_left_pwr;  /* 电量帧剩余重发次数 */
    uint8_t  backoff_ms;      /* 本次退避时长(ms) */
    uint32_t tick;            /* 进入当前状态/开始窗口的时刻 */
    uint8_t  seq_next;        /* 共享 SEQ 计数器:每发一帧 +1 */

    app_lora_uplink_stats_t stats;
} s_uplink;

/* 组一帧上行数据并立即发送(payload 1 字节)
 * 注:lora_send 为阻塞语义(DMA 发完才返回),tx 放栈上是安全的 */
static void uplink_send_frame(uint8_t fun, uint8_t seq, uint8_t payload)
{
    uint8_t tx[LORA_FRAME_UPLINK_LEN];

    tx[0] = LORA_FRAME_HEAD;
    tx[1] = (uint8_t)APP_DEVICE_ADDR;
    tx[2] = (uint8_t)LORA_FRAME_UPLINK_LEN;
    tx[3] = fun;
    tx[4] = seq;
    tx[5] = payload;
    tx[6] = lora_crc8_cal(tx, 6U);

    app_lora_send_bytes(tx, (uint16_t)sizeof(tx));
}

/* 开始一轮:两帧都待发,重试额度重置 */
static void uplink_begin_round(uint8_t sig, uint8_t power)
{
    s_uplink.round_sig = sig;
    s_uplink.round_pwr = power;
    s_uplink.ack_mask = 0U;
    s_uplink.pending_mask = UPLINK_BIT_ALL;
    s_uplink.retry_left_sig = (uint8_t)APP_LORA_RETRY_MAX;
    s_uplink.retry_left_pwr = (uint8_t)APP_LORA_RETRY_MAX;
    s_uplink.backoff_ms = (uint8_t)APP_LORA_TX_BACKOFF_MS;
    s_uplink.tick = HAL_GetTick();
    s_uplink.state = UPLINK_BACKOFF;

    dbg_printf("[UPLINK] round start: sig=0x%02X pwr=%u\r\n",
               (unsigned int)sig, (unsigned int)power);
}

/* 发送 pending 掩码里的帧(每帧换新 SEQ),然后进入等待窗口 */
static void uplink_send_pending(void)
{
    if ((s_uplink.pending_mask & UPLINK_BIT_SIG) != 0U)
    {
        s_uplink.seq_sig = s_uplink.seq_next++;
        uplink_send_frame(LORA_FUN_SIGNAL, s_uplink.seq_sig, s_uplink.round_sig);
    }

    if ((s_uplink.pending_mask & UPLINK_BIT_PWR) != 0U)
    {
        s_uplink.seq_pwr = s_uplink.seq_next++;
        uplink_send_frame(LORA_FUN_POWER, s_uplink.seq_pwr, s_uplink.round_pwr);
    }

    s_uplink.tick = HAL_GetTick();          /* 窗口从最后一帧发出后开始计 */
    s_uplink.state = UPLINK_WAIT_ACK;
}

/* 结束本轮:结算统计;若事务期间来过新快照,立即补发一轮 */
static void uplink_finish_round(void)
{
    if (s_uplink.ack_mask == UPLINK_BIT_ALL)
    {
        s_uplink.stats.round_ok++;
        dbg_printf("[UPLINK] round ok (sig+pwr acked)\r\n");
    }

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

/* 窗口超时:对还缺 ACK 的帧按剩余额度重发;额度用尽则记失败 */
static void uplink_window_timeout(void)
{
    uint8_t resend = 0U;

    if ((s_uplink.pending_mask & UPLINK_BIT_SIG) != 0U)
    {
        if (s_uplink.retry_left_sig > 0U)
        {
            s_uplink.retry_left_sig--;
            resend |= UPLINK_BIT_SIG;
            s_uplink.stats.retry_cnt++;
        }
        else
        {
            s_uplink.stats.sig_fail++;
            dbg_printf("[UPLINK] SIG retry exhausted\r\n");
        }
    }

    if ((s_uplink.pending_mask & UPLINK_BIT_PWR) != 0U)
    {
        if (s_uplink.retry_left_pwr > 0U)
        {
            s_uplink.retry_left_pwr--;
            resend |= UPLINK_BIT_PWR;
            s_uplink.stats.retry_cnt++;
        }
        else
        {
            s_uplink.stats.pwr_fail++;
            dbg_printf("[UPLINK] PWR retry exhausted\r\n");
        }
    }

    s_uplink.pending_mask = resend;

    if (resend == 0U)
    {
        uplink_finish_round();
    }
    else
    {
        s_uplink.backoff_ms = (uint8_t)APP_LORA_RETRY_BACKOFF_MS;   /* 重发前退避,错开碰撞 */
        s_uplink.tick = HAL_GetTick();
        s_uplink.state = UPLINK_BACKOFF;
    }
}

/* ACK 到达:按 (帧类型, CORR) 匹配当前窗口内对应的帧 */
static void uplink_ack_rx(uint8_t fun, uint8_t corr, uint8_t code)
{
    uint8_t bit;

    if (s_uplink.state != UPLINK_WAIT_ACK)
    {
        return;                     /* 不在等 ACK(迟到/重复包):忽略 */
    }

    if (fun == LORA_FUN_SIGNAL)
    {
        bit = UPLINK_BIT_SIG;
        if (((s_uplink.pending_mask & bit) == 0U) || (corr != s_uplink.seq_sig))
        {
            return;                 /* 不是当前在等的那一帧:忽略 */
        }
    }
    else if (fun == LORA_FUN_POWER)
    {
        bit = UPLINK_BIT_PWR;
        if (((s_uplink.pending_mask & bit) == 0U) || (corr != s_uplink.seq_pwr))
        {
            return;
        }
    }
    else
    {
        return;                     /* 未知 ACK:忽略 */
    }

    s_uplink.pending_mask &= (uint8_t)~bit;     /* 该帧已有结论 */

    if (code == LORA_ACK_CODE_OK)
    {
        s_uplink.ack_mask |= bit;
        dbg_printf("[UPLINK] %s acked (seq=%u)\r\n",
                   (bit == UPLINK_BIT_SIG) ? "SIG" : "PWR", (unsigned int)corr);
    }
    else
    {
        /* 主机明确拒收:重发没有意义,直接记失败 */
        if (bit == UPLINK_BIT_SIG)
        {
            s_uplink.stats.sig_fail++;
        }
        else
        {
            s_uplink.stats.pwr_fail++;
        }
        dbg_printf("[UPLINK] %s rejected, code=%u\r\n",
                   (bit == UPLINK_BIT_SIG) ? "SIG" : "PWR", (unsigned int)code);
    }
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
            uplink_send_pending();
        }
        break;

    case UPLINK_WAIT_ACK:
        if (s_uplink.pending_mask == 0U)
        {
            uplink_finish_round();      /* 窗口内两帧都已有结论 */
        }
        else if ((now - s_uplink.tick) >= APP_LORA_ACK_TIMEOUT_MS)
        {
            uplink_window_timeout();    /* 缺 ACK:重发或记失败 */
        }
        break;

    default:
        break;
    }
}

/* ---- 对外接口 ---- */

void app_lora_uplink_status(uint8_t sig, uint8_t power)
{
    if (s_uplink.state == UPLINK_IDLE)
    {
        uplink_begin_round(sig, power);
    }
    else if ((s_uplink.state == UPLINK_BACKOFF) && (s_uplink.ack_mask == 0U))
    {
        /* 帧还没发出去:直接刷新本轮快照 */
        s_uplink.round_sig = sig;
        s_uplink.round_pwr = power;
    }
    else
    {
        /* 已发出/等 ACK:存为最新快照,本轮结束后自动补发一轮 */
        s_uplink.latest_sig = sig;
        s_uplink.latest_pwr = power;
        s_uplink.reshoot = 1U;
    }
}

uint8_t app_lora_uplink_busy(void)
{
    return (s_uplink.state != UPLINK_IDLE) ? 1U : 0U;
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
void app_lora_signal(uint8_t sig)
{
    if (app_lora_get_status() == LORA_IDLE)
    {
        /* 点名应答:带 SEQ 立即发出,不等 ACK(主机按请求自行关联) */
        uplink_send_frame(LORA_FUN_SIGNAL, s_uplink.seq_next++, sig);
    }
}

void app_lora_power(uint8_t power)
{
    if (app_lora_get_status() == LORA_IDLE)
    {
        uplink_send_frame(LORA_FUN_POWER, s_uplink.seq_next++, power);
    }
}

void app_lora_set_signal(uint8_t *signal)
{
    signal_status = signal;
}

void app_lora_set_power(uint8_t *power)
{
    power_status = power;
}

/* lora 接收消息解析 */
/* ---- 整帧校验通过后:ACK 匹配 -> 功能码 switch 落地;其余丢弃 ---- */
static void lora_rx_parse(const uint8_t *f, uint8_t len)
{
    uint8_t fun = f[3];

    /* 下行帧 [1] = 目标设备地址:只处理"发给本机"或"广播"的帧。
     * 主机模块自身是 0xFFFF(广播)也没关系 —— 过滤看的是帧里填的目标地址。 */
    if ((f[1] != (uint8_t)APP_DEVICE_ADDR) && (f[1] != LORA_ADDR_BROADCAST))
    {
        return;
    }

    /* ACK:FUN bit7=1,数据域 = CORR(被确认帧的SEQ) + CODE。
     * ACK 必须精确指向本机:广播 ACK 一律忽略,避免多台设备 SEQ 相同时互相误确认。 */
    if ((fun & LORA_FRAME_ACK_BIT) != 0U)
    {
        if ((f[1] == (uint8_t)APP_DEVICE_ADDR) && (len >= LORA_FRAME_ACK_LEN))
        {
            uplink_ack_rx((uint8_t)(fun & (uint8_t)~LORA_FRAME_ACK_BIT), f[4], f[5]);
        }
        return;
    }

    switch (fun)
    {
    case LORA_FUN_GET_POWER:                       /* 主动获取电量(点名) */
        if (power_status != NULL)
        {
            app_lora_power(*power_status);
        }
        break;

    case LORA_FUN_GET_SIGNAL:                       /* 主动获取ABC相挂接信号(点名) */
        if (signal_status != NULL)
        {
            app_lora_signal(*signal_status);
        }
        break;
    default:                                        /* 未定义功能码:丢弃 */
        break;
    }
}

/* ---- 接收:找帧头 -> 按 LEN 收满 -> CRC 校验 -> 解析 ---- */
static uint8_t  s_rx_buf[LORA_FRAME_BUF_MAX];
static uint8_t  s_rx_idx;       /* 0 = 正在找帧头,其余为已收字节数 */
static uint32_t s_rx_tick;      /* 最近一个字节的时刻,用于字节间超时 */

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
        s_rx_idx = 0U;
    }
}

void app_lora_process(void)
{
    uint8_t b;

    /* 1) 收:把接收 FIFO 里的字节全部喂给状态机(非阻塞,有多少吃多少)。
     *    注:不再依赖“模块唤醒标志”——AUX(PA0) 没有 EXTI(要让出 EXTI0 给 PB0),
     *    该标志不会置位;直接抽干 FIFO,ACK/下行命令才能被处理。 */
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
    uplink_poll();
}
