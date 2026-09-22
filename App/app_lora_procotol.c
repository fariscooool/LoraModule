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
#include "main.h"

/* 自定义帧格式(收发一致,与现有发送函数保持一致):
 *   [0]      0x5A
 *   [1]      LEN  = 整帧字节数(含帧头与 CRC)
 *   [2]      FUN  功能码
 *   [3..]    DATA 数据域,长度 = LEN - 4
 *   [LEN-1]  CRC8 (对 [0..LEN-2] 计算,多项式 0x07)
 * 举例:心跳帧 0x5A 0x04 0x00 0xF7
 */
#define LORA_FRAME_HEAD     0x5A
#define LORA_FRAME_MIN_LEN  4U      /* 头+长度+FUN+CRC(数据域为空) */
#define LORA_FRAME_DATA_MAX 12U     /* 数据域上限 */
#define LORA_FRAME_BUF_MAX  (LORA_FRAME_MIN_LEN + LORA_FRAME_DATA_MAX)
#define LORA_FRAME_GAP_MS   50U     /* 字节间超时:超时则丢弃半帧 */

/* 上行(设备 -> 主机)功能码 */
#define LORA_FUN_HEARTBEAT 0x00     // 心跳帧
#define LORA_FUN_SIGNAL 0x01        // 信号帧，主要是ABC相开关状态
#define LORA_FUN_POWER 0x02         // 电池电量帧

/* 下行(主机 -> 设备)功能码:控制参数 */
#define LORA_FUN_GET_POWER   0x10U  /* 数据 0B:获取电池电量*/
#define LORA_FUN_GET_SIGNAL   0x11U  /* 数据 1B:主动获取ABC相开关状态*/
// #define LORA_FUN_SET_ADDR     0x12U  /* 数据 2B:模块地址(小端,掉电保存)  */
// #define LORA_FUN_SET_CHANNEL  0x13U  /* 数据 1B:模块信道(掉电保存)       */
// #define LORA_FUN_QUERY_CFG    0x14U  /* 数据 0B:请求回读当前配置         */

#define LORA_CFG_CMD_WRITE  0xC0U   /* 写寄存器(掉电保存);临时写可改 0xC2 */
#define LORA_CFG_CMD_READ   0xC1U   /* 读参数 */

/* 模块寄存器地址(E22/E220 系列,按你的模块手册核对) */
#define LORA_CFG_REG_ADDR_H   0x02U  /* 地址高 */
#define LORA_CFG_REG_ADDR_L   0x03U  /* 地址低 */
#define LORA_CFG_REG_CHANNEL  0x05U  /* 信道   */
#define LORA_CFG_SWITCH_MS  20U     /* 模式切换后等待模块稳定(ms) */
#define LORA_CFG_READY_MS   100U    /* 等 AUX 就绪的超时(ms) */
#define LORA_CFG_REPLY_MS   300U    /* 等配置回包的超时(ms) */

/* 单次配置读/写的最大数据字节数 */
#define APP_LORA_CFG_DATA_MAX   5U

/* 心跳帧内容(固定 4 字节:0x5A 0x04 0x00 0xF7) */
static const uint8_t lora_hb_frame[4] = { 0x5A, 0x04, 0x00, 0xF7 };
static uint8_t lora_sig_frame[5] = { LORA_FRAME_HEAD, 0x05, LORA_FUN_SIGNAL, 0x00, 0xF7 };
static uint8_t lora_power_frame[5] = { LORA_FRAME_HEAD, 0x05, LORA_FUN_POWER, 0x00, 0xF7 };

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

uint8_t app_lora_cfg_write(const lora_reg_parm_cfg_t cfg, uint8_t len)
{
    uint8_t tx[1U + APP_LORA_CFG_DATA_MAX];
    uint8_t ok = 0U;
    uint8_t i;

    if ((len == 0U) || (len > APP_LORA_CFG_DATA_MAX))
    {
        return 0U;
    }

    tx[0] = LORA_CFG_CMD_WRITE;

    for (i = 0U; i < len; i++)
    {
        tx[1U + i] = cfg.data[i];
    }

    lora_cfg_mode_enter();
    if (lora_cfg_transaction(tx, (uint8_t)(1U + len), NULL, 0) != 0U)
    {
        ok = 1U;
    }
    lora_cfg_mode_exit();

    return ok;
}


uint8_t app_lora_cfg_read(lora_reg_parm_cfg_t *buf, uint8_t len)
{
    uint8_t tx[3U];
    uint8_t rx[APP_LORA_CFG_DATA_MAX];
    uint8_t ok = 0U;
    uint8_t i;

    if ((buf == NULL) || (len == 0U) || (len > APP_LORA_CFG_DATA_MAX))
    {
        return 0U;
    }

    tx[0] = LORA_CFG_CMD_READ;
    tx[1] = LORA_CFG_CMD_READ;
    tx[2] = LORA_CFG_CMD_READ;

    lora_cfg_mode_enter();
    if (lora_cfg_transaction(tx, 3U, rx, (uint8_t)(sizeof(rx))) != 0U)
    {
        for (i = 0U; i < len; i++)
        {
            buf->data[i] = rx[i];
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
static uint8_t lora_cfg_write_read_verify(const lora_reg_parm_cfg_t val, uint8_t len)
{
    uint8_t tx[1U + APP_LORA_CFG_DATA_MAX];
    uint8_t rx[APP_LORA_CFG_DATA_MAX];
    uint8_t ok = 0U;
    uint8_t i;

    if (len != 5U)
    {
        return 0U;
    }

    lora_cfg_mode_enter();

    /* 1) 写:模块应原样回显 */
    tx[0] = LORA_CFG_CMD_WRITE;
    for (i = 0U; i < len; i++)
    {
        tx[1U + i] = val.data[i];
    }
    if (lora_cfg_transaction(tx, (uint8_t)(1U + len), NULL, 0) != 0U)
    {
        ok = 1U;
    }

    /* 2) 读回数据,解析后与期望值逐个比对 */
    if(app_lora_cfg_read((lora_reg_parm_cfg_t *)&rx, (uint8_t)sizeof(rx)) != 0U)
    {
        for (i = 0U; i < len; i++)
        {
            if (rx[i] != val.data[i])
            {
                ok = 0U;
                break;
            }
        }
    }

    lora_cfg_mode_exit();

    return ok;
}

uint8_t app_lora_cfg_reg_verify(lora_reg_parm_cfg_t val, uint8_t len)
{
    return lora_cfg_write_read_verify(val, len);
}


/*********************************
 * 二、主机自定义通信发送以及解析
 * 逐字节收包 -> 收满整帧 -> CRC8 校验 -> 功能码 switch 落地
 *********************************/

// 主机自定义通信发送接口，包含心跳、信号和电池电压
void app_lora_heartbeat(void)
{
    
    if(app_lora_get_status() == LORA_IDLE)
    {
        app_lora_send_bytes(lora_hb_frame, (uint16_t)sizeof(lora_hb_frame));
    }
}

void app_lora_signal(uint8_t sig)
{
    if(app_lora_get_status() == LORA_IDLE)
    {
        lora_sig_frame[3] = sig;
        lora_sig_frame[4] = lora_crc8_cal(lora_sig_frame, 4);
        app_lora_send_bytes(lora_sig_frame, (uint16_t)sizeof(lora_sig_frame));
    }
}

void app_lora_power(uint8_t power)
{
    if(app_lora_get_status() == LORA_IDLE)
    {
        lora_power_frame[3] = power;
        lora_power_frame[4] = lora_crc8_cal(lora_power_frame, 4);
        app_lora_send_bytes(lora_power_frame, (uint16_t)sizeof(lora_power_frame));
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

/* ---- 整帧校验通过后:功能码 switch 直接落地,长度不符/未知功能码一律丢弃 ---- */
static void lora_rx_parse(const uint8_t *f)
{
    switch (f[2])
    {
    case LORA_FUN_GET_POWER:                       /* 主动获取电量 */
        if (power_status != NULL)
        {
            app_lora_power(*power_status);
        }
        break;

    case LORA_FUN_GET_SIGNAL:                       /* 主动获取ABC相挂接信号 */
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

    if (s_rx_idx == 2U)                     /* 刚收完长度字节:先做范围检查 */
    {
        uint8_t len = s_rx_buf[1];

        if ((len < LORA_FRAME_MIN_LEN) || (len > (uint8_t)sizeof(s_rx_buf)))
        {
            s_rx_idx = 0U;                  /* 长度非法:丢帧重找帧头 */
        }
        return;
    }

    if (s_rx_idx == s_rx_buf[1])            /* 整帧收满:CRC 对才解析 */
    {
        uint8_t len = s_rx_buf[1];

        if (lora_crc8_cal(s_rx_buf, (uint16_t)(len - 1U)) == s_rx_buf[len - 1U])
        {
            lora_rx_parse(s_rx_buf);
        }
        s_rx_idx = 0U;
    }
}

void app_lora_process(void)
{
    uint8_t b;

    /* 无唤醒事件 / 正在发送:本轮不拆包 */
    if (app_lora_wakeup_flag() == 0U)
    {
        return;
    }
    if (app_lora_get_status() != LORA_IDLE)
    {
        return;
    }

    /* 1) 收:把接收 FIFO 里的字节全部喂给状态机(非阻塞,有多少吃多少) */
    while (app_lora_read_byte(&b) != 0U)
    {
        lora_rx_byte(b);
    }

    /* 2) 半帧超时保护:字节间超过 LORA_FRAME_GAP_MS 就丢弃重找帧头 */
    if ((s_rx_idx != 0U) && ((HAL_GetTick() - s_rx_tick) > LORA_FRAME_GAP_MS))
    {
        s_rx_idx = 0U;
    }
}
