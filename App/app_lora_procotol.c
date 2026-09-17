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
#include "app_lora.h"
#include "main.h"

/* 自定义协议帧信息 */
#define LORA_FRAME_HEAD 0x5A
#define LORA_FUN_HEARTBEAT 0x00
#define LORA_FUN_SIGNAL 0x01
#define LORA_FUN_POWER 0x02

#define LORA_CFG_CMD_WRITE  0xC0U   /* 写寄存器(掉电保存);临时写可改 0xC2 */
#define LORA_CFG_CMD_READ   0xC1U   /* 读寄存器 */
#define LORA_CFG_SWITCH_MS  20U     /* 模式切换后等待模块稳定(ms) */
#define LORA_CFG_READY_MS   100U    /* 等 AUX 就绪的超时(ms) */
#define LORA_CFG_REPLY_MS   300U    /* 等配置回包的超时(ms) */

/* 单次配置读/写的最大数据字节数 */
#define APP_LORA_CFG_DATA_MAX   8U

/* 心跳帧内容(固定 4 字节:0x5A 0x04 0x00 0xF7) */
static const uint8_t lora_hb_frame[4] = { 0x5A, 0x04, 0x00, 0xF7 };
static uint8_t lora_sig_frame[5] = { LORA_FRAME_HEAD, 0x05, LORA_FUN_SIGNAL, 0x00, 0xF7 };
static uint8_t lora_power_frame[5] = { LORA_FRAME_HEAD, 0x05, LORA_FUN_POWER, 0x00, 0xF7 };

/*********************************
 * 静态函数
 *********************************/
// CRC 8-CCITT (多项式 0x07)
static uint8_t lora_crc8_cal(uint8_t *data, uint16_t len)
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

static void app_lora_heartbeat(void)
{
    
    if(app_lora_get_status() == LORA_IDLE)
    {
        app_lora_send_bytes(lora_hb_frame, (uint16_t)sizeof(lora_hb_frame));
    }
}

static void app_lora_signal(uint8_t sig)
{
    if(app_lora_get_status() == LORA_IDLE)
    {
        lora_sig_frame[3] = sig;
        lora_sig_frame[4] = lora_crc8_cal(lora_sig_frame, 4);
        app_lora_send_bytes(lora_sig_frame, (uint16_t)sizeof(lora_sig_frame));
    }
}

static void app_lora_power(uint8_t power)
{
    if(app_lora_get_status() == LORA_IDLE)
    {
        lora_power_frame[3] = power;
        lora_power_frame[4] = lora_crc8_cal(lora_power_frame, 4);
        app_lora_send_bytes(lora_power_frame, (uint16_t)sizeof(lora_power_frame));
    }
}

/*********************************
 * 模块配置(寄存器帧,本地调用)
 * 厂商配置帧默认按"寄存器帧"类模块实现(如亿佰特 E22/E220 系列):
 *   写: C0 + 寄存器地址 + 数据...   模块回显同一帧
 *   读: C1 + 寄存器地址 + 长度      模块回 C1 + 地址 + 数据...
 * !! 按你的模块手册核对命令字(0xC0/0xC1)与回包格式 !!
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

/* 清空接收 FIFO 残留(交易前后用) */
static void lora_cfg_flush_rx(void)
{
    uint8_t b;

    while (app_lora_read_byte(&b) != 0U)
    {
        /* 丢弃 */
    }
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

uint8_t app_lora_cfg_write(uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t tx[2U + APP_LORA_CFG_DATA_MAX];
    uint8_t rx[2U + APP_LORA_CFG_DATA_MAX];
    uint8_t ok = 0U;
    uint8_t i;

    if ((data == NULL) || (len == 0U) || (len > APP_LORA_CFG_DATA_MAX))
    {
        return 0U;
    }

    tx[0] = LORA_CFG_CMD_WRITE;
    tx[1] = reg;
    for (i = 0U; i < len; i++)
    {
        tx[2U + i] = data[i];
    }

    lora_cfg_mode_enter();
    if (lora_cfg_transaction(tx, (uint8_t)(2U + len), rx, (uint8_t)(2U + len)) != 0U)
    {
        ok = 1U;                        /* 模块应回显同一帧 */
        for (i = 0U; i < (uint8_t)(2U + len); i++)
        {
            if (rx[i] != tx[i])
            {
                ok = 0U;
                break;
            }
        }
    }
    lora_cfg_mode_exit();

    return ok;
}



uint8_t app_lora_cfg_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx[3U];
    uint8_t rx[2U + APP_LORA_CFG_DATA_MAX];
    uint8_t ok = 0U;
    uint8_t i;

    if ((buf == NULL) || (len == 0U) || (len > APP_LORA_CFG_DATA_MAX))
    {
        return 0U;
    }

    tx[0] = LORA_CFG_CMD_READ;
    tx[1] = reg;
    tx[2] = len;

    lora_cfg_mode_enter();
    if (lora_cfg_transaction(tx, 3U, rx, (uint8_t)(2U + len)) != 0U)
    {
        if ((rx[0] == LORA_CFG_CMD_READ) && (rx[1] == reg))
        {
            for (i = 0U; i < len; i++)
            {
                buf[i] = rx[2U + i];
            }
            ok = 1U;
        }
    }
    lora_cfg_mode_exit();

    return ok;
}


void app_lora_update(void)
{
   
}
