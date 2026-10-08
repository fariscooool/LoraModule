/**
 * @file app_lora_config.c
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-10-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */


 #include "app_lora.h"
 #include "app_lora_config.h"
 #include "app_config.h"
 #include "bsp_gpio.h"
#include "main.h"


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
