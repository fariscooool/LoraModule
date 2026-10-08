#ifndef APP_LORA_CONFIG_H
#define APP_LORA_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "app_lora.h"

// LoRa 配置寄存器操作接口================================
/**
 * @brief 写一个模块寄存器(等待模块回显)
 * @param  reg 寄存器地址
 * @param  data 待写入数据
 * @param  len  字节数(<=8)
 * @retval 1=回显一致 0=失败
 */
uint8_t app_lora_cfg_write(const lora_reg_parm_cfg_t cfg);

/**
 * @brief 读一个模块寄存器(解析 C1+REG+DATA 回包)
 * @param  buf 读回数据缓冲区
 * @retval 1=成功 0=失败(超时或回包头不匹配)
 */
uint8_t app_lora_cfg_read(lora_reg_parm_cfg_t *buf);

/**
 * @brief 将配置参数写进模块寄存器(一次配置模式内完成 "写(回显校验) + 读(读回比对)")
 * @param  val 期望值
 * @retval 1=写回显与读回都一致 0=失败
 */
uint8_t app_lora_cfg_reg_verify(lora_reg_parm_cfg_t val);

/**
 * @brief 让 LoRa 模块进入睡眠模式,降低设备整体功耗
 */
void app_lora_enter_sleep_mode(void);


#ifdef __cplusplus
}
#endif

#endif // APP_LORA_CONFIG_H