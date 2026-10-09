#ifndef APP_LORA_PROCOTOL_H
#define APP_LORA_PROCOTOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "app_lora.h"


/**
 * @brief 主机下发的控制参数(解析后的落地值)
 * @note  由 app_lora_process() 中的接收状态机解析后更新
 */
typedef struct
{
    uint8_t  report_enable;      /* 0=停止上报 1=允许上报 */
    uint16_t report_period_s;    /* 周期上报间隔(秒)      */
    uint16_t device_addr;        /* 模块地址              */
    uint8_t  channel;            /* 模块信道              */
} app_lora_ctrl_t;



/* ================ 上报交付事务(一帧快照 + ACK 确认/重发) ================ */

/**
 * @brief 设备唯一标识短码(96bit UID 经 FNV-1a 派生的 32bit)
 * @note  首次调用时读取 HAL_GetUIDw0/1/2 并缓存;随每帧下发,主机据此建白名单过滤
 */
uint32_t app_lora_uid32(void);

/**
 * @brief 上行交付统计(调试/现场诊断用)
 * @note  APP_LORA_ACK_ENABLE=0(简化版)时不做交付确认,本组计数恒为 0
 */
typedef struct
{
    uint16_t round_ok;    /* 确认成功的轮次数 */
    uint16_t round_fail;  /* 重试用尽仍未确认的轮次数 */
    uint16_t rejected;    /* 被主机拒收(ACK CODE≠0)的次数 */
    uint16_t retry_cnt;   /* 累计重发次数 */
} app_lora_uplink_stats_t;

/**
 * @brief 提交一轮上报(信号+电量,合并为一帧 0x03 快照帧)
 * @param sig   信号数据域(如 APP_SIG_HOOKED_OK / APP_SIG_HOOKED_FAIL)
 * @param power 电量 0~100;0xFF=无效
 * @note  非阻塞。
 *        APP_LORA_ACK_ENABLE=1:后台事务负责发送快照帧、等 ACK、超时重发(换新 SEQ);
 *          确认成功(或重试用尽)前 busy() 保持 1;事务进行中再次调用:
 *          数据记为"最新快照",本轮结束后自动补发一轮。
 *        APP_LORA_ACK_ENABLE=0(简化版):快照帧发出即返回,不等 ACK、
 *          不重发,下一次调用立即再发一轮。
 */
void app_lora_uplink_status(uint8_t sig, uint8_t power);

/**
 * @brief 上报事务是否未完成(等 ACK / 重发中)
 * @retval 1=未完成(不要进 Stop)  0=空闲
 * @note  APP_LORA_ACK_ENABLE=0 时恒为 0(帧已发完,可立即进 Stop)
 */
uint8_t app_lora_uplink_busy(void);

/**
 * @brief 读取上行交付统计
 * @param  out 输出的统计结构(可为 NULL)
 */
void app_lora_uplink_get_stats(app_lora_uplink_stats_t *out);

/**
 * @brief 设置外部IO状态(信号)
 * @param signal 信号状态指针
 */
void app_lora_set_signal(uint8_t *signal);

/**
 * @brief 设置电池电量状态指针
 * @param power 电池电量状态指针
 */
void app_lora_set_power(uint8_t *power);

/**
 * @brief LoRa 模块周期处理函数
 * @note  在主循环里周期性调用:
 *        唤醒事件 -> 把 FIFO 字节喂给接收状态机 -> 整帧收齐则校验并分发
 *        (非阻塞:来多少字节吃多少,不做忙等)
 */
void app_lora_process(void);


#ifdef __cplusplus
}
#endif

#endif // APP_LORA_PROCOTOL_H