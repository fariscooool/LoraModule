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



// LoRa 协议处理接口================================
/**
 * @brief 发送信号帧，lora 自定义协议比较简单，只有触点信号帧和电量帧
 * 
 * @param sig 
 */
void app_lora_signal(uint8_t sig);

/**
 * @brief 发送功率帧
 * 
 * @param power 0~100 表示电池电量百分比
 */
void app_lora_power(uint8_t power);

/* ================ 上报交付事务(两帧连发 + 单窗口双 ACK + 缺帧重发) ================ */

/**
 * @brief 上行交付统计(调试/现场诊断用)
 * @note  APP_LORA_ACK_ENABLE=0(简化版)时不做交付确认,本组计数恒为 0
 */
typedef struct
{
    uint16_t round_ok;    /* 完整交付(两帧都确认)的轮次数 */
    uint16_t sig_fail;    /* 信号帧重试耗尽/被主机拒绝次数 */
    uint16_t pwr_fail;    /* 电量帧重试耗尽/被主机拒绝次数 */
    uint16_t retry_cnt;   /* 累计重发帧次数 */
} app_lora_uplink_stats_t;

/**
 * @brief 提交一轮上报(信号+电量)
 * @param sig   信号数据域(如 APP_SIG_HOOKED_OK / APP_SIG_HOOKED_FAIL)
 * @param power 电量 0~100;0xFF=无效
 * @note  非阻塞。
 *        APP_LORA_ACK_ENABLE=1:后台事务负责两帧连发、单窗口等双 ACK、缺帧重发、
 *          超时重试;两帧都确认(或重试用尽)前 busy() 保持 1;事务进行中再次调用:
 *          数据记为"最新快照",本轮结束后自动补发一轮。
 *        APP_LORA_ACK_ENABLE=0(简化版):两帧背靠背发出即返回,不等 ACK、
 *          不重发、不合并,下一次调用立即再发一轮。
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