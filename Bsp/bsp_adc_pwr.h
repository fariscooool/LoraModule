/**
 * @file bsp_adc_pwr.h
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief  电池电压检测(ADC1_IN6 / PA6)驱动接口
 * @version 0.1
 * @date 2026-09-23
 *
 * @note   硬件连接(请按实际板子确认):
 *
 *             Vbat ──[R1]──┬──[R2]── GND
 *                          └── PA6 (ADC1_IN6)
 *
 *         ADC 引脚上的电压 Vadc = Vbat * R2 / (R1 + R2),
 *         所以 Vbat = Vadc * BSP_ADC_PWR_DIV_NUM / BSP_ADC_PWR_DIV_DEN。
 *         例:R1 = 100k(上臂)、R2 = 100k(下臂) -> 2 / 1;
 *             电池直连 PA6(无分压)         -> 1 / 1。
 *
 * @note   参考电压 = VDD(=VDDA,该封装没有 VREF+ 引脚):
 *         - VDDA 稳定(如 3.3V LDO 供电):BSP_ADC_PWR_USE_VREFINT 可设 0,
 *           直接用标称 BSP_ADC_PWR_VREF_MV 换算;
 *         - VDDA 不稳、或 VDD 本身就是电池电压:保持 BSP_ADC_PWR_USE_VREFINT=1,
 *           驱动会用内部 VREFINT 实测 VDDA 再换算,结果不受 VDDA 漂移影响
 *           (此时 bsp_adc_pwr_read_vdd_mv() 读到的就是电池电压本身)。
 *
 * @note   低功耗:所有 read_xxx() 都是"临时开 ADC -> 转换 -> 立即关 ADC"
 *         (连调压器、VREFINT 缓冲一起关),测量间隙 ADC 完全不耗电;
 *         PA6 平时保持模拟输入(高阻),分压电阻上没有额外漏电流。
 *         因此应用层进 Stop 模式前不需要做任何处理。
 */
#ifndef BSP_ADC_PWR_H
#define BSP_ADC_PWR_H

#include <stdint.h>

#include "main.h"   /* 引入 HAL 类型 */

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * 可配置项(硬件相关)
 *   建议在 app_config.h 里统一定义(需在包含本头文件之前);
 *   这里给出默认值,方便单独调试本驱动。
 *============================================================================*/

/** 分压比:Vbat = Vadc * NUM / DEN */
#ifndef BSP_ADC_PWR_DIV_NUM
#define BSP_ADC_PWR_DIV_NUM     2U
#endif
#ifndef BSP_ADC_PWR_DIV_DEN
#define BSP_ADC_PWR_DIV_DEN     1U
#endif

/** 参考电压标称值(mV),即 VDDA。
 *  仅在 BSP_ADC_PWR_USE_VREFINT = 0 时使用。 */
#ifndef BSP_ADC_PWR_VREF_MV
#define BSP_ADC_PWR_VREF_MV     3300U
#endif

/** 1 = 用内部 VREFINT(1.224V)实测 VDDA 再换算(推荐,与 VDDA 实际值无关)
 *  0 = 直接使用标称 BSP_ADC_PWR_VREF_MV(快一点,但误差取决于 VDD 精度) */
#ifndef BSP_ADC_PWR_USE_VREFINT
#define BSP_ADC_PWR_USE_VREFINT 1U
#endif

/** 每次测量的采样次数(取平均,抑制随机噪声) */
#ifndef BSP_ADC_PWR_SAMPLES
#define BSP_ADC_PWR_SAMPLES     8U
#endif

/** 单次转换超时(ms) */
#ifndef BSP_ADC_PWR_TIMEOUT_MS
#define BSP_ADC_PWR_TIMEOUT_MS  5U
#endif

/** 等待 VREFINT 内部缓冲就绪的上限(ms) */
#ifndef BSP_ADC_PWR_VREFINT_READY_MS
#define BSP_ADC_PWR_VREFINT_READY_MS  5U
#endif

/** 电量百分比映射区间(mV):Vbat <= EMPTY -> 0%,Vbat >= FULL -> 100%
 *  !! 占位值(按 2.0V ~ 3.0V 电池组),请按实际电池修改 !! */
#ifndef BSP_ADC_PWR_BAT_EMPTY_MV
#define BSP_ADC_PWR_BAT_EMPTY_MV  2000U
#endif
#ifndef BSP_ADC_PWR_BAT_FULL_MV
#define BSP_ADC_PWR_BAT_FULL_MV   3000U
#endif

/** 低压告警阈值(mV) */
#ifndef BSP_ADC_PWR_LOW_MV
#define BSP_ADC_PWR_LOW_MV       2200U
#endif

/*==============================================================================
 * 类型定义
 *============================================================================*/

typedef enum
{
    BSP_ADC_PWR_OK = 0,         /* 成功                 */
    BSP_ADC_PWR_ERR_INIT,       /* ADC 初始化/校准失败  */
    BSP_ADC_PWR_ERR_TIMEOUT,    /* 转换超时             */
    BSP_ADC_PWR_ERR_PARAM       /* 出参指针为空         */
} bsp_adc_pwr_status_t;

/*==============================================================================
 * 接口
 *============================================================================*/

/**
 * @brief 初始化并校准 ADC(上电自检用,可选)
 * @note  不调用也没关系:每个 read_xxx() 内部都会自己完成"初始化 + 校准",
 *        调用它主要是为了在上电时尽早发现硬件问题。
 * @retval BSP_ADC_PWR_OK 表示 ADC 可用
 */
bsp_adc_pwr_status_t bsp_adc_pwr_init(void);

/**
 * @brief 读电池检测通道的 ADC 原始码(0 ~ 4095)
 * @param raw_avg [out] 已按 BSP_ADC_PWR_SAMPLES 次平均后的原始码
 */
bsp_adc_pwr_status_t bsp_adc_pwr_read_raw(uint16_t *raw_avg);

/**
 * @brief 读电池电压
 * @param bat_mv [out] 电池电压(mV),已按分压比还原到分压前
 */
bsp_adc_pwr_status_t bsp_adc_pwr_read_mv(uint32_t *bat_mv);

/**
 * @brief 读实测 VDDA(即 ADC 的参考电压),单位 mV
 * @note  若 VDD 由电池直接供电,这个值就是电池电压本身。
 * @retval 失败时返回标称 BSP_ADC_PWR_VREF_MV 兜底值
 */
bsp_adc_pwr_status_t bsp_adc_pwr_read_vdd_mv(uint32_t *vdd_mv);

/**
 * @brief 电压 -> 电量百分比(0 ~ 100),按 EMPTY / FULL 线性映射并截断
 */
uint8_t bsp_adc_pwr_percent(uint32_t bat_mv);

/**
 * @brief 是否低于低压阈值 BSP_ADC_PWR_LOW_MV
 * @retval 1 = 电压偏低  0 = 正常
 */
uint8_t bsp_adc_pwr_is_low(uint32_t bat_mv);

/**
 * @brief 手动关闭 ADC(含调压器、VREFINT 内部缓冲),PA6 恢复模拟输入
 * @note  所有 read_xxx() 结束后都会自动调用,应用层一般不需要再调用。
 */
void bsp_adc_pwr_power_off(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ADC_PWR_H */
