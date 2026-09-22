/**
 * @file bsp_adc_pwr.c
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief  电池电压检测(ADC1_IN6 / PA6)驱动实现
 * @version 0.1
 * @date 2026-09-23
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp_adc_pwr.h"

#include "adc.h"    /* hadc / MX_ADC_Init() */

/*==============================================================================
 * 内部常量
 *============================================================================*/

/* 电池分压点接入的 ADC 通道(与 CubeMX 中 PA6 = ADC_IN6 对应) */
#define BSP_ADC_PWR_CH_VBAT             ADC_CHANNEL_6

/* 采样时间。
 * 注意:STM32L0 的采样时间是"所有通道共用一个 SMPR",只能在 HAL_ADC_Init()
 * 里设置,而 CubeMX 生成的是 1.5 周期 —— 对高阻分压点和内部 VREFINT 都太短,
 * 所以本驱动在每次打开 ADC 时用下面的值重新 HAL_ADC_Init()(不动生成的 adc.c)。
 * 依据:VREFINT 要求采样时间 >= 4us;高阻分压点要求 (R1//R2)*C_adc*ln(2^13) 量级。
 * 当前 ADC 时钟约 12MHz(PCLK 24MHz / 同步 2 分频),160.5 周期 ≈ 14.4us,足够。
 * 若修改了系统时钟/ADC 分频,需要重新核算这里。 */
#ifndef BSP_ADC_PWR_SAMPLING_TIME
#define BSP_ADC_PWR_SAMPLING_TIME       ADC_SAMPLETIME_160CYCLES_5
#endif

/* VREFINT 出厂校准值:VREF+ = 3.0V 时测得的 VREFINT 原始码
 * (地址与条件见 stm32l0xx_ll_adc.h 的 VREFINT_CAL_ADDR / VREFINT_CAL_VREF) */
#define BSP_ADC_PWR_VREFINT_CAL_ADDR    ((const uint16_t *)0x1FF80078UL)
#define BSP_ADC_PWR_VREFINT_CAL_VREF_MV (3000U)

/* ADC 满量程(12 位)对应的原始码 */
#define BSP_ADC_PWR_FS                  4095U

/*==============================================================================
 * 内部函数
 *============================================================================*/

/**
 * @brief 打开 ADC:重新初始化(含采样时间) + 校准;此时 ADC 仍处于关闭态
 */
static bsp_adc_pwr_status_t adc_open(void)
{
    /* 采样时间只能在 HAL_ADC_Init() 里配置,这里覆盖 CubeMX 默认的 1.5 周期 */
    hadc.Init.SamplingTime = BSP_ADC_PWR_SAMPLING_TIME;

    /* 内部会调用 MspInit:打开 ADC 时钟、把 PA6 配成模拟输入 */
    if (HAL_ADC_Init(&hadc) != HAL_OK)
    {
        return BSP_ADC_PWR_ERR_INIT;
    }

    /* 校准必须在 ADC 关闭(ADEN = 0)时做,可提高转换精度 */
    if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK)
    {
        (void)HAL_ADC_DeInit(&hadc);
        return BSP_ADC_PWR_ERR_INIT;
    }

    return BSP_ADC_PWR_OK;
}

/**
 * @brief 关闭 ADC:关调压器/时钟/VREFINT 缓冲,PA6 保持模拟输入(高阻)
 */
static void adc_close(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* 会调用 MspDeInit:关 ADC 时钟、反初始化 PA6 */
    (void)HAL_ADC_DeInit(&hadc);

    /* MspDeInit 里 PA6 被反初始化,这里显式恢复为模拟输入:
     * 模拟态才是真正高阻,分压电阻上没有额外漏电流,也不影响 Stop 电流 */
    gpio.Pin  = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
}

/**
 * @brief 选择要转换的通道(CHSELR 是按位或写入,必须先清掉用过的通道)
 */
static bsp_adc_pwr_status_t adc_select_channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef cfg = {0};

    /* 1) 关掉可能还使能着的通道:
     *    - 否则扫描模式下序列里会有多个通道,读数会错位;
     *    - 关 VREFINT 通道同时会关掉它的内部缓冲,省电 */
    cfg.Rank = ADC_RANK_NONE;
    cfg.Channel = BSP_ADC_PWR_CH_VBAT;
    (void)HAL_ADC_ConfigChannel(&hadc, &cfg);
    cfg.Channel = ADC_CHANNEL_VREFINT;
    (void)HAL_ADC_ConfigChannel(&hadc, &cfg);

    /* 2) 打开目标通道 */
    cfg.Rank = ADC_RANK_CHANNEL_NUMBER;
    cfg.Channel = channel;
    if (HAL_ADC_ConfigChannel(&hadc, &cfg) != HAL_OK)
    {
        return BSP_ADC_PWR_ERR_INIT;
    }

    if (channel == ADC_CHANNEL_VREFINT)
    {
        /* VREFINT 内部缓冲刚打开时需要建立时间,等它就绪(超时则按"凑合用"继续) */
        uint32_t tickstart = HAL_GetTick();

        while ((SYSCFG->CFGR3 & SYSCFG_VREFINT_ADC_RDYF) == 0U)
        {
            if ((HAL_GetTick() - tickstart) > BSP_ADC_PWR_VREFINT_READY_MS)
            {
                break;
            }
        }
    }

    return BSP_ADC_PWR_OK;
}

/**
 * @brief 对已选中的通道连续转换 samples 次,返回平均值
 * @param [out] out_avg 平均后的原始码
 */
static bsp_adc_pwr_status_t adc_convert_avg(uint8_t samples, uint32_t *out_avg)
{
    uint32_t sum = 0U;
    uint8_t  i;

    for (i = 0U; i < samples; i++)
    {
        /* 软件触发 + 单次转换模式:每采一次都要重新 Start
         * (第一次 Start 会自动使能 ADC;后续 ADEN 已置位,直接再启动一次转换) */
        if (HAL_ADC_Start(&hadc) != HAL_OK)
        {
            return BSP_ADC_PWR_ERR_INIT;
        }
        if (HAL_ADC_PollForConversion(&hadc, BSP_ADC_PWR_TIMEOUT_MS) != HAL_OK)
        {
            (void)HAL_ADC_Stop(&hadc);
            return BSP_ADC_PWR_ERR_TIMEOUT;
        }
        sum += HAL_ADC_GetValue(&hadc);
    }

    (void)HAL_ADC_Stop(&hadc);      /* 关 ADC(ADEN = 0) */

    *out_avg = sum / (uint32_t)samples;
    return BSP_ADC_PWR_OK;
}

/**
 * @brief 一次完整测量:开 ADC -> 选通道 -> 平均 -> 关 ADC(失败也会关)
 */
static bsp_adc_pwr_status_t adc_measure(uint32_t channel, uint32_t *out_avg)
{
    bsp_adc_pwr_status_t st;

    st = adc_open();
    if (st != BSP_ADC_PWR_OK)
    {
        return st;
    }

    st = adc_select_channel(channel);
    if (st == BSP_ADC_PWR_OK)
    {
        st = adc_convert_avg(BSP_ADC_PWR_SAMPLES, out_avg);
    }

    adc_close();
    return st;
}

/**
 * @brief 取当前 ADC 参考电压(= VDDA),单位 mV
 * @note  BSP_ADC_PWR_USE_VREFINT = 1 时用内部 VREFINT 实测;否则用标称值兜底
 */
static void adc_get_vref_mv(uint32_t *vref_mv)
{
#if (BSP_ADC_PWR_USE_VREFINT == 1)
    uint32_t vrefint_raw = 0U;
    uint16_t cal = *BSP_ADC_PWR_VREFINT_CAL_ADDR;

    if ((cal != 0U) && (cal != 0xFFFFU) &&
        (adc_measure(ADC_CHANNEL_VREFINT, &vrefint_raw) == BSP_ADC_PWR_OK) &&
        (vrefint_raw != 0U))
    {
        /* VDDA = 校准条件(3.0V) * 校准码 / 实测码 */
        *vref_mv = (BSP_ADC_PWR_VREFINT_CAL_VREF_MV * (uint32_t)cal) / vrefint_raw;
        return;
    }
#endif

    *vref_mv = BSP_ADC_PWR_VREF_MV;     /* 兜底:标称值 */
}

/*==============================================================================
 * 接口实现
 *============================================================================*/

bsp_adc_pwr_status_t bsp_adc_pwr_init(void)
{
    bsp_adc_pwr_status_t st = adc_open();

    adc_close();    /* 初始化/校准完立即关掉,真正测量时再临时打开 */

    return st;
}

bsp_adc_pwr_status_t bsp_adc_pwr_read_raw(uint16_t *raw_avg)
{
    uint32_t             avg = 0U;
    bsp_adc_pwr_status_t st;

    if (raw_avg == NULL)
    {
        return BSP_ADC_PWR_ERR_PARAM;
    }

    st = adc_measure(BSP_ADC_PWR_CH_VBAT, &avg);
    if (st == BSP_ADC_PWR_OK)
    {
        *raw_avg = (uint16_t)avg;
    }

    return st;
}

bsp_adc_pwr_status_t bsp_adc_pwr_read_mv(uint32_t *bat_mv)
{
    uint32_t             raw = 0U;
    uint32_t             vref_mv = 0U;
    uint32_t             vadc_mv;
    bsp_adc_pwr_status_t st;

    if (bat_mv == NULL)
    {
        return BSP_ADC_PWR_ERR_PARAM;
    }

    /* 1) 先量参考电压(必要时),再量分压点 */
    adc_get_vref_mv(&vref_mv);

    st = adc_measure(BSP_ADC_PWR_CH_VBAT, &raw);
    if (st != BSP_ADC_PWR_OK)
    {
        return st;
    }

    /* 2) 分压点电压 = 参考电压 * 原始码 / 满量程 */
    vadc_mv = (vref_mv * raw) / BSP_ADC_PWR_FS;

    /* 3) 按分压比还原成电池电压 */
    *bat_mv = (vadc_mv * BSP_ADC_PWR_DIV_NUM) / BSP_ADC_PWR_DIV_DEN;

    return BSP_ADC_PWR_OK;
}

bsp_adc_pwr_status_t bsp_adc_pwr_read_vdd_mv(uint32_t *vdd_mv)
{
    if (vdd_mv == NULL)
    {
        return BSP_ADC_PWR_ERR_PARAM;
    }

    adc_get_vref_mv(vdd_mv);

    return BSP_ADC_PWR_OK;
}

uint8_t bsp_adc_pwr_percent(uint32_t bat_mv)
{
    if (bat_mv <= BSP_ADC_PWR_BAT_EMPTY_MV)
    {
        return 0U;
    }
    if (bat_mv >= BSP_ADC_PWR_BAT_FULL_MV)
    {
        return 100U;
    }

    return (uint8_t)(((bat_mv - BSP_ADC_PWR_BAT_EMPTY_MV) * 100UL) /
                     (BSP_ADC_PWR_BAT_FULL_MV - BSP_ADC_PWR_BAT_EMPTY_MV));
}

uint8_t bsp_adc_pwr_is_low(uint32_t bat_mv)
{
    return (bat_mv < BSP_ADC_PWR_LOW_MV) ? 1U : 0U;
}

void bsp_adc_pwr_power_off(void)
{
    adc_close();
}
