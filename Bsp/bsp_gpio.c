/**
 * @file bsp_gpio.c
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief  外部信号检测 GPIO 驱动实现
 * @version 0.1
 * @date 2026-09-05
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp_gpio.h"

#include "main.h"

/*==============================================================================
 * 通道 <-> 引脚映射表
 *============================================================================*/

#define EXTI_NULL 0xFF

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint32_t      mode;
    uint32_t      pull;
    uint32_t      speed;
    uint32_t      alternate;
    uint32_t      exti_irq;    /* 该引脚所属的 EXTI 中断号 */
    uint32_t      pre_priority;
    uint32_t      sub_priority;
} bsp_sig_map_t;

/* 引脚映射表。
 * 注意:每条 EXTI 线号只能属于一个端口(SYSCFG_EXTICR),而 HAL_GPIO_Init()
 * 每初始化一个 EXTI 引脚就会重写 EXTICR —— 谁最后初始化谁抢到该线。
 * 所以 PA0(LoRa AUX) 绝不能配成 GPIO_MODE_IT_*,否则会抢走 PB0(CH0)的 EXTI0。 */
static const bsp_sig_map_t s_sig_map[BSP_SIG_CH_MAX] =
{
    [BSP_SIG_CH0] = { GPIOB, GPIO_PIN_0, GPIO_MODE_IT_RISING_FALLING, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW, 0, EXTI0_1_IRQn, 1, 0 },   /* CH0 */
    [BSP_SIG_CH1] = { GPIOB, GPIO_PIN_1, GPIO_MODE_IT_RISING_FALLING, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW, 0, EXTI0_1_IRQn, 1, 0 },   /* CH1 */
    [BSP_SIG_CH2] = { GPIOB, GPIO_PIN_3, GPIO_MODE_IT_RISING_FALLING, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW, 0, EXTI2_3_IRQn, 1, 0 },   /* CH2 */

    /* AUX(PA0):只做普通电平读取。绝不能配成 GPIO_MODE_IT_*,
     * 否则 HAL_GPIO_Init() 会把 SYSCFG_EXTICR1 的 EXTI0 重映射到 PA0,
     * 抢走 PB0(CH0)的唤醒能力(exti_irq 填 EXTI_NULL 并不能阻止这一点)。 */
    [BSP_LORA_AUX] = { GPIOA, GPIO_PIN_0, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0, EXTI_NULL, 0, 0 },
    [BSP_LORA_M0] = { GPIOA, GPIO_PIN_7, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0, EXTI_NULL, 0, 0 },
    [BSP_LORA_M1] = { GPIOA, GPIO_PIN_8, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0, EXTI_NULL, 0, 0 }
};

/* 记录低功耗唤醒时触发过的信号(bit0=CH0,bit1=CH1,bit2=CH2),中断里置位 */
static volatile uint32_t s_wake_events = 0U;

/* EXTI 回调表: 每个通道一个函数指针, 未注册为 NULL */
static bsp_gpio_exti_callback_t s_exti_callbacks[BSP_SIG_CH_MAX] = {0};

/*==============================================================================
 * 接口实现
 *============================================================================*/

void bsp_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    // 使能对应 GPIO 端口的时钟
    for(int i = 0; i < BSP_SIG_CH_MAX; i++)
    {
        if(s_sig_map[i].port == GPIOA)
        {
            /* 对 GPIOA 的引脚进行初始化 */
            __HAL_RCC_GPIOA_CLK_ENABLE();
        }
        if(s_sig_map[i].port == GPIOB)
        {
            /* 对 GPIOB 的引脚进行初始化 */
            __HAL_RCC_GPIOB_CLK_ENABLE();
        }
    }

    // GPIO 基础参数初始化
    for(int i = 0; i < BSP_SIG_CH_MAX; i++)
    {
        gpio.Pin   = s_sig_map[i].pin;
        gpio.Mode  = s_sig_map[i].mode;
        gpio.Pull  = s_sig_map[i].pull;
        gpio.Speed = s_sig_map[i].speed;
        gpio.Alternate = s_sig_map[i].alternate;
        HAL_GPIO_Init(s_sig_map[i].port, &gpio);
    }

    for(int i = 0; i < BSP_SIG_CH_MAX; i++)
    {
        if(s_sig_map[i].exti_irq != EXTI_NULL)
        {
            HAL_NVIC_SetPriority(s_sig_map[i].exti_irq, s_sig_map[i].pre_priority, s_sig_map[i].sub_priority);
            HAL_NVIC_EnableIRQ(s_sig_map[i].exti_irq);
        }
    }
}

bsp_gpio_level_t bsp_gpio_sig_level(bsp_sig_ch_t ch)
{
    if (ch >= BSP_SIG_CH_MAX)
    {
        return BSP_GPIO_HIGH;
    }
    return (HAL_GPIO_ReadPin(s_sig_map[ch].port, s_sig_map[ch].pin) == GPIO_PIN_RESET)
           ? BSP_GPIO_LOW : BSP_GPIO_HIGH;
}

void bsp_gpio_set_level(bsp_sig_ch_t ch, bsp_gpio_level_t level)
{
    if (ch >= BSP_SIG_CH_MAX)
    {
        return;
    }
    HAL_GPIO_WritePin(s_sig_map[ch].port, s_sig_map[ch].pin, (level == BSP_GPIO_LOW) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void bsp_gpio_register_exti_callback(bsp_sig_ch_t ch, bsp_gpio_exti_callback_t callback)
{
    if (ch >= BSP_SIG_CH_MAX)
    {
        return;
    }

    if(s_sig_map[ch].exti_irq != EXTI_NULL)
    {
        s_exti_callbacks[ch] = callback;
    }
}

uint32_t bsp_gpio_get_wake_events(void)
{
    return s_wake_events;
}

void bsp_gpio_clear_wake_events(void)
{
    s_wake_events = 0U;
}

/*==============================================================================
 * HAL 弱回调覆盖: 引脚 EXTI 中断触发时由 HAL_GPIO_EXTI_IRQHandler 调用
 *============================================================================*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t i;

    /* 按引脚号找映射表(PA0 已不再配 EXTI,所以 PIN_0 必定是 PB0,不存在歧义)。
     * 只有三路信号通道需要记录唤醒事件,AUX/M0/M1 不参与。 */
    for (i = 0U; i < BSP_SIG_CH_MAX; i++)
    {
        if ((s_sig_map[i].exti_irq == EXTI_NULL) || (s_sig_map[i].pin != GPIO_Pin))
        {
            continue;
        }

        if (i < BSP_SIG_CH_SIGNAL_MAX)
        {
            s_wake_events |= (1UL << i);
        }

        if (s_exti_callbacks[i] != NULL)
        {
            s_exti_callbacks[i]((bsp_sig_ch_t)i);
        }
        break;
    }
}
