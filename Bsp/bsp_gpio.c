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

static const bsp_sig_map_t s_sig_map[BSP_SIG_CH_MAX] =
{
    [BSP_SIG_CH0] = { GPIOB, GPIO_PIN_0, GPIO_MODE_IT_FALLING, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW, 0, EXTI0_1_IRQn, 1, 0 },   /* CH0 */
    [BSP_SIG_CH1] = { GPIOB, GPIO_PIN_1, GPIO_MODE_IT_FALLING, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW, 0, EXTI0_1_IRQn, 1, 0 },   /* CH1 */
    [BSP_SIG_CH2] = { GPIOB, GPIO_PIN_3, GPIO_MODE_IT_FALLING, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW, 0, EXTI2_3_IRQn, 1, 0 },   /* CH2 */

    [BSP_LORA_AUX] = { GPIOA, GPIO_PIN_0, GPIO_MODE_IT_FALLING, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW, 0, EXTI0_1_IRQn, 1, 0 },
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
    for(int i = 0; i < BSP_SIG_CH_MAX; i++)
    {
        if(s_sig_map[i].pin == GPIO_Pin)
        {
            
            break;
        }
    }
    if (GPIO_Pin == GPIO_PIN_0)
    {
        // PIN 0通道有GPIOA和GPIOB，需要区分。
        if(bsp_gpio_sig_level(BSP_SIG_CH0) == BSP_GPIO_LOW)
        {
            s_wake_events |= (1UL << BSP_SIG_CH0);
            if(s_exti_callbacks[BSP_SIG_CH0] != NULL)
            {
                s_exti_callbacks[BSP_SIG_CH0](BSP_SIG_CH0);
            }
        }
        else
        {
            if(s_exti_callbacks[BSP_LORA_AUX] != NULL)
            {
                s_exti_callbacks[BSP_LORA_AUX](BSP_LORA_AUX);
            }
        }
    }
    else if (GPIO_Pin == GPIO_PIN_1)
    {
        s_wake_events |= (1UL << BSP_SIG_CH1);
        if(s_exti_callbacks[BSP_SIG_CH1] != NULL)
        {
            s_exti_callbacks[BSP_SIG_CH1](BSP_SIG_CH1);
        }
    }
    else if (GPIO_Pin == GPIO_PIN_3)
    {
        s_wake_events |= (1UL << BSP_SIG_CH2);
        if(s_exti_callbacks[BSP_SIG_CH2] != NULL)
        {
            s_exti_callbacks[BSP_SIG_CH2](BSP_SIG_CH2);
        }
    }
}
