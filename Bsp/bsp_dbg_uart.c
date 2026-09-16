/**
 * @file bsp_dbg_uart.c
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief  调试串口(USART2)驱动实现(DMA 收发)
 * @version 0.1
 * @date 2026-09-05
 *
 * @note   - 发送用 DMA,但保持“阻塞”语义(等发完才返回)。
 *         - 接收用 DMA+空闲检测(HAL_UARTEx_ReceiveToIdle_DMA):
 *             DMA 为 Normal 模式,收满一缓冲或总线空闲即停止并回调;
 *             回调里把数据搬进 FIFO 后重新启动接收。
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp_dbg_uart.h"

#include <stdarg.h>
#include <stdio.h>

#include "main.h"
#include "usart.h"
#include "bsp_lora_uart.h"   /* 仅在 HAL 回调分发中使用 */

/*==============================================================================
 * 局部定义
 *============================================================================*/

#define DBG_RX_DMA_SIZE   64U    /* DMA 接收缓冲(单次最多缓存字节) */
#define DBG_RX_FIFO_SIZE  128U   /* 内部 FIFO(供应用层读取) */

/*==============================================================================
 * 局部变量
 *============================================================================*/

static uint8_t   s_rx_dma[DBG_RX_DMA_SIZE];      /* DMA 接收缓冲 */
static uint8_t   s_rx_fifo[DBG_RX_FIFO_SIZE];    /* 内部 FIFO 存储区 */
static volatile uint16_t s_rx_head = 0;          /* FIFO 写指针(中断写) */
static volatile uint16_t s_rx_tail = 0;          /* FIFO 读指针         */

static char s_printf_buf[160];                   /* 格式化缓冲区        */

/*==============================================================================
 * 内部函数
 *============================================================================*/

static void dbg_fifo_push(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_rx_head + 1U) % DBG_RX_FIFO_SIZE);
    if (next != s_rx_tail)           /* 未满才写入 */
    {
        s_rx_fifo[s_rx_head] = byte;
        s_rx_head = next;
    }
}

/* 启动一次 DMA“收到空闲/收满”接收 */
static void dbg_rx_start(void)
{
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_rx_dma, DBG_RX_DMA_SIZE);
}

/* 等待 DMA 发送真正结束(gState=READY 且 UART 移位寄存器发完 TC) */
static void dbg_wait_tx_done(UART_HandleTypeDef *huart)
{
    uint32_t t0 = HAL_GetTick();
    while ((huart->gState != HAL_UART_STATE_READY) ||
           (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET))
    {
        if ((HAL_GetTick() - t0) > 200U)
        {
            break;
        }
    }
}

/*==============================================================================
 * 对外接口
 *============================================================================*/

void dbg_uart_init(void)
{
    /* USART2 与 DMA 中断已由 CubeMX(MX_DMA_Init / HAL_UART_MspInit)使能 */
    dbg_rx_start();
}

void dbg_send(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U))
    {
        return;
    }

    /* 先清掉上一次遗留的 TC,再启动 DMA 发送 */
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_TCF);
    if (HAL_UART_Transmit_DMA(&huart2, (uint8_t *)data, len) != HAL_OK)
    {
        /* DMA 异常时回退为阻塞发送,保证能发出去 */
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)data, len, 100U);
        return;
    }
    dbg_wait_tx_done(&huart2);   /* 阻塞等待发完(进低功耗前总线空闲) */
}

void dbg_putchar(uint8_t ch)
{
    uint8_t tmp = ch;
    dbg_send(&tmp, 1U);
}

void dbg_printf(const char *fmt, ...)
{
    va_list ap;
    int     n;

    va_start(ap, fmt);
    n = vsnprintf(s_printf_buf, sizeof(s_printf_buf), fmt, ap);
    va_end(ap);

    if (n > 0)
    {
        uint16_t len = (n >= (int)sizeof(s_printf_buf)) ? (uint16_t)(sizeof(s_printf_buf) - 1U) : (uint16_t)n;
        dbg_send((const uint8_t *)s_printf_buf, len);
    }
}

uint8_t dbg_rx_get(uint8_t *byte)
{
    if (s_rx_tail == s_rx_head)
    {
        return 0;                    /* 空 */
    }
    *byte = s_rx_fifo[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % DBG_RX_FIFO_SIZE);
    return 1;
}

uint32_t dbg_rx_available(void)
{
    return (uint32_t)((s_rx_head - s_rx_tail) % DBG_RX_FIFO_SIZE);
}

/*==============================================================================
 * 中断上下文处理(被 HAL 回调调用)
 *============================================================================*/

/* 收到一段完整数据(空闲/收满)后:搬进 FIFO 并重新启动接收 */
static void dbg_uart_rx_event(uint16_t size)
{
    uint16_t i;

    if (size > DBG_RX_DMA_SIZE)
    {
        size = DBG_RX_DMA_SIZE;
    }
    for (i = 0U; i < size; i++)
    {
        dbg_fifo_push(s_rx_dma[i]);
    }
    dbg_rx_start();
}

/* 接收出错后重新启动接收 */
static void dbg_uart_rx_restart(void)
{
    dbg_rx_start();
}

/*==============================================================================
 * HAL 弱回调覆盖(所有 UART 共用,按实例分发)
 *============================================================================*/

/**
 * @brief UARTEx 接收事件回调(ReceiveToIdle_DMA 用)
 * @note  只有“接收已结束”的事件才处理:
 *        - HAL_UART_RXEVENT_IDLE : 总线空闲(短包/命令)
 *        - HAL_UART_RXEVENT_TC   : 收满整个 DMA 缓冲
 *        半满(HT)事件不结束接收,这里直接忽略,等空闲/收满再统一处理。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if ((huart->RxEventType != HAL_UART_RXEVENT_IDLE) &&
        (huart->RxEventType != HAL_UART_RXEVENT_TC))
    {
        return;
    }

    if (huart->Instance == USART2)
    {
        dbg_uart_rx_event(Size);
    }
    else if (huart->Instance == LPUART1)
    {
        lora_uart_rx_event(Size);
    }
}

/**
 * @brief UART 出错回调(如溢出),出错会中止接收,这里重启接收
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        dbg_uart_rx_restart();
    }
    else if (huart->Instance == LPUART1)
    {
        lora_uart_rx_restart();
    }
}



