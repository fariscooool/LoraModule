/**
 * @file bsp_dbg_uart.h
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief  调试串口(USART2, PA9=TX / PA10=RX, 115200-8N1)驱动接口
 * @version 0.1
 * @date 2026-09-05
 *
 * @note   驱动层:负责调试串口的收发,不包含任何业务逻辑。
 *         - 发送:DMA 方式,但保持“阻塞”语义(发完才返回),进低功耗前总线已空闲。
 *         - 接收:DMA + 空闲(IDLE)检测(HAL_UARTEx_ReceiveToIdle_DMA),
 *           收满一个缓冲区或总线空闲时把数据搬进内部 FIFO,对外接口不变。
 *         需要在 CubeMX 中给 USART2 使能 DMA(RX/TX)。
 */
#ifndef BSP_DBG_UART_H
#define BSP_DBG_UART_H

#include <stdint.h>

/* 调试打印总开关:
 *   1 = 开启(默认,开发调试阶段)
 *   0 = 屏蔽所有调试打印(正式发布固件)
 * 说明:本头文件给出默认值;若想在 app_config.h 里统一管理,
 *       请确保在包含本头文件之前先 #include "app_config.h"
 *       (app_config.h 中已 #define APP_DEBUG_ENABLE)。 */
#ifndef APP_DEBUG_ENABLE
#define APP_DEBUG_ENABLE 1
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * 初始化
 *============================================================================*/

/**
 * @brief 初始化调试串口:启动 USART2 DMA 接收(收满或空闲触发回调)
 * @note  需要在 MX_USART2_UART_Init() / MX_DMA_Init() 之后调用
 */
void dbg_uart_init(void);

/*==============================================================================
 * 发送(DMA,阻塞语义)
 *============================================================================*/

#if (APP_DEBUG_ENABLE == 1)

/**
 * @brief 发送一帧数据(DMA;返回前已发送完成,保证进低功耗前总线空闲)
 */
void dbg_send(const uint8_t *data, uint16_t len);

/**
 * @brief 发送一个字节(DMA)
 */
void dbg_putchar(uint8_t ch);

/**
 * @brief 格式化打印(内部使用 vsnprintf + dbg_send)
 * @note  用于把运行信息打印到调试串口
 */
void dbg_printf(const char *fmt, ...);

#else  /* 发布版:调试打印编译为空,连参数都不求值,不占代码 */

#define dbg_send(...)    ((void)0)
#define dbg_putchar(...) ((void)0)
#define dbg_printf(...)  ((void)0)

#endif /* APP_DEBUG_ENABLE */

/*==============================================================================
 * 接收(DMA+空闲检测,数据被搬入内部 FIFO,接口保持简单)
 *============================================================================*/

/**
 * @brief 从接收 FIFO 取一个字节
 * @retval 1=成功  0=空
 */
uint8_t dbg_rx_get(uint8_t *byte);

/**
 * @brief 当前接收 FIFO 中可读字节数
 */
uint32_t dbg_rx_available(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_DBG_UART_H */
