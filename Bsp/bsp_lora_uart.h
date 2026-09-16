/**
 * @file bsp_lora_uart.h
 * @author Fariscooool (fariscooool@gmail.com)
 * @brief  LoRa 数据串口(LPUART1, PA2=TX / PA3=RX)驱动接口
 * @version 0.1
 * @date 2026-09-05
 *
 * @note   驱动层:负责把一帧数据原样发送给 LoRa 模块,以及接收模块返回的数据。
 *         这里的“帧”只是字节流,具体协议(报文格式、校验)由应用层决定。
 *         - 发送:DMA 方式,但保持“阻塞”语义(发完才返回)。
 *         - 接收:DMA + 空闲(IDLE)检测(HAL_UARTEx_ReceiveToIdle_DMA),
 *           模块返回的一小段数据会在总线空闲后自动收进内部 FIFO。
 *         需要在 CubeMX 中给 LPUART1 使能 DMA(RX/TX)。
 */
#ifndef BSP_LORA_UART_H
#define BSP_LORA_UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * 初始化
 *============================================================================*/

/**
 * @brief 初始化 LoRa 串口:启动 LPUART1 DMA 接收(收满或空闲触发回调)
 * @note  需要在 MX_LPUART1_UART_Init() / MX_DMA_Init() 之后调用
 */
void lora_uart_init(void);

/*==============================================================================
 * 发送(DMA,阻塞语义)
 *============================================================================*/

/**
 * @brief 把一帧数据原样发送给 LoRa 模块(DMA;返回前已发送完成)
 * @param data  待发送数据
 * @param len   字节数
 */
void lora_send(const uint8_t *data, uint16_t len);

/*==============================================================================
 * 接收(DMA+空闲检测,数据被搬入内部 FIFO,接口保持简单)
 *============================================================================*/

/**
 * @brief 当前接收 FIFO 中可读字节数
 */
uint32_t lora_rx_available(void);

/**
 * @brief 从接收 FIFO 取一个字节
 * @retval 1=成功  0=空
 */
uint8_t lora_rx_get(uint8_t *byte);

/**
 * @brief 从接收 FIFO 取多字节
 * @retval 1=成功  0=失败(空或可读字节不足)
 */
uint8_t lora_rx_get_bytes(uint8_t *buf, uint16_t len);

/**
 * @brief DMA 收到一段完整数据后回调(由 bsp_dbg_uart 的 HAL 回调分发过来)
 * @note  仅中断上下文调用;内部会搬数据并重新启动接收
 */
void lora_uart_rx_event(uint16_t size);

/**
 * @brief 接收出错后重新启动接收(由 HAL_UART_ErrorCallback 分发过来)
 * @note  仅中断上下文调用
 */
void lora_uart_rx_restart(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_LORA_UART_H */

