#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

#include "stdint.h"
#include "stdlib.h"
#include "string.h"

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * Type Definitions
 *============================================================================*/

typedef struct {
    uint8_t *buffer;            /*!< pointer to the buffer memory */
    volatile uint8_t locked;    /*!< lock flag for thread-safe access (0 = unlocked, 1 = locked) */
    volatile uint32_t head;     /*!< index of the head (write position), volatile for ISR access */
    volatile uint32_t tail;     /*!< index of the tail (read position), volatile for ISR access */
    uint32_t max_size;          /*!< maximum size of the buffer (must be power of 2) */
} ring_buffer_t;

/*==============================================================================
 * Initialization
 *============================================================================*/

/**
 * @brief  Initialize a ring buffer with external memory.
 * @param  rb    Pointer to ring_buffer_t instance.
 * @param  buffer Pointer to externally allocated memory.
 * @param  size   Size of the buffer in bytes (must be power of 2 for best performance).
 */
void rb_init(ring_buffer_t *rb, uint8_t *buffer, uint32_t size);

/*==============================================================================
 * Write Operations
 *============================================================================*/

/**
 * @brief  Write a single byte into the ring buffer.
 * @param  rb   Pointer to ring_buffer_t instance.
 * @param  byte The byte to write.
 * @retval 1 on success, 0 if buffer is full.
 */
uint8_t rb_write(ring_buffer_t *rb, uint8_t byte);

/**
 * @brief  Write multiple bytes into the ring buffer.
 * @param  rb     Pointer to ring_buffer_t instance.
 * @param  data   Pointer to source data.
 * @param  length Number of bytes to write.
 * @retval Number of bytes actually written (may be less than length if buffer full).
 */
uint32_t rb_write_bulk(ring_buffer_t *rb, const uint8_t *data, uint32_t length);

/*==============================================================================
 * Read Operations
 *============================================================================*/

/**
 * @brief  Read a single byte from the ring buffer.
 * @param  rb   Pointer to ring_buffer_t instance.
 * @param  byte Pointer to store the read byte.
 * @retval 1 on success, 0 if buffer is empty.
 */
uint8_t rb_read(ring_buffer_t *rb, uint8_t *byte);

/**
 * @brief  Read multiple bytes from the ring buffer (without removing).
 * @param  rb     Pointer to ring_buffer_t instance.
 * @param  data   Pointer to destination buffer.
 * @param  length Number of bytes to peek.
 * @retval Number of bytes actually peeked.
 * @note   Data remains in the buffer after this operation.
 */
uint32_t rb_peek(const ring_buffer_t *rb, uint8_t *data, uint32_t length);

/**
 * @brief  Read multiple bytes from the ring buffer (and remove them).
 * @param  rb     Pointer to ring_buffer_t instance.
 * @param  data   Pointer to destination buffer.
 * @param  length Number of bytes to read.
 * @retval Number of bytes actually read.
 */
uint32_t rb_read_bulk(ring_buffer_t *rb, uint8_t *data, uint32_t length);

/**
 * @brief  Discard a number of bytes from the ring buffer without reading.
 * @param  rb     Pointer to ring_buffer_t instance.
 * @param  length Number of bytes to discard.
 * @retval Number of bytes actually discarded.
 */
uint32_t rb_discard(ring_buffer_t *rb, uint32_t length);

/*==============================================================================
 * Status / Query
 *============================================================================*/

/**
 * @brief  Get the number of bytes available for reading.
 */
uint32_t rb_available(const ring_buffer_t *rb);

/**
 * @brief  Get the number of bytes of free space.
 */
uint32_t rb_free_space(const ring_buffer_t *rb);

/**
 * @brief  Check if the buffer is empty.
 */
uint8_t rb_is_empty(const ring_buffer_t *rb);

/**
 * @brief  Check if the buffer is full.
 */
uint8_t rb_is_full(const ring_buffer_t *rb);

/**
 * @brief  Reset the ring buffer (clear all data).
 */
void rb_reset(ring_buffer_t *rb);

#ifdef __cplusplus
}
#endif

#endif /* __RING_BUFFER_H__ */
