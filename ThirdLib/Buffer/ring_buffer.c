/**
 * @file ring_buffer.c
 * @brief  Generic ring buffer implementation.
 * @author fariscooool (fariscooool@gmail.com)
 * @version
 * @date 2026-05-25
 *
 * @copyright Copyright (c) 2026  fariscooool
 *
**/

#include "ring_buffer.h"

/*==============================================================================
 * Internal Helpers
 *============================================================================*/

/**
 * @brief  Advance an index with wrap-around.
 */
static inline uint32_t rb_advance(const ring_buffer_t *rb, uint32_t index, uint32_t n)
{
    return (index + n) & (rb->max_size - 1U);
}

/*==============================================================================
 * Initialization
 *============================================================================*/

void rb_init(ring_buffer_t *rb, uint8_t *buffer, uint32_t size)
{
    rb->buffer = buffer;
    rb->max_size = size;
    rb->head = 0;
    rb->tail = 0;
}

/*==============================================================================
 * Write Operations
 *============================================================================*/

uint8_t rb_write(ring_buffer_t *rb, uint8_t byte)
{
    if (rb_is_full(rb)) {
        return 0;
    }
    rb->buffer[rb->head] = byte;
    rb->head = rb_advance(rb, rb->head, 1U);
    return 1;
}

uint32_t rb_write_bulk(ring_buffer_t *rb, const uint8_t *data, uint32_t length)
{
    uint32_t i;
    uint32_t written = 0;
    rb->locked = 1;  /* Lock the buffer for thread-safe access */
    for (i = 0; i < length; i++) {
        if (rb_write(rb, data[i]) == 0) {
            break;
        }
        written++;
    }
    rb->locked = 0;  /* Unlock the buffer */
    return written;
}

/*==============================================================================
 * Read Operations
 *============================================================================*/

uint8_t rb_read(ring_buffer_t *rb, uint8_t *byte)
{
    if(rb->locked) {
        return 0;  /* Buffer is locked, cannot read */
    }
    if (rb_is_empty(rb)) {
        return 0;
    }
    *byte = rb->buffer[rb->tail];
    rb->tail = rb_advance(rb, rb->tail, 1U);
    return 1;
}

uint32_t rb_peek(const ring_buffer_t *rb, uint8_t *data, uint32_t length)
{
    uint32_t avail;
    uint32_t i;
    uint32_t index;

    avail = rb_available(rb);
    if (length > avail) {
        length = avail;
    }

    index = rb->tail;
    for (i = 0; i < length; i++) {
        data[i] = rb->buffer[index];
        index = rb_advance(rb, index, 1U);
    }

    return length;
}

uint32_t rb_read_bulk(ring_buffer_t *rb, uint8_t *data, uint32_t length)
{
    uint32_t avail;
    uint32_t i;

    if(rb->locked) {
        return 0;  /* Buffer is locked, cannot read */
    }

    avail = rb_available(rb);
    if (length > avail) {
        length = avail;
    }

    for (i = 0; i < length; i++) {
        data[i] = rb->buffer[rb->tail];
        rb->tail = rb_advance(rb, rb->tail, 1U);
    }

    return length;
}

uint32_t rb_discard(ring_buffer_t *rb, uint32_t length)
{
    uint32_t avail;

    if(rb->locked) {
        return 0;  /* Buffer is locked, cannot discard */
    }

    avail = rb_available(rb);
    if (length > avail) {
        length = avail;
    }

    rb->tail = rb_advance(rb, rb->tail, length);
    return length;
}

/*==============================================================================
 * Status / Query
 *============================================================================*/

uint32_t rb_available(const ring_buffer_t *rb)
{
    return (rb->head - rb->tail) & (rb->max_size - 1U);
}

uint32_t rb_free_space(const ring_buffer_t *rb)
{
    return rb->max_size - rb_available(rb) - 1U;
}

uint8_t rb_is_empty(const ring_buffer_t *rb)
{
    return (rb->head == rb->tail) ? 1U : 0U;
}

uint8_t rb_is_full(const ring_buffer_t *rb)
{
    return (rb_advance(rb, rb->head, 1U) == rb->tail) ? 1U : 0U;
}

void rb_reset(ring_buffer_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
}




