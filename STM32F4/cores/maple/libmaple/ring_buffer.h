/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2011 LeafLabs, LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file ring_buffer.h
 * @brief Simple circular buffer
 *
 * This implementation is not thread-safe.  In particular, none of
 * these functions is guaranteed re-entrant.
 */

#ifndef _RING_BUFFER_H_
#define _RING_BUFFER_H_

#include "libmaple_types.h"

#ifdef __cplusplus
extern "C"{
#endif

/**
 * Ring buffer type.
 *
 * The buffer is empty when rd_index == wr_index.
 *
 * The buffer is full when the rd_index is one byte in front of the wr_index,
 * modulo buffer length.
 *
 * One byte is left free to distinguish empty from full. */
typedef struct ring_buffer_t {
    volatile uint8_t *buf; /**< Buffer items are stored into */
    volatile uint32_t size;      /**< Buffer capacity */
    volatile uint32_t rd_index;  /**< Index of the next item to remove */
    volatile uint32_t wr_index;  /**< Index where the next item will get inserted */
    volatile uint32_t available;  /**< available bytes to read out */
} ring_buffer_t;

/**
 * @brief Discard all items from a ring buffer.
 * @param rb Ring buffer to discard all items from.
 */
static inline void rb_reset(ring_buffer_t *rb)
{
    rb->rd_index = 0;
    rb->wr_index = 0;
    rb->available = 0;
}

/**
 * Initialise a ring buffer.
 *
 *  @param rb   Instance to initialise
 *
 *  @param size Number of items in buf.
 *
 *  @param buf  Buffer to store items into
 */
static inline void rb_init(ring_buffer_t *rb, uint32_t size, uint8_t *buf)
{
    rb->buf = buf;
    rb->size = size;
    rb_reset(rb);
}

/**
 * @brief Return the number of elements stored in the ring buffer.
 * @param rb Buffer whose elements to count.
 */
static inline uint32_t rb_rd_available(ring_buffer_t *rb) { return rb->available; }

/**
 * @brief Return the number of elements stored in the ring buffer.
 * @param rb Buffer whose elements to count.
 */
static inline uint32_t rb_wr_available(ring_buffer_t *rb)
{
    return (rb->size - rb->available);
}

/**
 * @brief Returns true if and only if the ring buffer is full.
 * @param rb Buffer to test.
 */
static inline bool rb_is_full(ring_buffer_t *rb)
{
    return !(rb->available < rb->size);
}

/**
 * @brief Returns true if and only if the ring buffer is empty.
 * @param rb Buffer to test.
 */
static inline bool rb_is_empty(ring_buffer_t *rb) { return (rb->available == 0); }

/**
 * Append element onto the end of a ring buffer.
 * User should assure that rb is not full before calling this function.
 * @param rb Buffer to append onto.
 * @param element Value to append.
 */
static inline void rb_write(ring_buffer_t *rb, uint8_t element)
{
    rb->buf[rb->wr_index] = element;
    if ((++ rb->wr_index) >= rb->size) rb->wr_index = 0;
    rb->available ++;
}

/**
 * @brief Remove and return the first item from a ring buffer.
 * @param rb Buffer to remove from, must contain at least one element.
 */
static inline uint8_t rb_read(ring_buffer_t *rb)
{
    uint8_t ch = rb->buf[rb->rd_index];
    if ((++ rb->rd_index) >= rb->size) rb->rd_index = 0;
    rb->available --;
    return ch;
}

/*
 * Roger Clark. 20141125, 
 * added peek function.
 * @brief Return the first item from a ring buffer, without removing it
 * @param rb Buffer to remove from, must contain at least one element.
 */
static inline uint8_t rb_peek(ring_buffer_t *rb) 
{
    return rb->buf[rb->rd_index];
}

/**
 * @brief Attempt to remove the first item from a ring buffer.
 *
 * If the ring buffer is nonempty, removes and returns its first item.
 * If it is empty, does nothing and returns a negative value.
 *
 * @param rb Buffer to attempt to remove from.
 */
static inline int32_t rb_safe_read(ring_buffer_t *rb)
{
    return (rb->available) ? rb_read(rb) : -1;
}

/**
 * @brief Attempt to insert an element into a ring buffer.
 *
 * @param rb Buffer to insert into.
 * @param element Value to insert into rb.
 * @sideeffect If rb is not full, appends element onto buffer.
 * @return If element was appended, then true; otherwise, false. */
static inline uint8_t rb_safe_write(ring_buffer_t *rb, uint8_t element)
{
    if (!(rb->available < rb->size)) return 0;
    rb_write(rb, element);
    return 1;
}

/**
 * @brief Append an item onto the end of a non-full ring buffer.
 *
 * If the buffer is full, removes its first item, then inserts the new
 * element at the end.
 *
 * @param rb Ring buffer to insert into.
 * @param element Value to insert into ring buffer.
 * @return On success, returns -1.  If an element was popped, returns
 *         the popped value.
 */
static inline int32_t rb_push_write(ring_buffer_t *rb, uint8_t element)
{
    int32_t ret = -1;
    if (!(rb->available < rb->size)) {
        ret = rb_read(rb); // buffer full, make space for new data
    }
    rb_write(rb, element);
    return ret;
}

#ifdef __cplusplus
} // extern "C"
#endif

#endif

