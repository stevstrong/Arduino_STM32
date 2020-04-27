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
 * @file libmaple/include/libmaple/ring_buffer.h
 * @brief Simple circular buffer
 *
 * This implementation is not thread-safe.  In particular, none of
 * these functions is guaranteed re-entrant.
 */

#ifndef _LIBMAPLE_RING_BUFFER_H_
#define _LIBMAPLE_RING_BUFFER_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "libmaple_types.h"


/**
   A union containing 16 bit head and tail offsets into the ring buffer.
   The union allows the c code to grab both values with one assembler instruction access.
*/
typedef union rb_ptrs_t {
	// access as 32 bit
	uint32_t both;
	// -- or as 2 16 bit values --
	struct {
		uint16_t head;
		uint16_t tail;
	};
} rb_ptrs_t;

typedef struct ring_buffer_t {
	volatile rb_ptrs_t ptrs;
	const int CAPACITY; // should be set to SIZE-1
	uint8_t buffer[];
} ring_buffer_t;

#define RING_BUFFER(name, size) ring_buffer_t (name) = { \
									.ptrs.both = 0, \
									.CAPACITY = (size-1), \
									.buffer[size] = 0 }

//	enum { CAPACITY = SIZE - 1 }; // leave one slot open

// --- public methods ---

// write access zeros out head and tail
inline void rb_reset(ring_buffer_t * rb) { rb->ptrs.both = 0; }

// return the size of the buffer
inline int rb_size(ring_buffer_t * rb) { return rb->CAPACITY; }

uint16_t rb_read_available(ring_buffer_t * rb);  // return the count of used slots

uint16_t rb_write_available(ring_buffer_t * rb);  // return the count of free slots

// returns true when there is no used slots
inline bool rb_is_empty(ring_buffer_t * rb) {
	register rb_ptrs_t temp = { rb->ptrs.both };
	return (temp.head == temp.tail);
}

// returns true when all slots are used
inline int rb_is_full(ring_buffer_t * rb) {
	return (rb_read_available(rb) == (uint16)rb->CAPACITY);
}

//  write() - adds an element to the end of the queue
//  Note: affects head, reads tail, element ignored if overflow ~300 ns @72MHz
//---------------------------------------------------------------------------
int rb_write_safe(ring_buffer_t * rb, const uint8_t element);
uint32 rb_write_safe_n(ring_buffer_t * rb, const uint8_t * buf, uint16_t len);

// does not check for available free space !!
void rb_write_n(ring_buffer_t * rb, const uint8_t * buf, uint16_t len);
/*
static inline uint8_t * rb_write_ptr(ring_buffer_t * rb) {
	return &rb->buffer[rb->ptrs.head];
}
*/
inline void rb_write_finish(ring_buffer_t * rb, uint16_t n) {
	register uint16_t temp_head = (rb->ptrs.head + n) & rb->CAPACITY;
	rb->ptrs.head = temp_head;
}
/*
inline uint8_t * rb_read_ptr(ring_buffer_t * rb) {
	return &rb->buffer[rb->ptrs.tail];
}
*/
inline int rb_peek(ring_buffer_t * rb) {
	return rb->buffer[rb->ptrs.tail];
}

uint32 rb_peek_n(ring_buffer_t * rb, uint8_t* buf, uint16 len);
uint32 rb_read_n(ring_buffer_t * rb, uint8_t* buf, uint16 len);

int rb_read_safe(ring_buffer_t * rb);
uint8_t rb_read(ring_buffer_t * rb);



#ifdef __cplusplus
} // extern "C"
#endif

#endif
