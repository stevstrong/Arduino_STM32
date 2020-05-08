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
 * @file ring_buffer.c
 * @brief Simple circular buffer
 *
 * This implementation is not thread-safe.  In particular, none of
 * these functions is guaranteed re-entrant.
 */

#include "ring_buffer.h"


// returns the count of used slots
uint16_t rb_read_available(ring_buffer_t * rb)
{
	register rb_ptrs_t temp = { rb->ptrs.both };
	return (temp.head - temp.tail) & rb->CAPACITY;
}

// return the count of free slots
uint16_t rb_write_available(ring_buffer_t * rb)
{
	register rb_ptrs_t temp = { rb->ptrs.both };
	return ((temp.tail - temp.head - 1) & rb->CAPACITY); // overall free space
}

// Adds an element to the end of the queue
// does not check for free space. Use together with rb_write_available()
//---------------------------------------------------------------------------
void rb_write(ring_buffer_t * rb, const uint8_t element)
{
	register uint16_t temp_head = rb->ptrs.head;
	rb->buffer[temp_head++] = element;
	rb->ptrs.head = temp_head & rb->CAPACITY;
}

// Adds an element to the end of the queue
// It does check for free space. returns false if no space available
//---------------------------------------------------------------------------
int rb_write_safe(ring_buffer_t * rb, const uint8_t element)
{
	register rb_ptrs_t temp = { rb->ptrs.both };
	rb->buffer[temp.head++] = element; // safe to write because it does not overwrite active data
	temp.head &= rb->CAPACITY;
	if ( temp.head==temp.tail ) return false;
	rb->ptrs.head = temp.head;
	return true;
}

// Adds multiple elements to the end of the queue
// checks for available free space
//---------------------------------------------------------------------------
uint32 rb_write_safe_n(ring_buffer_t * rb, const uint8_t * buf, uint16_t len)
{
	register rb_ptrs_t temp = { rb->ptrs.both };
	register uint16 cap = rb->CAPACITY;
	uint32 txd = 0;
	while ( txd < len ) {
		rb->buffer[temp.head++] = *buf++; // safe to write because it does not overwrite active data
		temp.head &= cap;
		if (temp.head==temp.tail)
			break;
		++txd;
	}
	rb->ptrs.head = temp.head; // update volatile variable
	return txd;
}

// does not check for available free space !!
void rb_write_n(ring_buffer_t * rb, const uint8_t * buf, uint16_t len)
{
	register uint16_t temp_head = rb->ptrs.head;
	register uint16 cap = rb->CAPACITY;
	while (len--) {
		rb->buffer[temp_head++] = *buf++;
		temp_head &= cap;
	}
	rb->ptrs.head = temp_head;
}

// affects tail, reads head
int rb_read_safe(ring_buffer_t * rb)
{
	register rb_ptrs_t temp = { rb->ptrs.both };
	if ( (temp.head == temp.tail) )
		return -1;
	int elem = rb->buffer[temp.tail++];
	rb->ptrs.tail = temp.tail & rb->CAPACITY;
	return elem;
}

// does not check for available data !!
uint8_t rb_read(ring_buffer_t * rb)
{
	register uint16_t temp_tail = rb->ptrs.tail;
	uint8_t elem = rb->buffer[temp_tail++];
	rb->ptrs.tail = temp_tail & rb->CAPACITY;
	return elem;
}

// does not check for available data !!
uint32 rb_read_n(ring_buffer_t * rb, uint8_t* buf, uint16 len)
{
	uint32 rd = 0;
	register rb_ptrs_t temp = {rb->ptrs.both};
	register uint16 tmp_tail = temp.tail;
	register uint16 cap = rb->CAPACITY;
	while ( (tmp_tail!=temp.head) && ((rd++)<len) ) {
		*buf++ = rb->buffer[tmp_tail++];
		tmp_tail &= cap;
	}
	rb->ptrs.tail = tmp_tail; // update volatile variable
	return rd;
}

uint32 rb_peek_n(ring_buffer_t * rb, uint8_t* buf, uint16 len)
{
	uint32 peeked = 0;
	register rb_ptrs_t temp = {rb->ptrs.both};
	register uint16 tmp_tail = temp.tail;
	register uint16 cap = rb->CAPACITY;
	while ( (tmp_tail!=temp.head) && ((peeked++)<len) ) {
		*buf++ = rb->buffer[tmp_tail++];
		tmp_tail &= cap;
	}
	// no need to update volatile tail
	return peeked;
}

