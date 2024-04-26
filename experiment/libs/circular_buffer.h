#ifndef _CIRCULAR_BUFFER_H
#define _CIRCULAR_BUFFER_H

#include <stdint.h>

#define BUF_SIZE 16

typedef struct {
    volatile uint8_t head;
    volatile uint8_t tail;
    /* We create the buffer with 1 additional item to use it to flag
       the buffer full condition */
    uint16_t data[BUF_SIZE + 1];
} circular_buffer_t;

void cb_init(circular_buffer_t* buffer);
uint8_t is_cb_full(circular_buffer_t* buffer);
uint8_t is_cb_empty(circular_buffer_t* buffer);
uint8_t cb_occupancy(circular_buffer_t* buffer);
uint8_t cb_size(circular_buffer_t* buffer);

void cb_push(circular_buffer_t* buffer, uint16_t e);
uint16_t cb_pop(circular_buffer_t* buffer);

#define wrapped_inc(p) do {                          \
        if (++(p) > BUF_SIZE)                              \
            (p) = 0;                                 \
    } while (0)

#endif
