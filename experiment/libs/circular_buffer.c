#include "circular_buffer.h"

void cb_init(circular_buffer_t* buffer) {
    buffer->head = buffer->tail = 0;
}

uint8_t is_cb_full(circular_buffer_t* buf) {
    return buf->head == (buf->tail + 1 > BUF_SIZE ? 0 : buf->tail + 1);
}
 
uint8_t is_cb_empty(circular_buffer_t* buf) {
    return buf->head == buf->tail;
}

uint8_t cb_occupancy(circular_buffer_t* buf) {
    return buf->tail >= buf->head ? buf->tail - buf->head :
        buf->tail + (BUF_SIZE + 1 - buf->head);
}

uint8_t cb_size(circular_buffer_t* buf) {
    return BUF_SIZE;
}

void cb_push(circular_buffer_t* buf, uint16_t e) {
    if (!is_cb_full(buf)) {
        buf->data[buf->tail] = e;
        wrapped_inc(buf->tail);
    }
}

uint16_t cb_pop(circular_buffer_t* buf) {
    uint16_t e = -1;

    if (!is_cb_empty(buf)) {
        e = buf->data[buf->head];
        wrapped_inc(buf->head);
    }

    return e;
}

