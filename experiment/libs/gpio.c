#include <stdint.h>
#include "gpio.h"


struct GPIO_struct {
    volatile uint8_t pin;
    volatile uint8_t ddr;
    volatile uint8_t port;
};


/*
 * Functions to operate on the whole port
 */
uint8_t gpio_read_port(GPIO_t *port) {
    return port->pin;
}

void gpio_write_port(GPIO_t *port, uint8_t value) {
    port->port = value;
}

void gpio_clear_port(GPIO_t *port) {
    port->port = 0;
}

void gpio_set_port(GPIO_t *port) {
    port->port = 0xFF;
}

void gpio_toggle_port(GPIO_t *port) {
    port->pin = 0xFF;
}

void gpio_set_port_mode(GPIO_t *port, GPIO_mode mode[]) {
    gpio_set_group_mode(port, 0xFF, mode);
}


/*
 * Functions to operate on a group of bits
 */
uint8_t gpio_read_group(GPIO_t *port, uint8_t mask) {
    return port->port &= mask;
}

void gpio_write_group(GPIO_t *port, uint8_t mask, uint8_t value) {
    port->port &= ~mask;
    port->port |= (value & mask);
}

void gpio_clear_group(GPIO_t *port, uint8_t mask) {
    port->port &= ~mask;
}

void gpio_set_group(GPIO_t *port, uint8_t mask) {
    port->port |= mask;
}

void gpio_toggle_group(GPIO_t *port, uint8_t mask) {
    port->pin |= mask;
}

void gpio_set_group_mode(GPIO_t *port, uint8_t bitmask,
                         GPIO_mode mode[]) {
    uint8_t ddr_value = 0;
    uint8_t port_value = 0;
    int8_t i;

    for (i=PORT_WIDTH-1; i>=0; i--) {
        ddr_value <<= 1;
        port_value <<= 1;
        if ((bitmask & (1 << i)) == 0)
            continue;
        if (mode[i] == GPIO_OUT)
            ddr_value |= 1;
        else if (mode[i] == GPIO_IN_PULLUP)
            port_value |= 1;
    }

    port->ddr = ddr_value;
    port->port = port_value;
}


/*
 * Functions to operate on a single bit
 */
uint8_t gpio_read_pin(GPIO_t *port, uint8_t pin_nbr) {
    return port->port & (1 << pin_nbr);
}

void gpio_write_pin(GPIO_t *port, uint8_t pin_nbr, uint8_t value) {
    port->port &= ~(1 << pin_nbr);
    port->port |= ((value & 1) << pin_nbr);
}

void gpio_set_pin(GPIO_t *port, uint8_t pin_nbr) {
    port->port |= (1 << pin_nbr);
}

void gpio_clear_pin(GPIO_t *port, uint8_t pin_nbr) {
    port->port &= ~(1 << pin_nbr);
}

void gpio_toggle_pin(GPIO_t *port, uint8_t pin_nbr) {
    port->pin |= (1 << pin_nbr);
}

void gpio_set_pin_mode(GPIO_t *port, uint8_t pin_nbr,
                       GPIO_mode mode) {
    if (mode == GPIO_OUT)
        port->ddr |= (1 << pin_nbr);
    else {
        port->ddr &= ~(1 << pin_nbr);
        if (mode == GPIO_IN_PULLUP)
            port->port |= (1 << pin_nbr);
    }
}
