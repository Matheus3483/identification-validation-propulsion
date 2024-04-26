#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include "avr/io.h"

#define PORT_WIDTH 8

/*
 * Constantes para uso na configuração dos pinos.
 */
typedef enum {
    GPIO_IN_FLOATING,
    GPIO_IN_PULLUP,
    GPIO_OUT
} GPIO_mode;

/*
 * 
 */
typedef struct GPIO_struct GPIO_t;

/*
 * Ports definitions
 */
#define GPIOD2 ((GPIO_t *) &PINB)
#define GPIOD3 ((GPIO_t *) &PINC)
#define GPIOD4 ((GPIO_t *) &PIND)


/*
 * Functions to operate on the whole port
 */
uint8_t gpio_read_port(GPIO_t *port);
void gpio_write_port(GPIO_t *port, uint8_t value);
void gpio_clear_port(GPIO_t *port);
void gpio_set_port(GPIO_t *port);
void gpio_toggle_port(GPIO_t *port);
void gpio_set_port_mode(GPIO_t *port, GPIO_mode mode[]);

/*
 * Functions to operate on a group of bits
 */
uint8_t gpio_read_group(GPIO_t *port, uint8_t mask);
void gpio_write_group(GPIO_t *port, uint8_t mask, uint8_t value);
void gpio_clear_group(GPIO_t *port, uint8_t mask);
void gpio_set_group(GPIO_t *port, uint8_t mask);
void gpio_toggle_group(GPIO_t *port, uint8_t mask);
void gpio_set_group_mode(GPIO_t *port, uint8_t bitmask,
                         GPIO_mode mode[]);

/*
 * Functions to operate on a single bit
 */
uint8_t gpio_read_pin(GPIO_t *port, uint8_t pin_nbr);
void gpio_write_pin(GPIO_t *port, uint8_t pin_nbr, uint8_t value);
void gpio_set_pin(GPIO_t *port, uint8_t pin_nbr);
void gpio_clear_pin(GPIO_t *port, uint8_t pin_nbr);
void gpio_toggle_pin(GPIO_t *port, uint8_t pin_nbr);
void gpio_set_pin_mode(GPIO_t *port, uint8_t pin_nbr,
                       GPIO_mode mode);

#endif /* GPIO_H */