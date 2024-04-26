#ifndef UART_H
#define UART_H

#include <stdint.h>

#define _UART_BUFFER_SIZE 32

typedef enum{
   TX_RISING_RX_FALLING,
   TX_FALLING_RX_RISING
} uart_clock_polarity_t;

typedef enum {
   FIVE_DATA_BITS,
   SIX_DATA_BITS,
   SEVEN_DATA_BITS,
   EIGHT_DATA_BITS,
   NINE_DATA_BITS   
} uart_databits_t;

typedef enum {
   DISABLE_PARITY,
   EVEN_PARITY,
   ODD_PARITY
} uart_parity_t;

typedef enum {
   ONE_STOP_BIT,
   TWO_STOP_BITS
} uart_stopbits_t;

typedef enum {
   ASSYNCHRONOUS,
   SYNCHRONOUS,
   MASTER_SPI
} uart_synchronism_t;

typedef enum {
   SINGLE_CPU,
   MULTI_CPU
} uart_processors_t;

typedef enum {
   NORMAL_SPEED,
   DOUBLE_SPEED
} uart_speed_t;

typedef enum {
   RX_OFF,
   RX_ON
} rx_t;

typedef enum {
   TX_OFF,
   TX_ON
} tx_t;

/*
 * Definição dos tipos relacionados com o módulo serial
 */
/* Definição do tipo relacionado à estrutura que representa o módulo
   serial.  A definição da estrutura em si é feita depois */
typedef struct Uart_Struct Uart_t;

/* Tipo relativo à função de callback a ser usada na interrupção de
   recepção de dados.  Este tipo consiste de um ponteiro para uma
   função que aceita como parâmetro o ponteiro para um
   driver de serial. */
typedef void (*uart_rx_cb_t)(Uart_t*, uint8_t);

/* Tipo relativo à função de callback a ser usada na interrupção de
   transmissão de dados.  Este tipo consiste de um ponteiro para uma
   função que deve retornar ou um byte a ser transmitido ou o valor
   -1, que informará a quem a chamar para buscar o byte em outro lugar
   (no caso da interrupção, o buffer circular de dados a serem
   transmitidos). A função aceita como parâmetro o ponteiro para um
   driver de serial. */
typedef uint16_t (*uart_tx_cb_t)(Uart_t*);

/* Estrutura usada para configuração da porta serial.  Além dos
   parâmetros da comunicação serial, temos também possíveis funções de
   callback a ser usada nas interrupções. Se não for usar os
   callbacks, inicialize-os com o valor nulo (0) */
typedef struct {
    uint32_t freq_cpu;
    uint32_t baud_rate;
    uart_speed_t double_speed;
    uart_processors_t multi_processor;
    uart_synchronism_t sync_mode;
    uart_parity_t parity;
    uart_databits_t nbr_data_bits;
    uart_stopbits_t nbr_stop_bits;
    uart_clock_polarity_t clkpol;
    uart_rx_cb_t rx_cb;
    uart_tx_cb_t tx_cb;
} Uart_Config_t;

/*
 * Declaração das variáveis relativas a cada módulo de serial
 */
extern Uart_t* UARTD1;

/*
 * Declaração das funções que lidam com a serial
 */
void uart_init(Uart_t* drv);
void uart_start(Uart_t* drv, Uart_Config_t* cfg,
                uint8_t transmitter_on, uint8_t receiver_on);
void uart_stop(Uart_t* drv, uint8_t transmitter_off,
               uint8_t receiver_off);

void uart_writechar(Uart_t* drv, uint16_t ch);
uint8_t uart_write(Uart_t* drv, uint16_t* buf, uint8_t len);
uint16_t uart_read(Uart_t* drv);
uint8_t uart_read_bytes(Uart_t* drv, uint8_t* buf, uint8_t len);
uint8_t uart_available(Uart_t* drv);
uint8_t uart_available_for_write(Uart_t* drv);
 
#endif /* UART_H */