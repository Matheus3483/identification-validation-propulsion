#ifndef __AVR_ATmega328P__
    #define __AVR_ATmega328P__
#endif

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include <math.h>
#include "../libs/timer.h"
#include "../libs/gpio.h"
#include "../libs/uart.h"
#include "../libs/nsines.h"   // sinal de entrada

/*!
 * Escolha de entrada. Please uncoment the PLATFORM define and choose your platform
 * or add/change the PLATFORM definition on the compiler Defines option
 */
// ENTRADA
//#define DEGRAU
#define NSENOS

// tempo de amostra
//#define T10MS
//#define T100MS

// N
#define N10
//#define N11

// Definições
#define UART_BAUD_RATE 1000000 // 1 byte => 10us
#define F_CPU 16000000
#define TX_DATA_SIZE 4//5
#define PWM_MAX 249
#define PWM_MIN 125
#define PWM_MID 187
#ifdef N10
#define PWM_T0  187
#define N 10.0
#endif
#ifdef N11
#define PWM_T0  217
#define N 11.0
#endif

#ifdef DEGRAU
#define TOTAL_TIME  TOTAL_TIME_15S
#define T10MS
#endif
#ifdef NSENOS
#define TOTAL_TIME  TOTAL_TIME_120S 
#define T100MS
#endif

#ifdef T10MS
#define SAMPLE_TIME 5// x2ms = 10.24ms
#define TOTAL_TIME_15S   1500  + 10
#define TOTAL_TIME_30S   3000  + 10
#define TOTAL_TIME_60S   6000  + 10
#define TOTAL_TIME_120S  12000 + 10
#endif

#ifdef T100MS
#define SAMPLE_TIME 50// x2ms = 102.4ms
#define TOTAL_TIME_15S   150  + 10
#define TOTAL_TIME_30S   300  + 10
#define TOTAL_TIME_60S   600  + 10
#define TOTAL_TIME_120S  1200 + 10
#endif


// variaveis universais
uint8_t old_value = 0;
volatile uint32_t tempo = 0;
uint16_t tx_data[TX_DATA_SIZE];
uint32_t value_tx = 0;
uint8_t entrada = 0;
volatile uint32_t delta_tempo = 0;
volatile uint32_t old_tempo;


/* 
 * Funções de callback para interrupções
 */

// callback do temporizador do tempo e pwm
void callback_T0(GPT_t* drv) { // 10 microssegundos
    tempo++;
}
void cb_shutdown(GPT_t* drv){ // ciclo de 2.06 milissegundos
    static uint8_t cnt = 0;

    cnt++;
    if (cnt == 50) {
        if (entrada > 125) {
            entrada = entrada-5;
            gpt_enable_pwm_channel(GPTD3, 0, entrada);
        } else {
            gpio_clear_pin(GPIOD4, 5);
            gpt_stop_notification(GPTD3);
            gpt_disable_pwm_channel(GPTD3, 0);
            gpt_stop(GPTD3);
            gpt_stop_channel_notification(GPTD1,0);
            gpt_stop(GPTD1);
        }
    }
}

// callback do temporizador do tick e tempo de amostragem
void cb_update(GPT_t* drv){ // ciclo de 2.06 milissegundos
    static uint8_t cnt = 0;
    static uint16_t cnt2 = 0;

    cnt++;
    if (cnt == SAMPLE_TIME) {
        value_tx = delta_tempo;

        #ifdef NSENOS 
        entrada = NSines(tempo, N);                // segundo teste entrada variável
        gpt_enable_pwm_channel(GPTD3, 0, entrada); 
        #endif      

        tx_data[0] =  value_tx & 0xFF;
        tx_data[1] = (value_tx >> 8) & 0xFF;
        tx_data[2] = (value_tx >> 16) & 0xFF;
        tx_data[3] = entrada;
        uart_write(UARTD1, tx_data, TX_DATA_SIZE);
        cnt = 0;
        cnt2++;
        if (cnt2 == TOTAL_TIME){
            gpt_start_notification(GPTD3, cb_shutdown, 0);
        }
    }
}

void cb_calibration(GPT_t* drv){ // ciclo de 2.06 milissegundo
    static uint16_t cnt = 0;
    static uint8_t fase = 0;
    static GPT_Config config_T0 = {MODE_CTC, DIVISOR_1, 159};

    cnt++;
    switch (fase)
    {
    case 0:
        if (cnt == 2500) { // 5s
            gpt_enable_pwm_channel(GPTD3, 0, PWM_MAX);
            cnt = 0;
            fase = 1;
        }
        break;
    case 1:
        if (cnt == 5000) { // 10s
            gpt_enable_pwm_channel(GPTD3, 0, PWM_MIN);
            cnt = 0;
            fase = 2;
        }
        break;  
    case 2:
        if (cnt == 5000) { // 10s
            gpt_start_notification(GPTD3, cb_update, 0);
            gpt_start(GPTD1, &config_T0); // chama o temporizador para contagem de tempo
            gpt_start_channel_notification(GPTD1, 0, 0, callback_T0, 0); // habilita interrupção na comparação do temporizador

            #ifdef DEGRAU
            entrada = PWM_MAX;  // 0=125 - 100=249
            #endif
            #ifdef NSENOS
            entrada = PWM_T0;  // 0=125 - 100=249
            #endif

            gpt_enable_pwm_channel(GPTD3, 0, entrada);

            value_tx = delta_tempo;
            tx_data[0] =  value_tx & 0xFF;
            tx_data[1] = (value_tx >> 8) & 0xFF;
            tx_data[2] = (value_tx >> 16) & 0xFF;
            tx_data[3] = entrada;
            uart_write(UARTD1, tx_data, TX_DATA_SIZE);
            PCICR |= (1 << PCIE1);    // habilita a interrupção da porta C
            PCMSK1 |= (1 << PCINT8);  // habilita a interrupção do pino 0 da porta C
        }
        break; 
    default:
        if (cnt == 500) { // 1s
            gpt_enable_pwm_channel(GPTD3, 0, PWM_MAX);
            cnt = 0;
            fase = 1;
        }
        break;
    }
}

void cb_start(GPT_t* drv){
    static uint16_t cnt = 0;

    cnt++;
    if (cnt == 2500){
        gpt_start_notification(GPTD3, cb_calibration, 0);
    }
}

/*
 * Interrupções
 */

// interrupção da porta C
ISR(PCINT1_vect) {
    uint8_t new_value;
    uint8_t chg;
    volatile uint32_t new_tempo;

    PCICR &= ~(1 << PCIE1); // desabilita interrupção da porta C
    new_value = PINC;       // checar registrador da porta C

    if( ((new_value & (1 << PINC0)) != (old_value & (1 << PINC0))) && !(new_value & (1 << PINC0))){ // mudança no pino 0 da porta C e pino 0 = 0 (falling_edge)
        new_tempo = tempo;
        delta_tempo = new_tempo - old_tempo;
        old_tempo = new_tempo;
    }

    old_value = new_value;
    PCICR |= (1 << PCIE1); // habilita a interrupção da porta C
}


void init_uart() {
    Uart_Config_t uart_cfg = {F_CPU, UART_BAUD_RATE, DOUBLE_SPEED, SINGLE_CPU, ASSYNCHRONOUS, 
        DISABLE_PARITY, EIGHT_DATA_BITS, ONE_STOP_BIT, TX_RISING_RX_FALLING, 0, 0};

    uart_init(UARTD1);
    uart_start(UARTD1, &uart_cfg, TX_ON, RX_OFF);
}

int main(){
    /*
     * SETUP
     */

    // CONFIRAÇÕES DOS TEMPORIZADORES
    GPT_Config config_T0 = {MODE_CTC, DIVISOR_1, 159}; // 10 microssegundos (100kHz)
    GPT_Config config_T2 = {MODE_FAST_PWM_MAX_TOP, DIVISOR_128, 0xFF}; // ciclo de 2.057 milissegundos (486.2Hz 2ms = 249 e 1ms = 125)

    cli(); // desabilita interrupções para configurar as interrupções
    gpt_init();
    init_uart();

    // SETUP DOS PINOS E PORTAS
    gpio_set_pin_mode(GPIOD2, 3, GPIO_OUT);         // SAIDA PWM OC2A (pino 3 porta B ou ~D11 no UNO)
    gpio_set_pin_mode(GPIOD2, 5, GPIO_OUT);         // DEBUG (LED DA PLACA)
    gpio_set_pin_mode(GPIOD3, 0, GPIO_IN_PULLUP);   // EVENTO DA VELOCIDADE (pino 0 porta C ou A0 no UNO)
    gpio_set_pin_mode(GPIOD4, 3, GPIO_IN_PULLUP);   // BOTÃO/SWITCH DE INICIAR (pino 3 porta D ou D3 no UNO)
    gpio_set_pin_mode(GPIOD4, 5, GPIO_OUT);         // ACIONAMENTO DO ALIMENTAÇÃO DO MOTOR (NMOS) (D5 no UNO)

    gpio_clear_pin(GPIOD2, 3);
    gpio_clear_pin(GPIOD2, 5);
    gpio_clear_pin(GPIOD4, 5);
    
    // temporizador do PWM
    gpt_start(GPTD3, &config_T2);
    gpio_set_pin(GPIOD4, 5);
    gpt_start_notification(GPTD3, cb_start, 0);

    sei(); // habilita interrupções após configurar para iniciá-las ao mesmo tempo
    /*
     * LOOP
     */

    while(1)
        ;
    
    return 0;
}
