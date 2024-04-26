#ifndef __AVR_ATmega328P__
    #define __AVR_ATmega328P__
#endif

#include <avr/interrupt.h>
#include "timer.h"


/*
 * Definição da estrutura de alguns registradores (mas não todos) do
 * temporizador 0 e 2 de acordo com a ordem sequencial dos
 * registradores no endereçamento de memória
 */
typedef struct {
    volatile uint8_t tccra; // Timer/Counter Control Register A
    volatile uint8_t tccrb; // Timer/Counter Control Register B
    volatile uint8_t tcnt;  // Timer/Counter
    volatile uint8_t ocra;  // Output Compare Register A
    volatile uint8_t ocrb;  // Output Compare Register B
} GPT_Regs_t;

/*
 * Definição da estrutura de dados do driver que representa o
 * temporizador. Esta estrutura deve conter tudo o que é necessário
 * para as função abaixo trabalharem com os temporizadores 0 e 2.
 */
struct GPT_Struct {
    GPT_Regs_t* regs;        // registradores
    volatile uint8_t* tifr;  // timer flag
    volatile uint8_t* timsk; // timer mascara
    gpt_mode_t current_mode; // modo de operação
    gpt_divisor_t divisor;   //
    gpt_cb_t current_cb;     // callback atual 
    uint8_t is_oneshot;      // 
    gpt_cb_t current_channel_cb[2];
    uint8_t is_channel_oneshot[2];
};

typedef struct {
    volatile uint8_t tccra; // Timer/Counter Control Register A
    volatile uint8_t tccrb; // Timer/Counter Control Register B
    volatile uint8_t tccrc; // Timer/Counter Control Register C
} GPT_Regs_crt_t;
typedef struct {
    volatile uint8_t tcntl;  // Timer/Counter low
    volatile uint8_t tcnth;  // Timer/Counter high
    volatile uint8_t icrl;   // Input capture Register
    volatile uint8_t icrh;   // Input capture Register
    volatile uint8_t ocral;  // Output Compare Register A
    volatile uint8_t ocrah;  // Output Compare Register A
    volatile uint8_t ocrbl;  // Output Compare Register B
    volatile uint8_t ocrbh;  // Output Compare Register B
} GPT_Regs16_t;

struct GPT_Struct16 {
    GPT_Regs_crt_t* regs_crt;// registradores de controle
    GPT_Regs16_t* regs;      // registradores
    volatile uint8_t* tifr;  // timer flag
    volatile uint8_t* timsk; // timer mascara
    gpt_mode_t current_mode; // modo de operação
    gpt_divisor_t divisor;   //
    gpt_cb_16_t current_cb;  // callback atual 
    uint8_t is_oneshot;      // 
    gpt_cb_16_t current_channel_cb[2];
    uint8_t is_channel_oneshot[2];
};

/*
 * Definição das variáveis referentes a estrutura de dados para os
 * temporizador 0, 2 e 1, respectivamente.
 */
GPT_t GPT_obj1;
GPT_t GPT_obj3;
GPT16_t GPT_obj2;

/*
 * Função para inicializar as variáveis relativas as estruturas que
 * representam os temporizadores do microcontrolador
 */
void gpt_init(void) {
    GPT_obj1.regs = (GPT_Regs_t *) &TCCR0A;
    GPT_obj3.regs = (GPT_Regs_t *) &TCCR2A;
    GPT_obj1.tifr = &TIFR0;
    GPT_obj3.tifr = &TIFR2;
    GPT_obj1.timsk = &TIMSK0;
    GPT_obj3.timsk = &TIMSK2;

    GPT_obj1.current_cb = 0;
    GPT_obj1.current_channel_cb[0] = GPT_obj1.current_channel_cb[1] = 0;
    GPT_obj3.current_cb = 0;
    GPT_obj3.current_channel_cb[0] = GPT_obj3.current_channel_cb[1] = 0;

    GPT_obj2.regs_crt = (GPT_Regs_crt_t *) &TCCR1A;
    GPT_obj2.regs = (GPT_Regs16_t *) &TCNT1L;
    GPT_obj2.tifr = &TIFR1;
    GPT_obj2.timsk = &TIMSK1;
    GPT_obj2.current_cb = 0;
    GPT_obj2.current_channel_cb[0] = GPT_obj2.current_channel_cb[1] = 0;

    /* NÃO HÁ MAIS NADA A IMPLEMENTAR AQUI */
}

void gpt_start_t1(GPT16_t *gptp, uint8_t topl, uint8_t toph) {

    gptp->regs->tcntl &= ~(0xFF);
    gptp->regs->tcnth &= ~(0xFF);
    gptp->regs_crt->tccra &= 0x00; // Normal port operation, OC1A/OC1B disconnected.
    gptp->regs_crt->tccrb |= ((1 << CS11) | (0 << CS10)); // modo de operação normal, divisor 16 (meio microssegundo)
    gptp->regs_crt->tccrc &= 0x00; 

    gptp->regs->ocral = topl;   // valor topo da contagem
    gptp->regs->ocrah = toph;   // valor topo da contagem
}

/*
 * Função para iniciar a contagem do temporizador (mas não as
 * notificações ou geração de sinal PWM).  Aqui deve ser configurado,
 * de acordo com os valores passados no parâmetro config, o modo de
 * contagem e, dependendo do modo, o topo da contagem.
 */
void gpt_start(GPT_t *gptp, const GPT_Config *config) {
    uint8_t MODE_MSK_A = 0b00000011;
    uint8_t MODE_MSK_B = 0b00001000;
    uint8_t DIV_MSK = 0b00000111;

    gptp->regs->tcnt &= ~(0xFF);
    gptp->current_mode = config->mode; // modo de operação
    gptp->divisor = config->divisor;   // setando o divisor

    gptp->regs->tccra &= ~(MODE_MSK_A);
    gptp->regs->tccrb &= ~(MODE_MSK_B);
    switch (gptp->current_mode)
    {
        case MODE_NORMAL: // 0 00 
            gptp->regs->tccra |= (0x00 & MODE_MSK_A);
            break;
        case MODE_CTC: //0 10
            gptp->regs->tccra |= (0x02 & MODE_MSK_A);
            gptp->regs->ocra = config->top;    // valor topo da contagem
            break;
        case MODE_FAST_PWM_MAX_TOP: // 0 11
            gptp->regs->tccra |= (0x03 & MODE_MSK_A);
            break;
        case MODE_FAST_PWM_USER_TOP: //1 11
            gptp->regs->tccra |= (0x03 & MODE_MSK_A);
            gptp->regs->tccrb |= (MODE_MSK_B);
            gptp->regs->ocra = config->top;    // valor topo da contagem
            break;
        default:
            gptp->regs->tccra |= (0x00 & MODE_MSK_A);
            gptp->regs->tccrb &= ~(MODE_MSK_B);
            break;
    }

    gptp->regs->tccrb &= ~(DIV_MSK);
    if (gptp->tifr == &TIFR2){
        switch (gptp->divisor)
        {
            case DIVISOR_1: // 001
                gptp->regs->tccrb |= (0x01 & DIV_MSK);
                break;
            case DIVISOR_8: // 010
                gptp->regs->tccrb |= (0x02 & DIV_MSK);
                break;
            case DIVISOR_32: // 011
                gptp->regs->tccrb |= (0x03 & DIV_MSK);
                break;
            case DIVISOR_64: // 100
                gptp->regs->tccrb |= (0x04 & DIV_MSK);
                break;
            case DIVISOR_128: // 101
                gptp->regs->tccrb |= (0x05 & DIV_MSK);
                break;
            case DIVISOR_256: // 110
                gptp->regs->tccrb |= (0x06 & DIV_MSK);
                break;
            case DIVISOR_1024: // 111
                gptp->regs->tccrb |= (0x07 & DIV_MSK);
                break;
            default:
                gptp->regs->tccrb |= (0x01 & DIV_MSK);
                break;
        }
    } else {
        switch (gptp->divisor)
        {
            case DIVISOR_1: // 001
                gptp->regs->tccrb |= (0x01 & DIV_MSK);
                break;
            case DIVISOR_8: // 010
                gptp->regs->tccrb |= (0x02 & DIV_MSK);
                break;
            case DIVISOR_64: // 011
                gptp->regs->tccrb |= (0x03 & DIV_MSK);
                break;
            case DIVISOR_256: // 100
                gptp->regs->tccrb |= (0x04 & DIV_MSK);
                break;
            case DIVISOR_1024: // 101
                gptp->regs->tccrb |= (0x05 & DIV_MSK);
                break;
            default:
                gptp->regs->tccrb |= (0x01 & DIV_MSK);
                break;
        }
    }

    /* ACHO QUE TÁ OK */
}

/*
 * Função para parar a contagem do temporizador (as notificações e
 * geração de sinal PWM, apesar de continuarem habilitadas, não mais
 * ocorrerão pelo contador estar parado)
 */
void gpt_stop(GPT_t *gptp) {
    uint8_t CS_MSK = 0b00000111;

    gptp->regs->tccrb &= ~(CS_MSK); // para a contagem
    gptp->regs->tcnt &= ~(0xFF);    // zera o contador

    /* ACHO QUE TÁ OK */
}

void gpt_stop_t1(GPT16_t *gptp) {
    uint8_t CS_MSK = 0b00000111;

    gptp->regs_crt->tccrb &= ~(CS_MSK);
    gptp->regs->tcntl &= ~(0xFF);
    gptp->regs->tcnth &= ~(0xFF);

    /* ACHO QUE TÁ OK */
}

uint8_t gpt_start_notification_t1(GPT16_t *gptp, uint8_t edge, uint8_t input_capture, gpt_cb_16_t cb, uint8_t is_oneshot) {
    gptp->current_cb = cb;
    gptp->is_oneshot = is_oneshot;
    gptp->regs_crt->tccrb |= (edge << ICES1);     // borda de captura
    *gptp->tifr |= (1 << ICF1) | (1 << TOV1);     // Flag de interrupção e Overflow
    *gptp->timsk |= (1 << ICIE1) | (1 << TOIE1);  // Input Capture Interrupt Enable
}

/*
 * Função para interromper as notificações.  O temporizador ainda deve
 * continuar a contar, mas as notificações não mais ocorrerão.
 */
uint8_t gpt_stop_notification_t1(GPT16_t *gptp) {
    *gptp->timsk &= ~(1 << TOIE1);

    /* NÃO HÁ MAIS NADA A IMPLEMENTAR AQUI */
}

/*
 * Função para iniciar as notificações de reinício da contagem (timer
 * overflow).
 *
 * Essas notificações consistem na chamada da função passada como
 * parâmetro no ponteiro cb(). Se o parâmetro is_oneshot for diferente
 * de 0, a função é chamada uma única vez (apenas uma notificação
 * ocorrerá)
 */
uint8_t gpt_start_notification(GPT_t *gptp, gpt_cb_t cb, uint8_t is_oneshot) {
    gptp->current_cb = cb;
    gptp->is_oneshot = is_oneshot;

    *gptp->tifr |= (1 << TOV0);
    *gptp->timsk |= (1 << TOIE0);
    /* NÃO HÁ MAIS NADA A IMPLEMENTAR AQUI */
}

/*
 * Função para interromper as notificações.  O temporizador ainda deve
 * continuar a contar, mas as notificações não mais ocorrerão.
 */
uint8_t gpt_stop_notification(GPT_t *gptp) {
    *gptp->timsk &= ~(1 << TOIE0);

    /* NÃO HÁ MAIS NADA A IMPLEMENTAR AQUI */
}

/*
 * Função para mudar o intervalo de ocorrência da notificação de
 * reinício da contagem, o que é o mesmo do topo da contagem.  Observe
 * que se o modo for o modo normal de contagem, este valor não pode
 * ser mudado e deve ser ignorado.
 */
uint8_t gpt_change_interval(GPT_t *gptp, uint8_t interval) {
    if (gptp->current_mode != MODE_NORMAL || gptp->current_mode != MODE_FAST_PWM_USER_TOP)
        gptp->regs->ocra = interval;

    /* ACHO QUE TÁ OK */
}

/*
 * Função para iniciar a geração do sinal PWM em um dos canais do
 * temporizador. A duração do valor lógico 1 do sinal PWM em pulsos do
 * sinal de relógio é dado pelo parâmetro width, de modo que o ciclo
 * de trabalho será a razão entre o valor de width e o valor do topo
 * da contagem mais 1.
 *
 * Observe que os temporizadores do ATmega328p tem 2 canais cada. O
 * parâmetro channel é 0 para o primeiro canal e 1 para o segundo.
 */
uint8_t gpt_enable_pwm_channel(GPT_t *gptp, uint8_t channel, uint8_t width) {

    *gptp->timsk |= (1 << TOIE0);
    if (!channel) {
        gptp->regs->ocra = width;
        *gptp->timsk |= (1 << OCIE0A);
        gptp->regs->tccra |= (1 << COM0A1); // LIGA PWM NA PORTA DIGITAL, 0 NA COMPARAÇÃO 1 NO OVERLOW
    }else{
        gptp->regs->ocrb = width;
        *gptp->timsk |= (1 << OCIE0B);
        gptp->regs->tccra |= (1 << COM0B1); // LIGA PWM NA PORTA DIGITAL, 0 NA COMPARAÇÃO 1 NO OVERLOW
    }
}

/*
 * Função para parar a geração do sinal PWM no canal
 * especificado. Observe que o contador do temporizador deve continuar
 * a contar.
 */
void gpt_disable_pwm_channel(GPT_t *gptp, uint8_t channel) {
    if (!channel){
        *gptp->timsk &= ~(1 << OCIE0A);
        gptp->regs->tccra &= ~((1 << COM0A1)|(1 << COM0A0));
    } else {
        *gptp->timsk &= ~(1 << OCIE0B);
        gptp->regs->tccra &= ~((1 << COM0B1)|(1 << COM0B0));
    }

    /* ACHO QUE TÁ OK */
}

/*
 * Função para iniciar as notificações de igualdade na comparação
 * (compare match).
 *
 * Essas notificações consistem na chamada da função passada como
 * parâmetro no ponteiro cb(). Se o parâmetro is_oneshot for diferente
 * de 0, a função é chamada uma única vez (apenas uma notificação
 * ocorrerá)
 */
uint8_t gpt_start_channel_notification(GPT_t *gptp, uint8_t channel,
                                       uint8_t interval,
                                       gpt_cb_t cb, uint8_t is_oneshot) {
    if (gptp->current_mode == MODE_FAST_PWM_USER_TOP)
        gptp->regs->ocra = interval;

    if (channel) {// channel 1 é a interrupção do OCIE0B 
        *gptp->tifr |= (1 << OCF0B);
        *gptp->timsk |= (1 << OCIE0B);
        gptp->current_channel_cb[1] = cb;
        gptp->is_channel_oneshot[1] = is_oneshot;
    }else{
        *gptp->tifr |= (1 << OCF0A);
        *gptp->timsk |= (1 << OCIE0A);
        gptp->current_channel_cb[0] = cb;
        gptp->is_channel_oneshot[0] = is_oneshot;
    }
}

/*
 * Função para parar as notificações de igualdade na comparação
 * (compare match). Obserqve que o contador deve continuar a contar.
 */
void gpt_stop_channel_notification(GPT_t *gptp, uint8_t channel) {
    if (channel) // channel 1 é a interrupção do OCIE0B 
        *gptp->timsk &= ~(1 << OCIE0B);
    else
        *gptp->timsk &= ~(1 << OCIE0A);

    /* ACHO QUE TÁ OK */
}


/*
 * Rotina de tratamento da interrupção de overflow do temporizador
 * 0. Se o ponteiro para a função de callback for não-nulo, chama a
 * função, e se o callback for oneshot, desativa a interrupção (mas
 * não o temporizador, pois pode ainda estar gerando algum sinal PWM.
 */
ISR(TIMER0_OVF_vect) {
    if (GPT_obj1.current_cb)
        GPT_obj1.current_cb(&GPT_obj1);

    if (GPT_obj1.is_oneshot)
        TIMSK0 &= ~(1 << TOIE0);

    /* NÃO HÁ MAIS NADA A IMPLEMENTAR AQUI */
}

/*
 * Rotina de tratamento da interrupção de comparação do 1o canal. Se o
 * ponteiro para a função de callback do canal for não-nulo, chama a
 * função, e se o callback for oneshot, desativa a interrupção
 */
ISR(TIMER0_COMPA_vect) {
    if (GPT_obj1.current_channel_cb[0])
        GPT_obj1.current_channel_cb[0](&GPT_obj1);

    if (GPT_obj1.is_channel_oneshot[0])
        TIMSK0 &= ~(1 << OCIE0A);

    /* NÃO HÁ MAIS NADA A IMPLEMENTAR AQUI */
}

/*
 * Rotina de tratamento da interrupção de comparação do 2o canal. Se o
 * ponteiro para a função de callback do canal for não-nulo, chama a
 * função, e se o callback for oneshot, desativa a interrupção
 */
ISR(TIMER0_COMPB_vect) {
    if (GPT_obj1.current_channel_cb[1])
        GPT_obj1.current_channel_cb[1](&GPT_obj1);

    if (GPT_obj1.is_channel_oneshot[1])
        TIMSK0 &= ~(1 << OCIE0B);

    /* NÃO HÁ MAIS NADA A IMPLEMENTAR AQUI */
}

/*
 * Rotina de tratamento da interrupção de overflow do temporizador
 * 2. Se o ponteiro para a função de callback for não-nulo, chama a
 * função, e se o callback for oneshot, desativa a interrupção (mas
 * não o temporizador, pois pode ainda estar gerando algum sinal PWM.
 */
ISR(TIMER2_OVF_vect) {
    if (GPT_obj3.current_cb)
        GPT_obj3.current_cb(&GPT_obj3);

    if (GPT_obj3.is_oneshot)
        TIMSK2 &= ~(1 << TOIE2);

    /* NÃO HÁ MAIS NADA A IMPLEMENTAR AQUI */
}

/*
 * Rotina de tratamento da interrupção de comparação do 1o canal. Se o
 * ponteiro para a função de callback do canal for não-nulo, chama a
 * função, e se o callback for oneshot, desativa a interrupção
 */
ISR(TIMER2_COMPA_vect) {
    if (GPT_obj3.current_channel_cb[0])
        GPT_obj3.current_channel_cb[0](&GPT_obj3);

    if (GPT_obj3.is_channel_oneshot[0])
        TIMSK2 &= ~(1 << OCIE2A);

    /* NÃO HÁ MAIS NADA A IMPLEMENTAR AQUI */
}

/*
 * Rotina de tratamento da interrupção de comparação do 2o canal. Se o
 * ponteiro para a função de callback do canal for não-nulo, chama a
 * função, e se o callback for oneshot, desativa a interrupção
 */
ISR(TIMER2_COMPB_vect) {
    if (GPT_obj3.current_channel_cb[1])
        GPT_obj3.current_channel_cb[1](&GPT_obj3);

    if (GPT_obj3.is_channel_oneshot[1])
        TIMSK2 &= ~(1 << OCIE2B);

    /* NÃO HÁ MAIS NADA A IMPLEMENTAR AQUI */
}

/*
 * Rotina de tratamento da interrupção de overflow do temporizador
 * 1. Se o ponteiro para a função de callback for não-nulo, chama a
 * função, e se o callback for oneshot, desativa a interrupção (mas
 * não o temporizador, pois pode ainda estar gerando algum sinal PWM.
 */
ISR(TIMER1_OVF_vect) {
    if (GPT_obj2.current_cb)
        GPT_obj2.current_cb(&GPT_obj2);

    if (GPT_obj2.is_oneshot)
        TIMSK1 &= ~(1 << TOIE1);

    /* NÃO HÁ MAIS NADA A IMPLEMENTAR AQUI */
}