#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

typedef struct GPT_Struct GPT_t;
typedef void (*gpt_cb_t)(GPT_t* drv);

typedef struct GPT_Struct16 GPT16_t;
typedef void (*gpt_cb_16_t)(GPT16_t* drv);

typedef enum{
    RISING_EDGE, 
    FALLING_EDGE
} edge_mode_t; 

typedef enum {
    MODE_NORMAL,
    MODE_CTC,
    MODE_FAST_PWM_MAX_TOP,
    MODE_FAST_PWM_USER_TOP
} gpt_mode_t;

typedef enum {
    DIVISOR_1,
    DIVISOR_8,
    DIVISOR_32,
    DIVISOR_64,
    DIVISOR_128,
    DIVISOR_256,
    DIVISOR_1024,
} gpt_divisor_t;

typedef struct {
    gpt_mode_t mode;
    gpt_divisor_t divisor;
    uint8_t top;
} GPT_Config;

extern GPT_t GPT_obj1;
extern GPT_t GPT_obj3;
extern GPT16_t GPT_obj2;
#define GPTD1 (&GPT_obj1)
#define GPTD3 (&GPT_obj3)
#define GPTD2 (&GPT_obj2)

void gpt_init(void);
void gpt_start(GPT_t *gptp, const GPT_Config *config);
void gpt_stop(GPT_t *gptp); 
uint8_t gpt_start_notification(GPT_t *gptp, gpt_cb_t cb, uint8_t is_oneshot);
uint8_t gpt_stop_notification(GPT_t *gptp);
uint8_t gpt_change_interval(GPT_t *gptp, uint8_t interval);

uint8_t gpt_enable_pwm_channel(GPT_t *gptp, uint8_t channel, uint8_t width);
void gpt_disable_pwm_channel(GPT_t *gptp, uint8_t channel);

uint8_t gpt_start_channel_notification(GPT_t *gptp, uint8_t channel,
                                       uint8_t interval,
                                       gpt_cb_t cb, uint8_t is_oneshot);
void gpt_stop_channel_notification(GPT_t *gptp, uint8_t channel);

void gpt_start_t1(GPT16_t *gptp, uint8_t topl, uint8_t toph);
void gpt_stop_t1(GPT16_t *gptp);
uint8_t gpt_start_notification_t1(GPT16_t *gptp, uint8_t edge, uint8_t input_capture, gpt_cb_16_t cb, uint8_t is_oneshot);
uint8_t gpt_stop_notification_t1(GPT16_t *gptp);


#endif /* TIMER_H */