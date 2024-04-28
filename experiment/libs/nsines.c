#ifndef NSiNES_C
#define NSiNES_C

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

//#define N 10.0
#define W_MIN 0.23
#define W_MAX 2.3
#define OFFSET 187
#define GAIN 12.0
#define PI 3.141592

#define W_DELTA W_MAX-W_MIN

/*
 * Funções de sinal de entrada
 */

volatile uint8_t NSines(int32_t t, float n) {
  float u = 0;
  float tt = (float) t / 1e5;

  for(int8_t j = 0; j<n; j++)
    u += cos( (W_MIN + (j*(W_MAX-W_MIN)/n))*tt - (j*(j+1)*PI)/n );
  
  u = GAIN * u;
  u = OFFSET + u;

  return (uint8_t) u;
}

#endif