#ifndef TIMERCTRL_H_
#define TIMERCTRL_H_

#include <stdint.h>

typedef struct
{
  volatile uint64_t counter;
  volatile uint64_t cmp;
} Timer_Reg;

#endif /* TIMERCTRL_H_ */
