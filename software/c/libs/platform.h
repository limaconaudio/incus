/*
 * briey.h
 *
 *  Created on: Aug 24, 2016
 *      Author: clp
 */

#ifndef BRIEY_H_
#define BRIEY_H_

#include "timer.h"
#include "interrupt.h"
#include "uart.h"
#include "gpio.h"

#define CORE_HZ 100000000

#define GPIO_BASE	((Gpio_Reg*)(0xF0000000))
#define UART_BASE	((Uart_Reg*)(0xF0010000))
#define TIMER_BASE	((Timer_Reg*)(0xF0008000))

#define RISCV_MTIME_BASE	(TIMER_BASE)
#define RISCV_MTIMECMP_BASE	(TIMER_BASE + 0x08)

#endif /* BRIEY_H_ */
