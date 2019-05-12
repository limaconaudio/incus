#include <string.h>
#include <stdint.h>
#include <platform.h>

#include "mini-printf.h"

unsigned long int cnt;

void _puts(char *str){
	while(*str){
		uart_write(UART_BASE,*(str++));
	}
}

static void set_mtimecmp(uint64_t time)
{
	TIMER_BASE->cmp = time;
}

static inline uint64_t mtime(void)
{
	return TIMER_BASE->counter;
}

static inline void uart_init(unsigned int baud)
{
	Uart_Config uartConfig;
	uartConfig.dataLength = 8;
	uartConfig.parity = NONE;
	uartConfig.stop = ONE;
	uartConfig.clockDivider = CORE_HZ/8/baud-1;
	uart_applyConfig(UART_BASE, &uartConfig);
}

static void delay(unsigned long int delay)
{
	unsigned long int i;

	for (i = 0; i < delay; i++);
}

int main() {
	GPIO_BASE->OUTPUT_ENABLE = 0x0000000F;

	uart_init(115200);
	set_mtimecmp(mtime() + 6000);

	while (1)
		cnt++;
}

void thread1(void)
{
	char buff[30];

	mini_snprintf(buff, sizeof(buff), "Thread 1\r\n");
	_puts(buff);
}

void thread2(void)
{
	char buff[30];

	mini_snprintf(buff, sizeof(buff), "Thread 2\r\n");
	_puts(buff);
}

void irqCallback()
{
	if (!(cnt % 2))
		thread1();
	else
		thread2();

	set_mtimecmp(mtime() + 6000);
}
