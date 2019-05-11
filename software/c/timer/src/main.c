#include <string.h>
#include <stdint.h>
#include <platform.h>

#include "mini-printf.h"

void _puts(char *str){
	while(*str){
		uart_write(UART_BASE,*(str++));
	}
}

static void set_mtimecmp(uint64_t time)
{
	volatile uint32_t *r = (uint32_t *)RISCV_MTIMECMP_BASE;

	/* Per spec, the RISC-V MTIME/MTIMECMP registers are 64 bit,
	 * but are NOT internally latched for multiword transfers.  So
	 * we have to be careful about sequencing to avoid triggering
	 * spurious interrupts: always set the high word to a max
	 * value first.
	 */
	r[1] = 0xffffffff;
	r[0] = (uint32_t)time;
	r[1] = (uint32_t)(time >> 32);
}

static uint64_t mtime(void)
{
	volatile uint32_t *r = (uint32_t *)RISCV_MTIME_BASE;
	uint32_t lo, hi;

	/* Likewise, must guard against rollover when reading */
	do {
		hi = r[1];
		lo = r[0];
	} while (r[1] != hi);

	return (((uint64_t)hi) << 32) | lo;
}

static void uart_init(unsigned int baud)
{
	Uart_Config uartConfig;
	uartConfig.dataLength = 8;
	uartConfig.parity = NONE;
	uartConfig.stop = ONE;
	uartConfig.clockDivider = CORE_HZ/8/baud-1;
	uart_applyConfig(UART_BASE, &uartConfig);
}

static void delay(int delay)
{
	int i;

	for (i = 0; i < delay; i++);
}

int main() {
	char buff[30];
	uint64_t time;

	uart_init(115200);

	while (1) {
		time = mtime();
		mini_snprintf(buff, sizeof buff, "mtime: %u%u\r\n",
					(time >> 32), (uint32_t)time);
		_puts(buff);
		delay(300000);
	}
}

void irqCallback()
{
}
