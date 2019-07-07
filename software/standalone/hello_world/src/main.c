#include <stdint.h>

#include "incus.h"

void main() {
	uart_writeStr(UART_A, "Hello World from Incus!!!\r\n");
}

