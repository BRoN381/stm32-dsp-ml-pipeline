#ifndef UART_H
#define UART_H

#include <stdint.h>

void uart_init(void);
void uart_write_char(char c);
void uart_print(const char *str);
void uart_print_hex(uint8_t val);
void uart_print_hex32(uint32_t val);
void uart_print_int(int16_t val);
#endif /* UART_H */
