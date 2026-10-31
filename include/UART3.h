#ifndef UART3_H
#define UART3_H

#include "stm32f4xx.h"
void UART3_Init(void);
void uart3_send_string(const char *s);
void uart3_send_char(char c);
void uart3_send_hex(uint8_t value);
void uart3_send_int32(int32_t val);
void uart3_send_float(float value);
#endif
