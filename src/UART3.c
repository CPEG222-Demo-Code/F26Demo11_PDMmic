#include "stm32f4xx.h"
#include <stdio.h>
#include "UART3.h"

#define UART_TX_PIN     8  // PD8
#define UART_RX_PIN     9  // PD9
#define UART_PORT       GPIOD
#define BAUDRATE        115200

static uint32_t uart3_get_pclk1_hz(void) {
  static const uint16_t ahb_presc_table[16] = {
      1, 1, 1, 1, 1, 1, 1, 1, 2, 4, 8, 16, 64, 128, 256, 512};
  static const uint8_t apb_presc_table[8] = {1, 1, 1, 1, 2, 4, 8, 16};

  SystemCoreClockUpdate();

  uint32_t hpre = (RCC->CFGR >> RCC_CFGR_HPRE_Pos) & 0x0FU;
  uint32_t ppre1 = (RCC->CFGR >> RCC_CFGR_PPRE1_Pos) & 0x07U;
  uint32_t hclk = SystemCoreClock / ahb_presc_table[hpre];
  return hclk / apb_presc_table[ppre1];
}

void UART3_Init(void) {
  // Enable GPIOD and USART3 clocks
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

  // Set PD8 (TX) and PD9 (RX) to alternate function AF7
  GPIOD->MODER &= ~((0x3 << (UART_TX_PIN * 2)) | (0x3 << (UART_RX_PIN * 2)));
  GPIOD->MODER |= (0x2 << (UART_TX_PIN * 2)) | (0x2 << (UART_RX_PIN * 2));
  GPIOD->AFR[1] &= ~((0xF << ((UART_TX_PIN - 8) * 4)) | (0xF << ((UART_RX_PIN - 8) * 4)));
  GPIOD->AFR[1] |= (0x7 << ((UART_TX_PIN - 8) * 4)) | (0x7 << ((UART_RX_PIN - 8) * 4));

  // Configure USART3: 115200 baud, 8N1 (oversampling by 16)
  uint32_t pclk1 = uart3_get_pclk1_hz();
  USART3->BRR = (pclk1 + (BAUDRATE / 2U)) / BAUDRATE;
  USART3->CR1 =
      USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // Enable TX, RX, USART
}

void uart3_send_char(char c) {
  while (!(USART3->SR & USART_SR_TXE))// Wait until TXE (Transmit Data Register Empty) flag is set
    ;
  USART3->DR = c; // Write the character to the data register
}

void uart3_send_string(const char *s) {
  while (*s) // Loop until the null terminator is reached
    uart3_send_char(*s++); // Send each character in the string
}

void uart3_send_int32(int32_t val) {
  char buf[12];
  sprintf(buf, "%ld", (long)val);
  uart3_send_string(buf);
}

void uart3_send_hex(uint8_t value) {
    const char hex[] = "0123456789ABCDEF";
    uart3_send_char(hex[(value >> 4) & 0x0F]);
    uart3_send_char(hex[value & 0x0F]);
}

void uart3_send_float(float value) {
    char buf[20];
    sprintf(buf, "%f", value);
    uart3_send_string(buf);
}