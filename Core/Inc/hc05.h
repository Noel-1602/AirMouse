#ifndef __HC05_H__
#define __HC05_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* UART handle for USART2 connected to HC-05 module */
extern UART_HandleTypeDef huart2;

/*
 * Initialise USART2 for HC-05 Bluetooth module:
 * 9600 baud, 8 data bits, 1 stop bit, no parity, no hardware flow control.
 * Pins: PA2 (TX), PA3 (RX).
 */
int hc05_init(void);

/*
 * Transmit 5-byte airmouse packet over USART2 via HAL_UART_Transmit_IT:
 * byte 0: 0xAA (sync byte)
 * byte 1: dx (int8_t)
 * byte 2: dy (int8_t)
 * byte 3: button flags (bit0 = left, bit1 = right)
 * byte 4: XOR checksum (byte0 ^ byte1 ^ byte2 ^ byte3)
 *
 * Returns:
 *   0  on success (transmit started)
 *  -1  if previous transmit is still busy (packet skipped)
 *  -2  if HAL_UART_Transmit_IT returned error
 */
int hc05_send_mouse(int8_t dx, int8_t dy, uint8_t buttons);

/* USART2 interrupt handler */
void USART2_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __HC05_H__ */
