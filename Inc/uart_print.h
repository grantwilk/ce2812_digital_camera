/**
  * @file uart_print.h
  * @author Grant Wilk
  * @created 2/24/2020
  * @modified 2/24/2020
  * @desc 
  */

#ifndef CE2812_WK09_LAB_DIGITAL_CAMERA_UART_PRINT_H
#define CE2812_WK09_LAB_DIGITAL_CAMERA_UART_PRINT_H
#endif

/**
 * Initializes the UART print functions
 */
void uart_print_init(void);

/**
 * Formatted print to UART2
 * @param fmt - formatted print string
 * @param ...
 */
int uart_printf(const char *fmt, ...);

/**
 * Print memory dump to UART2
 * @param ptr - pointer to the start byte
 * @param cnt - number of bytes to dump
 */
void uart_dump(char * ptr, int cnt);