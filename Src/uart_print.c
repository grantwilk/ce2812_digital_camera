/**
  * @file uart_print.c
  * @author Grant Wilk
  * @created 2/22/2020
  * @modified 2/24/2020
  * @desc a set of basic printing functions via the UART
  */

# include <stdarg.h>
# include <stdio.h>
# include <string.h>
# include "stm32f4xx_hal.h"
# include "uart_print.h"

/**
 * Character Buffer
 */
char buffer[256];

/**
 * UART2 Pointer
 */
extern UART_HandleTypeDef huart2;

/**
 * Initializes the UART print functions
 */
void uart_print_init(void) {
    setvbuf(stdout, NULL, _IONBF, 0);
}

/**
 * Formatted print to UART2
 * @param fmt - formatted print string
 * @param ...
 */
int uart_printf(const char *fmt, ...) {

    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    int len = strlen(buffer);
    HAL_UART_Transmit(&huart2, (uint8_t *) buffer, len, 1000);

    return 0;
}

/**
 * Print memory dump to UART2
 * @param ptr - pointer to the start byte
 * @param cnt - number of bytes to dump
 */
void uart_dump(char * ptr, int cnt) {

    int div = 16;

    // print header
    uart_printf("\n");
    uart_printf("\t\t\t   00 01 02 03 \t04 05 06 07 \t08 09 10 11 \t12 13 14 15\n");
    uart_printf("\t\t\t-----------------------------------------------------------\n");

    // print dump rows
    for (int y = 0; y < cnt / div; y++) {
        uart_printf("0x%08X  |  ", (unsigned int) (ptr + (y * div)));
        for (int x = 0; x < div; x++) {
            if (x % 4 == 0 && x != 0) uart_printf("\t");
            uart_printf("%02X ", *(ptr + (y * div) + x));
        }
        uart_printf("\n");
    }

    // extra space
    uart_printf("\n");
}