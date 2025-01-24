#include "logger.h"
#include <stdarg.h>

static char uart_buffer[LOG_BUFFER_SIZE];

/**
 * @brief Format a message and pass it to the output function
 * @param format: The format string (printf-style)
 * @param ...: Variable arguments
 */
void logger_print(const char *format, ...) {
    va_list args;

    // Format the string into the buffer
    va_start(args, format);
    vsnprintf(uart_buffer, sizeof(uart_buffer), format, args);
    va_end(args);

    // Pass the formatted message to the output handler
    logger_output(uart_buffer);
}
