/*
 * logger.h
 *
 *  Created on: Dec 17, 2024
 *      Author: bettysidepiece
 */

#ifndef INC_LOGGER_H_
#define INC_LOGGER_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define LOG_BUFFER_COUNT 8  // Number of buffers in the queue
#define LOG_BUFFER_SIZE 128 // Size of each buffer

typedef struct {
    char buffers[LOG_BUFFER_COUNT][LOG_BUFFER_SIZE];
    volatile uint8_t write_index;
    volatile uint8_t read_index;
    volatile uint8_t buffer_count;
    volatile uint8_t dma_busy;
} log_buffer_t;

// Debug levels
typedef enum {
    LOG_LEVEL_NONE = 0,   // No logging
    LOG_LEVEL_ERROR,      // Errors only
    LOG_LEVEL_WARNING,    // Warnings and errors
    LOG_LEVEL_INFO,       // Informational messages
    LOG_LEVEL_DEBUG       // Debugging messages
} log_level_t;

// Buffer size for log messages
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
#define BUFFER_SIZE 128

// Default debug level (can be modified during compilation)
#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL LOG_LEVEL_DEBUG
#endif

// Logging macros
#define LOG_ERROR(format, ...)   if (DEBUG_LEVEL >= LOG_LEVEL_ERROR)   logger_print("[ERROR] " format "\r\n", ##__VA_ARGS__)
#define LOG_WARNING(format, ...) if (DEBUG_LEVEL >= LOG_LEVEL_WARNING) logger_print("[WARNING] " format "\r\n", ##__VA_ARGS__)
#define LOG_INFO(format, ...)    if (DEBUG_LEVEL >= LOG_LEVEL_INFO)    logger_print("[INFO] " format "\r\n", ##__VA_ARGS__)
#define LOG_DEBUG(format, ...)   if (DEBUG_LEVEL >= LOG_LEVEL_DEBUG)   logger_print("[DEBUG] " format "\r\n", ##__VA_ARGS__)

// Function prototypes
void logger_print(const char *format, ...);
extern void logger_output(const char *message);

#endif /* INC_LOGGER_H_ */
