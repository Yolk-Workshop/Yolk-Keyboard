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

// Debug levels
typedef enum {
    LOG_LEVEL_NONE = 0,   // No logging
    LOG_LEVEL_ERROR,      // Errors only
    LOG_LEVEL_WARNING,    // Warnings and errors
    LOG_LEVEL_INFO,       // Informational messages
    LOG_LEVEL_DEBUG       // Debugging messages
} log_level_t;

// Buffer size for log messages
#define BUFFER_SIZE 256

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
