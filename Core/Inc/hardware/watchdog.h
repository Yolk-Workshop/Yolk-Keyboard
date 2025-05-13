/**
* watchdog.h - Simple, efficient watchdog interface
*
* Created on: Apr 10, 2025
* Author: bettysidepiece
* Modified: May 11, 2025
*/

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include "main.h"
#include "logger.h"

#define WDG_TIMEOUT         8000   // 8-second watchdog timeout
#define WDG_REFRESH_PERIOD  6000   // Refresh watchdog every 6 seconds

// Function prototypes
void WDG_Init(uint32_t timeout_ms);
void WDG_Refresh(void);
void WDG_DisableInDebug(void);

#endif /* WATCHDOG_H_ */
