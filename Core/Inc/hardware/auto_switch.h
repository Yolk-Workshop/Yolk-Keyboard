/*
 * auto_switch.h
 *
 * Created on: Jun 6, 2025
 * Author: bettysidepiece
 */

#ifndef INC_HARDWARE_AUTO_SWITCH_H_
#define INC_HARDWARE_AUTO_SWITCH_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32l0xx.h"
#include "logger.h"
#include "keys.h"
#include "pmsm.h"

typedef enum {
    MODE_SWITCH_IDLE,
    MODE_SWITCH_IN_PROGRESS,
    MODE_SWITCH_COMPLETE
} mode_switch_state_t;

extern mode_switch_state_t switch_state;
extern uint32_t switch_start_time;

void check_vbus_and_switch_mode(void);
void check_connection(void);

#endif /* INC_HARDWARE_AUTO_SWITCH_H_ */
