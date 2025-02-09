#ifndef IS31FLXX_H
#define IS31FLXX_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32l0xx_hal.h"
#include "i2c_core.h"

extern i2c_device i2c1_inst;

#define DRIVER_RESET 0x00

#define CMD_REG 0xFD // Reset after each command
#define CMD_LOCK_REG 0xFE
#define CMD_WRITE_EN 0xC5
#define INT_MSK_REG 0xF0
#define INT_STATUS_REG 0xF1

// LED Control Register Addresses
#define MAX_PWM 0xFF
#define MIN_PWM 0x00
#define MAX_GCC 0xFF
#define MID_GCC 127U

// Configuration Register Macros
#define CFG_SYNC_HIZ  0b00
#define CFG_SYNC_MAIN 0b01
#define CFG_SYNC_SUB  0b10

#define CFG_OSD_EN   0b00
#define CFG_OSD_OFF  0b01

#define AUTO_BREATH_OFF 0
#define AUTO_BREATH_ON  1

#define SSD_SW_SD    0
#define SSD_NORM_OP  1

// Interrupt Status Register Flags
#define IS31FL3737_ISR_ABM3_FINISH_Msk  (1 << 4)
#define IS31FL3737_ISR_ABM2_FINISH_Msk  (1 << 3)
#define IS31FL3737_ISR_ABM1_FINISH_Msk  (1 << 2)
#define IS31FL3737_ISR_SHORT_Msk        (1 << 1)
#define IS31FL3737_ISR_OPEN_Msk         (1 << 0)

// Interrupt Mask Register Flags
#define IS31FL3737_IMR_AUTO_CLEAR_Msk   (1 << 3)
#define IS31FL3737_IMR_AUTO_BREATH_Msk  (1 << 2)
#define IS31FL3737_IMR_DOT_SHORT_Msk    (1 << 1)
#define IS31FL3737_IMR_DOT_OPEN_Msk     (1 << 0)

// Helper macros for page and command access
#define IS31FL_SELECT_PAGE(page)    is31fl_write_reg(CMD_REG, page);
// LED Matrix Data Structure
typedef struct __attribute__((packed)) {
    uint8_t unused : 2;   // D7:D6 (unused bits)
    uint8_t led_bits : 6; // D5:D0 (CCS, OP, ST)
} led_register;

typedef struct {
    led_register on_off[2];  // CS1~CS6 and CS7~CS12
    led_register open[2];    // Open detection
    led_register short_[2];  // Short detection
} led_sw;

typedef struct {
    uint8_t SSD  : 1;  // D0: Software Shutdown (0 = Shutdown, 1 = Normal)
    uint8_t B_EN : 1;  // D1: Auto Breath Enable (0 = PWM mode, 1 = ABM)
    uint8_t OSD  : 1;  // D2: Open/Short Detection Trigger
    uint8_t _reserved : 3; // D5:D3: Unused (default 000)
    uint8_t SYNC : 2;  // D7:D6: Synchronization Mode (00 = Normal, 01 = Master, 10 = Slave)
} cfg_reg_t;

typedef struct {
    uint8_t _OB   : 1;  // D0: Open Interrupt (1 = Open detected)
    uint8_t _SB   : 1;  // D1: Short Interrupt (1 = Short detected)
    uint8_t ABM1 : 1;  // D2: Auto Breath Mode 1 Finished
    uint8_t ABM2 : 1;  // D3: Auto Breath Mode 2 Finished
    uint8_t ABM3 : 1;  // D4: Auto Breath Mode 3 Finished
    uint8_t _reserved : 3; // D5:D7 (Reserved, must be 0)
} is31fl3737_isr;

typedef struct {
    uint8_t IO  : 1;  // D0: Dot Open Interrupt Enable (1 = Enable, 0 = Disable)
    uint8_t IS  : 1;  // D1: Dot Short Interrupt Enable (1 = Enable, 0 = Disable)
    uint8_t IAB : 1;  // D2: Auto Breath Interrupt Enable (1 = Enable, 0 = Disable)
    uint8_t IAC : 1;  // D3: Auto Clear Interrupt Enable (1 = Enable, 0 = Disable)
    uint8_t _reserved : 4; // D4:D7 (Reserved, must be 0)
} is31fl3737_imr;


typedef struct __attribute__((packed)) {
    uint8_t address;
    i2c_transfer tranfer;  // Use i2c_transfer directly
    led_sw led_ctrl_reg;
    cfg_reg_t config_reg;
    is31fl3737_imr int_msk;
    is31fl3737_isr int_stat;
} is31fl3737_def_t;

typedef enum{
    LED_CTRL_REG = 0x00,
    LED_PWM_REG = 0x01,
    AUTO_BREATH_MODE_REG = 0x02,
    FUNC_REG = 0x03,
}is31fl3737_cmd_reg;

// Function Register Enum (Explicit Values)
typedef enum {
    CONFIG_REGISTER = 0x00,
    GGC_REGISTER = 0x01,

    ABM_1_FADE_IN = 0x02,
    ABM_1_FADE_OUT = 0x03,
    ABM_1_LOOP = 0x04,
    ABM_1_LOOP_1 = 0x05,

    ABM_2_FADE_IN = 0x06,
    ABM_2_FADE_OUT = 0x07,
    ABM_2_LOOP = 0x08,
    ABM_2_LOOP_1 = 0x09,

    ABM_3_FADE_IN = 0x0A,
    ABM_3_FADE_OUT = 0x0B,
    ABM_3_LOOP = 0x0C,
    ABM_3_LOOP_1 = 0x0D,

    TIME_UPDATE_REG = 0x0E,
    SWy_PUP_SEL_REG = 0x0F,
    CSx_PDWN_SEL_REG = 0x10,
    RESET_REG = 0x11
} is31fl3737_fn_reg;

// Timing Control Enums
typedef enum {
    T1_0_21s = 0b000,
    T1_0_42s = 0b001,
    T1_0_84s = 0b010,
    T1_1_68s = 0b011,
    T1_3_36s = 0b100,
    T1_6_72s = 0b101,
    T1_13_44s = 0b110,
    T1_26_88s = 0b111
} is31fl3737_t1_time;

typedef enum {
    T2_0s = 0b0000,
    T2_0_21s = 0b0001,
    T2_0_42s = 0b0010,
    T2_0_84s = 0b0011,
    T2_1_68s = 0b0100,
    T2_3_36s = 0b0101,
    T2_6_72s = 0b0110,
    T2_13_44s = 0b0111,
    T2_26_88s = 0b1000
} is31fl3737_t2_time;

// Pull-up Resistor Configuration Enum
typedef enum {
    PUR_no_pull = 0,
    PUR_0_5k = 0b001,
    PUR_1_0k = 0b010,
    PUR_2_0k = 0b011,
    PUR_4_0k = 0b100,
    PUR_8_0k = 0b101,
    PUR_16k = 0b110,
    PUR_32k = 0b111,
} is31fl3737_PUR;

// Pull-down Resistor Configuration Enum
typedef enum {
    PDR_no_pull = 0,
    PDR_0_5k = 0b001,
    PDR_1_0k = 0b010,
    PDR_2_0k = 0b011,
    PDR_4_0k = 0b100,
    PDR_8_0k = 0b101,
    PDR_16k = 0b110,
    PDR_32k = 0b111,
} is31fl3737_PDR;

// Initialization sequence steps
typedef enum {
    IS31FL_INIT_UNLOCK = 0,
    IS31FL_INIT_CONFIG,
    IS31FL_INIT_LED_CTRL,
    IS31FL_INIT_PWM,
    IS31FL_INIT_INTERRUPTS,
    IS31FL_INIT_COMPLETE
} is31fl_init_step_t;

typedef struct {
    bool state[12][12];  // 12x12 matrix of LED states
    uint8_t row;         // Current row being checked
    uint8_t col;         // Current column being checked
} led_matrix_state;

bool Init_IS31FL3737(void);

bool disableCMD_Lock(void);
bool isCMD_LockEnabled(void);
void IS31FL3737_STB_Enable(void);
void IS31FL3737_STB_Exit(void);

bool set_LED_state(led_sw led_matrix_state);
bool enableLED_Matrix(led_sw led_matrix_state);
bool disableLED_Matrix(void);
bool checkLEDopenstate(void);
bool checkLEDshortstate(void);

bool configABM(void);
bool setDeviceInterrupts(is31fl3737_imr);


// LED Matrix State functions
bool getLEDmatrixState(led_matrix_state *open_matrix, led_matrix_state *short_matrix);
bool isLEDopen(uint8_t row, uint8_t col);
bool isLEDshorted(uint8_t row, uint8_t col);

// LED Control functions
bool setLEDPWM(uint8_t row, uint8_t col, uint8_t pwm_value);
bool setAllLEDsPWM(uint8_t pwm_value);
bool getLEDPWM(uint8_t row, uint8_t col, uint8_t *pwm_value);

// Global Control functions
bool setGlobalCurrent(uint8_t gcc_value);
bool setPullUpDownResistors(is31fl3737_PUR pur_value, is31fl3737_PDR pdr_value);

// Interrupt handling
bool clearInterruptFlags(void);
bool getInterruptStatus(is31fl3737_isr *status);

// Function declarations
bool is31fl_write_reg(uint8_t page, uint8_t reg, uint8_t data);
bool is31fl_read_reg(uint8_t page, uint8_t reg, uint8_t *data);
bool is31fl_write_page(uint8_t page, uint8_t start_reg, uint8_t *data, uint8_t len);


#endif
