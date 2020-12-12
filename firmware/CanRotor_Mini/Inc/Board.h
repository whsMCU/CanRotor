#ifndef __BOARD_H
#define __BOARD_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
/////////////////////////////////////////
#include "config.h"
#include "EEPROM.h"
#include "fonts.h"
#include "GFX.h"
#include "GPS.h"
#include "I2C.h"
#include "IMU.h"
#include "LED_control.h"
#include "Mixer.h"
#include "PID.h"
#include "PWM.h"
#include "Queue.h"
#include "uart.h"
#include "Radio.h"
#include "Sensor.h"
#include "Serial.h"
//#include "ssd1306.h"
#include "System.h"
#include "Typedef.h"
#include "maths.h"

extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern char Buf[128];
extern int Flight_Status;
extern int Flight_Status;
extern float output[3];
extern float ITerm[3];
extern int MSP_TRIM[3];
extern eeror_t Error;
extern uint8_t rx1_buffer[1];
extern uint8_t rx2_buffer[1];
extern pidc_t pid;
extern imu_t imu;
extern Queue_t Q_buffer[UART_MAX_CH];
extern flags_t f;
extern int16_t motor[4];
extern uint8_t Manual_Motor_flag;
extern int16_t M_motor[4];
extern uint32_t time_manual_motor;
extern bat_t BAT;
extern ms5611_t ms5611;
extern gps_t GPS;
extern uint16_t calibratingA;
extern TM_AHRSIMU_t AHRS;
extern rc RC;
extern rc RC_Raw;
extern alt_t alt;
extern test_t test;
extern att_t att;

extern debug_t Debug_var;

extern PID_PARAM posholdPID_PARAM;
extern PID_PARAM poshold_ratePID_PARAM;
extern PID_PARAM navPID_PARAM;

extern int16_t  GPS_angle[2];
extern int32_t  GPS_coord[2];

extern uint8_t NAV_state;
extern uint8_t NAV_error;
extern uint8_t prv_gps_modes;

extern int16_t  nav[2];
extern int16_t  nav_rated[2];

extern uint8_t telemetry_loop_counter;
extern int16_t overrun_count;
extern uint16_t time, time1, aftertime;
extern uint16_t cycleTimeMax, cycleTimeMin;
extern uint32_t baroPressureSum;

uint32_t debug;

extern volatile uint32_t loopTime, cycleTime, currentTime;
extern uint32_t armedTime;

extern float magBias[3], magScale[3];

#ifndef M_PI
#define M_PI       3.14159265358979323846f
#endif /* M_PI */

#define D2R (3.141592653f / 180.0f)
#define R2D (180.0f / 3.141592653f)

#define TRUE 1
#define FALSE 0

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))
#define zofs(x, y, z) ((x) > (y+z) ? (x) : ((x) < (y-z) ? (x) : (y)))
#define map(x, in_min, in_max, out_min, out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

#define ROUND(x, dig)  ( floor((x) * pow((float)(10), dig) + 0.5f) / pow((float)(10), dig) )
#define applyDeadband(value, deadband)  \
  if(abs(value) < deadband) {           \
    value = 0;                          \
  } else if(value > 0){                 \
    value -= deadband;                  \
  } else if(value < 0){                 \
    value += deadband;                  \
  }

//void resetConf(void);

/* Update 09/12/16 - Resistors partition for battery voltage monitoring */
#define BAT_RUP 10      /* Pull-up resistor value [Kohm] */
#define BAT_RDW 20      /* Pull-Down resistor value [Kohm] */

//#ifdef

#define LED0_GPIO   GPIOB
#define LED0_PIN    GPIO_PIN_3 // PB3 (RED LED)
#define LED0
#define LED1_GPIO   GPIOB
#define LED1_PIN    GPIO_PIN_4 // PB4 (GREEN LED)
#define LED1

#define LED2_GPIO   GPIOB
#define LED2_PIN    GPIO_PIN_13 // PB4 (RED LED)
#define LED2

#define LED3_GPIO   GPIOB
#define LED3_PIN    GPIO_PIN_14 // PB3 (GREEN LED)
#define LED3

#define LED4_GPIO   GPIOB
#define LED4_PIN    GPIO_PIN_15 // PB3 (GREEN LED)
#define LED4

// Helpful macros
#ifdef LED0
#define LED0_TOGGLE              HAL_GPIO_TogglePin(LED0_GPIO, LED0_PIN);
#define LED0_OFF                 HAL_GPIO_WritePin(LED0_GPIO, LED0_PIN, GPIO_PIN_RESET);
#define LED0_ON                  HAL_GPIO_WritePin(LED0_GPIO, LED0_PIN, GPIO_PIN_SET);
#else
#define LED0_TOGGLE
#define LED0_OFF
#define LED0_ON
#endif

#ifdef LED1
#define LED1_TOGGLE              HAL_GPIO_TogglePin(LED1_GPIO, LED1_PIN);
#define LED1_OFF                 HAL_GPIO_WritePin(LED1_GPIO, LED1_PIN, GPIO_PIN_RESET);
#define LED1_ON                  HAL_GPIO_WritePin(LED1_GPIO, LED1_PIN, GPIO_PIN_SET);
#else
#define LED1_TOGGLE
#define LED1_OFF
#define LED1_ON
#endif

#ifdef LED2
#define RGB_R_TOGGLE              HAL_GPIO_TogglePin(LED0_GPIO, LED2_PIN);
#define RGB_R_OFF                 HAL_GPIO_WritePin(LED0_GPIO, LED2_PIN, GPIO_PIN_SET);
#define RGB_R_ON                  HAL_GPIO_WritePin(LED0_GPIO, LED2_PIN, GPIO_PIN_RESET);
#else
#define RGB_R_TOGGLE
#define RGB_R_OFF
#define RGB_R_ON
#endif

#ifdef LED3
#define RGB_G_TOGGLE              HAL_GPIO_TogglePin(LED1_GPIO, LED3_PIN);
#define RGB_G_OFF                 HAL_GPIO_WritePin(LED1_GPIO, LED3_PIN, GPIO_PIN_SET);
#define RGB_G_ON                  HAL_GPIO_WritePin(LED1_GPIO, LED3_PIN, GPIO_PIN_RESET);
#else
#define RGB_G_TOGGLE
#define RGB_G_OFF
#define RGB_G_ON
#endif

#ifdef LED4
#define RGB_B_TOGGLE              HAL_GPIO_TogglePin(LED1_GPIO, LED4_PIN);
#define RGB_B_OFF                 HAL_GPIO_WritePin(LED1_GPIO, LED4_PIN, GPIO_PIN_SET);
#define RGB_B_ON                  HAL_GPIO_WritePin(LED1_GPIO, LED4_PIN, GPIO_PIN_RESET);
#else
#define RGB_B_TOGGLE
#define RGB_B_OFF
#define RGB_B_ON
#endif

//#ifdef BEEP_GPIO
//#define BEEP_TOGGLE              digitalToggle(BEEP_GPIO, BEEP_PIN);
//#define BEEP_OFF                 systemBeep(false);
//#define BEEP_ON                  systemBeep(true);
//#else
//#define BEEP_TOGGLE              ;
//#define BEEP_OFF                 ;
//#define BEEP_ON                  ;
//#endif

extern volatile uint32_t loop_timer;

#endif

