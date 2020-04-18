#ifndef __CONFIG_H
#define __CONFIG_H

//#define MOTOR_DC
#define MOTOR_ESC
#define MOTOR_ESC_OneShot125 //0%    2250, 125  us
                             //50%   3375, 187.5us
                             //100%  4500, 250  us

//#define IMU_NORMAL
#define IMU_AHRS

#define HEADFREE

//#define Recive_PID_CHANGE

//#define debug

//#define MOTOR_Calibration

#define QUAD_X

//#define QUAD_P

//#define MAG_cal

#define GPS_Recive  //USART1, BaudRate : 57600
#define DEVO7_Recive
//#define BLE_Recive   // USART1, BaudRate : 115200
#define Telemetry    // USART2, BaudRate : 57600
//#define SSD1306


  /**********************************  constant loop time  ******************************/
    #define LOOP_TIME 4000

//    #define VOLTAGEDROP_COMPENSATION
    #define VBATLEVEL_WARN1 35
    #define VBATLEVEL_WARN2 33
    #define VBATLEVEL_CRIT 30
    #define VBATNOMINAL 42
    #define VBAT_SMOOTH 8              // len of averaging vector for smoothing the VBAT readings; should be power of 2; set to 1 to disable

    #define HEADING_SMOOTH 10

#endif
