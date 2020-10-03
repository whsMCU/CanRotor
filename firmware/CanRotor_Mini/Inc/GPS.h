/*
 * GPS.h
 *
 *  Created on: 2019. 1. 23.
 *      Author: WANG
 */

#ifndef GPS_H_
#define GPS_H_

#define TD 9 //KOREA 시차 +9시간

typedef struct gps_t {
  char GPS[120];

  uint8_t hour;
  uint8_t minute;
  uint8_t seconds;
  uint8_t year;
  uint8_t month;
  uint8_t day;

  uint16_t milliseconds;

  int32_t GPS_coord[2];

  uint16_t altitude;

  uint16_t speed;

  uint8_t GPS_update;
  uint8_t GPS_Frame;
  uint16_t GPS_ground_course;                       //                   - unit: degree*10

  uint8_t fixquality;
  uint8_t satellites;

  uint32_t error;

} gps_t;

typedef struct PID_PARAM_ {
  float kP;
  float kI;
  float kD;
  float Imax;
  } PID_PARAM;

#define LAT  0
#define LON  1

#define __X 1
#define __Y 0

#define RADX100                    0.000174532925

// Maximum allowable banking than navigation outputs
#define NAV_BANK_MAX 3000                 //(**)

#define sq(x) ((x)*(x))

void USART2_TX(unsigned char data);

void USART2_TX_str(char *str);

void gps_Init(void);
void GPS_set_pids(void);
void GPS_mode_check(void);
uint8_t GPS_Compute(void);
void GPS_reset_home_position(void);
void GPS_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2, int32_t* bearing);
static void GPS_distance_cm(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist);
static void GPS_calc_velocity(void);
static void GPS_calc_location_error( int32_t* target_lat, int32_t* target_lng, int32_t* gps_lat, int32_t* gps_lng );
static void GPS_calc_poshold(void);
void GPS_calc_longitude_scaling(int32_t lat);
void GPS_set_next_wp(int32_t* lat_to, int32_t* lon_to, int32_t* lat_from, int32_t* lon_from);

//Function prototypes for GPS frame parsing
bool GPS_newFrame(uint8_t c);
uint32_t GPS_coord_to_degrees(char* s);
uint16_t grab_fields(char* src, uint8_t mult);
uint8_t hex_c(uint8_t n);

uint8_t parseHex(char c);

#endif /* GPS_H_ */
