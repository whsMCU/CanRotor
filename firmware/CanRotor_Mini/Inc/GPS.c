/*
 * GPS.c
 *
 *  Created on: 2019. 1. 23.
 *      Author: WANG
 */

//#include "GPS.h"
#include "Board.h"

const unsigned char Disable_GPGSV[] = {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15
};

const unsigned char Set_to_5Hz[] = {
      0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A
};

const unsigned char Set_to_57kbps[] = {
      0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
      0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1
};

gps_t GPS;

int16_t  GPS_angle[2] = { 0, 0};                      // the angles that must be applied for GPS correction
int32_t  GPS_coord[2];
int32_t  GPS_home[2];
int32_t  GPS_hold[2];
int32_t  GPS_prev[2];                                 //previous pos
int32_t  GPS_poi[2];
uint8_t  GPS_numSat;
uint16_t GPS_distanceToHome;                          // distance to home  - unit: meter
int16_t  GPS_directionToHome;                         // direction to home - unit: degree
int32_t  GPS_directionToPoi;
uint16_t GPS_altitude;                                // GPS altitude      - unit: meter
uint16_t GPS_speed;                                   // GPS speed         - unit: cm/s
uint8_t  GPS_update = 0;                              // a binary toogle to distinct a GPS position update
uint16_t GPS_ground_course = 0;                       //                   - unit: degree*10

typedef struct PID_PARAM_ {
  float kP;
  float kI;
  float kD;
  float Imax;
  } PID_PARAM;

PID_PARAM posholdPID_PARAM;
PID_PARAM poshold_ratePID_PARAM;
PID_PARAM navPID_PARAM;

typedef struct PID_ {
  float   integrator; // integrator value
  int32_t last_input; // last input for derivative
  float   lastderivative; // last derivative for low-pass filter
  float   output;
  float   derivative;
} PID;
PID posholdPID[2];
PID poshold_ratePID[2];
PID navPID[2];

int32_t get_P(int32_t error, struct PID_PARAM_* pid) {
  return (float)error * pid->kP;
}

int32_t get_I(int32_t error, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param) {
  pid->integrator += ((float)error * pid_param->kI) * *dt;
  pid->integrator = constrain(pid->integrator,-pid_param->Imax,pid_param->Imax);
  return pid->integrator;
}

int32_t get_D(int32_t input, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param) { // dt in milliseconds
  pid->derivative = (input - pid->last_input) / *dt;

  /// Low pass filter cut frequency for derivative calculation.
  float filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";
  // Examples for _filter:
  // f_cut = 10 Hz -> _filter = 15.9155e-3
  // f_cut = 15 Hz -> _filter = 10.6103e-3
  // f_cut = 20 Hz -> _filter =  7.9577e-3
  // f_cut = 25 Hz -> _filter =  6.3662e-3
  // f_cut = 30 Hz -> _filter =  5.3052e-3

  // discrete low pass filter, cuts out the
  // high frequency noise that can drive the controller crazy
  pid->derivative = pid->lastderivative + (*dt / ( filter + *dt)) * (pid->derivative - pid->lastderivative);
  // update state
  pid->last_input = input;
  pid->lastderivative    = pid->derivative;
  // add in derivative component
  return pid_param->kD * pid->derivative;
}

void reset_PID(struct PID_* pid) {
  pid->integrator = 0;
  pid->last_input = 0;
  pid->lastderivative = 0;
}

//uint8_t GPS_mode  = GPS_MODE_NONE; // contains the current selected gps flight mode --> moved to the f. structure
uint8_t NAV_state = 0; // NAV_STATE_NONE;  /// State of the nav engine
uint8_t NAV_error = 0; // NAV_ERROR_NONE;
uint8_t prv_gps_modes = 0;              /// GPS_checkbox items packed into 1 byte for checking GPS mode changes

// The desired bank towards North (Positive) or South (Negative) : latitude
// The desired bank towards East (Positive) or West (Negative)   : longitude
int16_t  nav[2];
int16_t  nav_rated[2];    //Adding a rate controller to the navigation to make it smoother

uint8_t land_detect;                 //Detect land (extern)
static uint32_t land_settle_timer;

static float  dTnav;            // Delta Time in milliseconds for navigation computations, updated with every good GPS read
static int16_t actual_speed[2] = {0,0};
static float GPS_scaleLonDown; // this is used to offset the shrinking longitude as we go towards the poles

// The difference between the desired rate of travel and the actual rate of travel
// updated after GPS read - 5-10hz
static int16_t rate_error[2];
static int32_t error[2];

static int32_t GPS_WP[2];   //Currently used WP
static int32_t GPS_FROM[2]; //the pervious waypoint for precise track following
int32_t target_bearing;     // This is the angle from the copter to the "next_WP" location in degrees * 100
static int32_t original_target_bearing;  // deg * 100, The original angle to the next_WP when the next_WP was set, Also used to check when we pass a WP
uint32_t wp_distance;                // distance between plane and next_WP in cm
static uint16_t waypoint_speed_gov;  // used for slow speed wind up when start navigation;

////////////////////////////////////////////////////////////////////////////////////
// moving average filter variables
//

#define GPS_FILTER_VECTOR_LENGTH 5

static uint8_t GPS_filter_index = 0;
static int32_t GPS_filter[2][GPS_FILTER_VECTOR_LENGTH];
static int32_t GPS_filter_sum[2];
static int32_t GPS_read[2];
static int32_t GPS_filtered[2];
static int32_t GPS_degree[2];    //the lat lon degree without any decimals (lat/10 000 000)
static uint16_t fraction3[2];

static int16_t nav_takeoff_bearing;  // saves the bearing at takeof (1deg = 1) used to rotate to takeoff direction when arrives at home

void USART2_TX(unsigned char data){while(!(USART2->SR&0x40)); USART2->DR=data;}
void USART2_TX_str(char *str){while(*str){USART2_TX(*str++);}}

void gps_Init(void)
{
  huart1.Init.BaudRate = 9600;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_UART_Transmit_IT(&huart1, (uint8_t*)Disable_GPGSV, 11);
  HAL_Delay(350);
  HAL_UART_Transmit_IT(&huart1, (uint8_t*)Set_to_5Hz, 14);
  HAL_Delay(350);
  HAL_UART_Transmit_IT(&huart1, (uint8_t*)Set_to_57kbps, 28);
  HAL_Delay(350);
  huart1.Init.BaudRate = 57600;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

////////////////////////////////////////////////////////////////////////////////////
//PID based GPS navigation functions
//Author : EOSBandi
//Based on code and ideas from the Arducopter team: Jason Short,Randy Mackay, Pat Hickey, Jose Julio, Jani Hirvinen
//Andrew Tridgell, Justin Beech, Adam Rivera, Jean-Louis Naudin, Roberto Navoni

//original constraint does not work with variables
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high) {
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

void GPS_mode_check(void){
  if(f.ARMED){
    if(GPS.fixquality){
      if(GPS.satellites > 5){
         if(f.GPS_HOLD_MODE){
           f.GPS_MODE = GPS_MODE_HOLD;
           GPS_set_next_wp(&GPS_coord[LAT], &GPS_coord[LON], &GPS_coord[LAT], &GPS_coord[LON]); //hold at the current position
           //set_new_altitude(alt.EstAlt);                                //and current altitude
           NAV_state = NAV_STATE_HOLD_INFINIT;
         }
      }else{
        if (f.GPS_MODE == GPS_MODE_HOLD || f.GPS_MODE == GPS_MODE_RTH) {
          f.GPS_MODE = GPS_MODE_NONE;
          NAV_state = NAV_STATE_NONE;
          NAV_error = NAV_ERROR_SPOILED_GPS;
          //prv_gps_modes = 0xff;                                          //invalidates mode check, to allow re evaluate rcOptions when numsats raised again
        }
        nav[0] = 0; nav[1] = 0;
      }
    }else{
      // GPS Fix dissapeared, very unlikely that we will be able to regain it, abort mission
      f.GPS_MODE = GPS_MODE_NONE;
      NAV_state = NAV_STATE_NONE;
      //NAV_paused_at = 0;
      NAV_error = NAV_ERROR_GPS_FIX_LOST;
      //GPS_reset_nav();
      //prv_gps_modes = 0xff;                                              //Gives a chance to restart mission when regain fix
    }
  }else{
    //copter is disarmed
    f.GPS_MODE = GPS_MODE_NONE;
    //f.GPS_BARO_MODE = false;
    //f.THROTTLE_IGNORED = false;
    NAV_state = NAV_STATE_NONE;
    //NAV_paused_at = 0;
    NAV_error = NAV_ERROR_DISARMED;
    //GPS_reset_nav();
  }
}

uint8_t GPS_Compute(void) {
  unsigned char axis;
  uint32_t dist;        //temp variable to store dist to copter
  int32_t  dir;         //temp variable to store dir to copter
  static uint32_t nav_loopTimer;

  //check that we have a valid frame, if not then return immediatly
//  if (GPS.GPS_Frame == 0) return 0; else GPS.GPS_Frame = 0;

  //Apply moving average filter to GPS data
  if (1) {
    GPS_filter_index = (GPS_filter_index+1) % GPS_FILTER_VECTOR_LENGTH;
    for (axis = 0; axis< 2; axis++) {
      GPS_read[axis] = GPS.GPS_coord[axis]; //latest unfiltered data is in GPS_latitude and GPS_longitude
      GPS_degree[axis] = GPS_read[axis] / 10000000;  // get the degree to assure the sum fits to the int32_t

      // How close we are to a degree line ? its the first three digits from the fractions of degree
      // later we use it to Check if we are close to a degree line, if yes, disable averaging,
      fraction3[axis] = (GPS_read[axis]- GPS_degree[axis]*10000000) / 10000;

      GPS_filter_sum[axis] -= GPS_filter[axis][GPS_filter_index];
      GPS_filter[axis][GPS_filter_index] = GPS_read[axis] - (GPS_degree[axis]*10000000);
      GPS_filter_sum[axis] += GPS_filter[axis][GPS_filter_index];
      GPS_filtered[axis] = GPS_filter_sum[axis] / GPS_FILTER_VECTOR_LENGTH + (GPS_degree[axis]*10000000);
      if ( NAV_state == NAV_STATE_HOLD_INFINIT || NAV_state == NAV_STATE_HOLD_TIMED) {      //we use gps averaging only in poshold mode...
        if ( fraction3[axis]>1 && fraction3[axis]<999 ) GPS.GPS_coord[axis] = GPS_filtered[axis];
      }
    }
  }

  //dTnav calculation
  //Time for calculating x,y speed and navigation pids
  dTnav = (float)(millis() - nav_loopTimer)/ 1000.0;
  nav_loopTimer = millis();

  // prevent runup from bad GPS
  dTnav = min(dTnav, 1.0);

  //calculate distance and bearings for gui and other stuff continously - From home to copter
  GPS_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_home[LAT],&GPS_home[LON],&dir);
  GPS_distance_cm(&GPS_coord[LAT],&GPS_coord[LON],&GPS_home[LAT],&GPS_home[LON],&dist);
  GPS_distanceToHome = dist/100;
  GPS_directionToHome = dir/100;

  if (!GPS.fixquality) {     //If we don't have home set, do not display anything
    GPS_distanceToHome = 0;
    GPS_directionToHome = 0;
  }

  //calculate the current velocity based on gps coordinates continously to get a valid speed at the moment when we start navigating
  GPS_calc_velocity();

  if(f.GPS_MODE != GPS_MODE_NONE){
    //do gps nav calculations here, these are common for nav and poshold
    GPS_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_WP[LAT],&GPS_WP[LON],&target_bearing);
    GPS_distance_cm(&GPS_coord[LAT],&GPS_coord[LON],&GPS_WP[LAT],&GPS_WP[LON],&wp_distance);
    GPS_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_coord[LAT],&GPS_coord[LON]);


    switch(NAV_state){
      case NAV_STATE_HOLD_INFINIT:
        GPS_calc_poshold();
        break;
      default:
        break;
    }
  }

  return 1;
}

////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm
// Get bearing from pos1 to pos2, returns an 1deg = 100 precision

void GPS_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2, int32_t* bearing) {
  int32_t off_x = *lon2 - *lon1;
  int32_t off_y = (*lat2 - *lat1) / GPS_scaleLonDown;
  *bearing = 9000 + atan2(-off_y, off_x) * 5729.57795f;      //Convert the output redians to 100xdeg
  if (*bearing < 0) *bearing += 36000;
}

void GPS_distance_cm(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist) {
  float dLat = (float)(*lat2 - *lat1);                                    // difference of latitude in 1/10 000 000 degrees
  float dLon = (float)(*lon2 - *lon1) * GPS_scaleLonDown; //x
  *dist = sqrt(sq(dLat) + sq(dLon)) * 1.11318845f;
}

//*******************************************************************************************************
// calc_velocity_and_filtered_position - velocity in lon and lat directions calculated from GPS position
//       and accelerometer data
// lon_speed expressed in cm/s.  positive numbers mean moving east
// lat_speed expressed in cm/s.  positive numbers when moving north
// Note: we use gps locations directly to calculate velocity instead of asking gps for velocity because
//       this is more accurate below 1.5m/s
// Note: even though the positions are projected using a lead filter, the velocities are calculated
//       from the unaltered gps locations.  We do not want noise from our lead filter affecting velocity
//*******************************************************************************************************
static void GPS_calc_velocity(void){
  static int16_t speed_old[2] = {0,0};
  static int32_t last[2] = {0,0};
  static uint8_t init = 0;

  if (init) {
    float tmp = 1.0/dTnav;
    actual_speed[__X] = (float)(GPS.GPS_coord[LON] - last[LON]) *  GPS_scaleLonDown * tmp;
    actual_speed[__Y] = (float)(GPS.GPS_coord[LAT]  - last[LAT])  * tmp;

//    //TODO: Check unrealistic speed changes and signal navigation about posibble gps signal degradation
//    if (!GPS_conf.lead_filter) {
//      actual_speed[_X] = (actual_speed[_X] + speed_old[_X]) / 2;
//      actual_speed[_Y] = (actual_speed[_Y] + speed_old[_Y]) / 2;
//
//      speed_old[_X] = actual_speed[_X];
//      speed_old[_Y] = actual_speed[_Y];
//    }
  }
  init=1;

  last[LON] = GPS.GPS_coord[LON];
  last[LAT] = GPS.GPS_coord[LAT];

//  if (GPS_conf.lead_filter) {
//    GPS_coord_lead[LON] = xLeadFilter.get_position(GPS_coord[LON], actual_speed[_X], GPS_LAG);
//    GPS_coord_lead[LAT] = yLeadFilter.get_position(GPS_coord[LAT], actual_speed[_Y], GPS_LAG);
//  }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate a location error between two gps coordinates
// Because we are using lat and lon to do our distance errors here's a quick chart:
//   100  = 1m
//  1000  = 11m    = 36 feet
//  1800  = 19.80m = 60 feet
//  3000  = 33m
// 10000  = 111m
//
static void GPS_calc_location_error( int32_t* target_lat, int32_t* target_lng, int32_t* gps_lat, int32_t* gps_lng ) {
  error[LON] = (float)(*target_lng - *gps_lng) * GPS_scaleLonDown;  // X Error
  error[LAT] = *target_lat - *gps_lat; // Y Error
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate nav_lat and nav_lon from the x and y error and the speed
//
// TODO: check that the poshold target speed constraint can be increased for snappier poshold lock
static void GPS_calc_poshold(void) {
  int32_t d;
  int32_t target_speed;
  uint8_t axis;

  for (axis=0;axis<2;axis++) {
    target_speed = get_P(error[axis], &posholdPID_PARAM); // calculate desired speed from lat/lon error
    target_speed = constrain(target_speed,-100,100);      // Constrain the target speed in poshold mode to 1m/s it helps avoid runaways..
    rate_error[axis] = target_speed - actual_speed[axis]; // calc the speed error

    nav[axis]      =
        get_P(rate_error[axis],                                               &poshold_ratePID_PARAM)
       +get_I(rate_error[axis] + error[axis], &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);

    d = get_D(error[axis],                    &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);

    d = constrain(d, -2000, 2000);

    // get rid of noise
    if(abs(actual_speed[axis]) < 50) d = 0;

    nav[axis] +=d;
    // nav[axis]  = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
    nav[axis]  = constrain_int16(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
    navPID[axis].integrator = poshold_ratePID[axis].integrator;
  }
}

////////////////////////////////////////////////////////////////////////////////////
// this is used to offset the shrinking longitude as we go towards the poles
// It's ok to calculate this once per waypoint setting, since it changes a little within the reach of a multicopter
//
void GPS_calc_longitude_scaling(int32_t lat) {
  GPS_scaleLonDown = cos(lat * 1.0e-7f * 0.01745329251f);
}

////////////////////////////////////////////////////////////////////////////////////
// Sets the waypoint to navigate, reset neccessary variables and calculate initial values
//
void GPS_set_next_wp(int32_t* lat_to, int32_t* lon_to, int32_t* lat_from, int32_t* lon_from) {
  GPS_WP[LAT] = *lat_to;
  GPS_WP[LON] = *lon_to;

  GPS_FROM[LAT] = *lat_from;
  GPS_FROM[LON] = *lon_from;

  GPS_calc_longitude_scaling(*lat_to);

  GPS_bearing(&GPS_FROM[LAT],&GPS_FROM[LON],&GPS_WP[LAT],&GPS_WP[LON],&target_bearing);
  GPS_distance_cm(&GPS_FROM[LAT],&GPS_FROM[LON],&GPS_WP[LAT],&GPS_WP[LON],&wp_distance);
  GPS_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_FROM[LAT],&GPS_FROM[LON]);
  waypoint_speed_gov = 100;
  original_target_bearing = target_bearing;

}

void GPS_reset_home_position(void) {
  if (GPS.fixquality && GPS.satellites >= 5) {
    GPS_home[LAT] = GPS_coord[LAT];
    GPS_home[LON] = GPS_coord[LON];
    GPS_calc_longitude_scaling(GPS_coord[LAT]);    //need an initial value for distance and bearing calc
    nav_takeoff_bearing = imu.actual_compass_heading;             //save takeoff heading
    //TODO: Set ground altitude
    f.GPS_FIX_HOME = 1;
  }
}

#define FRAME_GGA  1
#define FRAME_RMC  2

bool GPS_newFrame(uint8_t c) {
  uint32_t time = 0;
  float timef = 0;
  uint8_t frameOK = 0;
  static uint8_t param = 0, offset = 0, parity = 0;
  static char string[15];
  static uint8_t checksum_param, frame = 0;
  //TX_CHR(c);

  if (c == '$') {
    param = 0; offset = 0; parity = 0;
  } else if (c == ',' || c == '*') {
    string[offset] = 0;
    if (param == 0) { //frame identification
      frame = 0;
      if (string[0] == 'G' && string[1] == 'N' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') frame = FRAME_GGA;
      if (string[0] == 'G' && string[1] == 'N' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') frame = FRAME_RMC;
    } else if (frame == FRAME_GGA) {
      RGB_B_TOGGLE;
      if (param == 1){
        timef = atof(string);
        //time = grab_fields(string_test,0);
        time = (uint32_t) timef;
        GPS.hour = (time / 10000)+TD;
        GPS.minute = (time % 10000) / 100;
        GPS.seconds = (time % 100);
        GPS.milliseconds = fmod(timef, 1.0) * 1000;
      }
      //HAL_UART_Transmit_IT(&huart1, (uint8_t*)string, strlen(string));
      else if (param == 2)                     {GPS.GPS_coord[LAT] = GPS_coord_to_degrees(string);GPS_coord[LAT]=GPS.GPS_coord[LAT];}
      else if (param == 3 && string[0] == 'S') {GPS.GPS_coord[LAT] = -GPS.GPS_coord[LAT]; GPS_coord[LAT]=GPS.GPS_coord[LAT];}
      else if (param == 4)                     {GPS.GPS_coord[LON] = GPS_coord_to_degrees(string); GPS_coord[LON]=GPS.GPS_coord[LON];}
      else if (param == 5 && string[0] == 'W') {GPS.GPS_coord[LON] = -GPS.GPS_coord[LON]; GPS_coord[LON]=GPS.GPS_coord[LON];}
      else if (param == 6)                     {GPS.fixquality = (string[0]  > '0');}
      else if (param == 7)                     {GPS.satellites = grab_fields(string,0);}
      else if (param == 9)                     {GPS.altitude = grab_fields(string,0);}  // altitude in meters added by Mis
    } else if (frame == FRAME_RMC) {
      if      (param == 7)                     {GPS.speed = ((uint32_t)grab_fields(string,1)*5144L)/1000L;}  //gps speed in cm/s will be used for navigation
      else if (param == 8)                     {GPS.GPS_ground_course = grab_fields(string,1); }             //ground course deg*10
    }
    param++; offset = 0;
    if (c == '*') checksum_param=1;
    else parity ^= c;
  } else if (c == '\r' || c == '\n') {
    if (checksum_param) { //parity checksum
      uint8_t checksum = hex_c(string[0]);
      checksum <<= 4;
      checksum += hex_c(string[1]);
      if (checksum == parity) frameOK = 1;
    }
    checksum_param=0;
  } else {
     if (offset < 15) string[offset++] = c;
     if (!checksum_param) parity ^= c;
  }
  return frameOK && (frame==FRAME_GGA);
}

// read a Hex value and return the decimal equivalent
uint8_t parseHex(char c) {
    if (c < '0')
      return 0;
    if (c <= '9')
      return c - '0';
    if (c < 'A')
       return 0;
    if (c <= 'F')
       return (c - 'A')+10;
    // if (c > 'F')
    return 0;
}

#define DIGIT_TO_VAL(_x)        (_x - '0')
uint32_t GPS_coord_to_degrees(char* s) {
  char *p, *q;
  uint8_t deg = 0, min = 0;
  unsigned int frac_min = 0;
  uint8_t i;

  // scan for decimal point or end of field
  for (p = s; isdigit(*p); p++) ;
  q = s;

  // convert degrees
  while ((p - q) > 2) {
    if (deg)
      deg *= 10;
    deg += DIGIT_TO_VAL(*q++);
  }
  // convert minutes
  while (p > q) {
    if (min)
      min *= 10;
    min += DIGIT_TO_VAL(*q++);
  }
  // convert fractional minutes
  // expect up to four digits, result is in
  // ten-thousandths of a minute
  if (*p == '.') {
    q = p + 1;
    for (i = 0; i < 4; i++) {
      frac_min *= 10;
      if (isdigit(*q))
        frac_min += *q++ - '0';
    }
  }
  return deg * 10000000UL + (min * 1000000UL + frac_min*100UL) / 6;
}

// helper functions
uint16_t grab_fields(char* src, uint8_t mult) {  // convert string to uint16
  uint8_t i;
  uint16_t tmp = 0;

  for(i=0; src[i]!=0; i++) {
    if(src[i] == '.') {
      i++;
      if(mult==0){
        break;
      }else{
        src[i+mult] = 0;
      }
    }
    tmp *= 10;
    if(src[i] >='0' && src[i] <='9') tmp += src[i]-'0';
  }
  return tmp;
}

uint8_t hex_c(uint8_t n) {    // convert '0'..'9','A'..'F' to 0..15
  n -= '0';
  if(n>9)  n -= 7;
  n &= 0x0F;
  return n;
}

//Get the relevant P I D values and set the PID controllers
void GPS_set_pids(void) {
  posholdPID_PARAM.kP   = 0;//(float)conf.pid[PIDPOS].P8/100.0;
  posholdPID_PARAM.kI   = 0;//(float)conf.pid[PIDPOS].I8/100.0;
  posholdPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;

  poshold_ratePID_PARAM.kP   = 0;//(float)conf.pid[PIDPOSR].P8/10.0;
  poshold_ratePID_PARAM.kI   = 0;//(float)conf.pid[PIDPOSR].I8/100.0;
  poshold_ratePID_PARAM.kD   = 0;//(float)conf.pid[PIDPOSR].D8/1000.0;
  poshold_ratePID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;

  navPID_PARAM.kP   = 0;//(float)conf.pid[PIDNAVR].P8/10.0;
  navPID_PARAM.kI   = 0;//(float)conf.pid[PIDNAVR].I8/100.0;
  navPID_PARAM.kD   = 0;//(float)conf.pid[PIDNAVR].D8/1000.0;
  navPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;
  }
