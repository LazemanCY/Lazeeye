#include "Arduino.h"
#include "config.h"
#include <math.h>
#include "GPS.h"
#include "Serial.h"

static float  dTnav;            // Delta Time in milliseconds for navigation computations, updated with every good GPS read
static int16_t actual_speed[2] = {0,0};
static float GPS_scaleLonDown; // this is used to offset the shrinking longitude as we go towards the poles
static int32_t error[2];
uint16_t GPS_ground_course = 0;   

uint8_t GPS_numSat = 0; 
uint16_t GPS_speed;  
int16_t GPS_altitude;   

#define _X 1
#define _Y 0

#define RADX100                    0.000174532925  

static int32_t  GPS_coord_lead[2];              // Lead filtered gps coordinates

int32_t  GPS_coord[2];

//Home coordinate values
int32_t  Home_coord_lon;
int32_t  Home_coord_lan;
int16_t  Home_alt;

void Set_Home_Coordinate(void)
{
  Home_coord_lan = GPS_coord[LAT];
  Home_coord_lon = GPS_coord[LON];
  Home_alt = GPS_altitude;
}

/*
void GPS_calc_longitude_scaling(int32_t lat) {
  GPS_scaleLonDown = cos(lat * 1.0e-7f * 0.01745329251f);
}
*/
void GPS_calc_longitude_scaling(void) {
  GPS_scaleLonDown = cos(GPS_coord[LAT] * 1.0e-7f * 0.01745329251f);
}

/*
 * EOS increased the precision here, even if we think that the gps is not precise enough, with 10e5 precision it has 76cm resolution
 * with 10e7 it's around 1 cm now. Increasing it further is irrelevant, since even 1cm resolution is unrealistic, however increased 
 * resolution also increased precision of nav calculations
*/

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
      if(mult==0)   break;
      else  src[i+mult] = 0;
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

uint32_t Get_Target_distance_hor(void){
  uint32_t dist_cm;
  float dLat = (float)(GPS_coord[LAT] - Home_coord_lan);                                
  float dLon = (float)(GPS_coord[LON] - Home_coord_lon) * GPS_scaleLonDown; 
  dist_cm = sqrt(sq(dLat) + sq(dLon)) * 1.11318845f;
  return (uint32_t)(dist_cm/100);
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
    actual_speed[_X] = (float)(GPS_coord[LON] - last[LON]) *  GPS_scaleLonDown * tmp;
    actual_speed[_Y] = (float)(GPS_coord[LAT]  - last[LAT])  * tmp;

    //TODO: Check unrealistic speed changes and signal navigation about posibble gps signal degradation
    if (!GPS_conf_lead_filter) {
      actual_speed[_X] = (actual_speed[_X] + speed_old[_X]) / 2;
      actual_speed[_Y] = (actual_speed[_Y] + speed_old[_Y]) / 2;

      speed_old[_X] = actual_speed[_X];
      speed_old[_Y] = actual_speed[_Y];
    }
  }
  init=1;

  last[LON] = GPS_coord[LON];
  last[LAT] = GPS_coord[LAT];

  /*
  if (GPS_conf_lead_filter) {
    GPS_coord_lead[LON] = xLeadFilter.get_position(GPS_coord[LON], actual_speed[_X], GPS_LAG);
    GPS_coord_lead[LAT] = yLeadFilter.get_position(GPS_coord[LAT], actual_speed[_Y], GPS_LAG);
  }*/
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

#if defined(NMEA)
/* This is a light implementation of a GPS frame decoding
   This should work with most of modern GPS devices configured to output NMEA frames.
   It assumes there are some NMEA GGA frames to decode on the serial bus
   Here we use only the following data :
     - latitude
     - longitude
     - GPS fix is/is not ok
     - GPS num sat (4 is enough to be +/- reliable)
     - GPS altitude
     - GPS speed
*/
#define FRAME_GGA  1
#define FRAME_RMC  2

void GPS_SerialInit(void) {
  SerialOpen(GPS_SERIAL,GPS_BAUD);
  //delay(500);
}

bool GPS_newFrame(uint8_t c) {
  uint8_t frameOK = 0;
  static uint8_t param = 0, offset = 0, parity = 0;
  static char string[15];
  static uint8_t checksum_param, frame = 0;

  if (c == '$') {
    param = 0; offset = 0; parity = 0;
  } else if (c == ',' || c == '*') {
    string[offset] = 0;
    if (param == 0) { //frame identification
      frame = 0;
      if (string[0] == 'G' && ( string[1] == 'N' || string[1] == 'P' ) && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') frame = FRAME_GGA;
      if (string[0] == 'G' && ( string[1] == 'N' || string[1] == 'P' ) && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') frame = FRAME_RMC;
    } else if (frame == FRAME_GGA) {
      if      (param == 2)                     {GPS_coord[LAT] = GPS_coord_to_degrees(string);}
      else if (param == 3 && string[0] == 'S') GPS_coord[LAT] = -GPS_coord[LAT];
      else if (param == 4)                     {GPS_coord[LON] = GPS_coord_to_degrees(string);}
      else if (param == 5 && string[0] == 'W') GPS_coord[LON] = -GPS_coord[LON];
      else if (param == 6)                     {GPS_FIX = (string[0]  > '0');}
      else if (param == 7)                     {GPS_numSat = grab_fields(string,0);}
      else if (param == 9)                     {GPS_altitude = grab_fields(string,0);}  // altitude in meters added by Mis
    } else if (frame == FRAME_RMC) {
      if      (param == 7)                     {GPS_speed = ((uint32_t)grab_fields(string,1)*5144L)/1000L;}  //gps speed in cm/s will be used for navigation
      else if (param == 8)                     {GPS_ground_course = grab_fields(string,1); }                 //ground course deg*10 
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
#endif //NMEA
