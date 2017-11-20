#ifndef GPS_H_
#define GPS_H_

#define LAT  0
#define LON  1

#define DIGIT_TO_VAL(_x)        (_x - '0')

bool GPS_newFrame(uint8_t c) ;

extern uint8_t GPS_update;
extern uint8_t GPS_FIX;
extern uint8_t GPS_Frame;  
extern uint8_t GPS_numSat;
extern int16_t GPS_altitude;   
extern uint16_t GPS_speed;
extern uint8_t GPS_Lost_Timeout;

void Set_Home_Coordinate(void);

//void GPS_calc_longitude_scaling(int32_t lat);
void GPS_calc_longitude_scaling(void);

uint32_t Get_Target_distance_hor(void);

//void GPS_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2, int32_t* bearing) ;
void GPS_distance_cm(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist) ;
//static void GPS_calc_velocity(void);
//static void GPS_calc_location_error( int32_t* target_lat, int32_t* target_lng, int32_t* gps_lat, int32_t* gps_lng );
void GPS_SerialInit(void);





#endif
