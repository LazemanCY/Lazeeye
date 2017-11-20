#ifndef CONFIG_H_
#define CONFIG_H_

//---------------------GPS Varibles---------------------
#define GPS_BAUD 9600

#define UART_NUMBER 1

#define GPS_SERIAL 0

#define GPS_conf_lead_filter 0

#define NMEA  //Only NMEA is sucessfully tested

#define GPS_TIMEOUT 1200000 //

//---------------------System Varibles---------------------
#define lo_speed_cycle  500 //2Hz
 
#define med_speed_cycle 200  //5Hz 

#define hi_speed_cycle  50 //20Hz

#define RECORD_EEPROM_ADD 0

#define EEPROM_SAVE_PERIOD 20//under low speed 2Hz loop, 20 = 10s 

//---------------------LED define---------------------
#define LedRed A0
#define LedGre A1

//---------------------Button define---------------------
#define KEY_A A2
#define KEY_B A3
#define KEY_C 13

#define KEY_A_DOWN (digitalRead(KEY_A) == LOW)
#define KEY_B_DOWN (digitalRead(KEY_B) == LOW)
#define KEY_C_DOWN (digitalRead(KEY_C) == LOW)

//buzzer define
#define Buz_Pin 9

#endif /* CONFIG_H_ */
