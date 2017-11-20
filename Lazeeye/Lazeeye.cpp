#include "Arduino.h"
#include "config.h"
#include "Serial.h"
#include "GPS.h"
#include "Protocol.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2


#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

uint8_t GPS_update = 0; //valid GPS frame will toogle this value to blink led
uint8_t GPS_FIX = 0;    //GPS fixed flag
uint8_t GPS_Frame = 0;      //GPS frame coming Sem
uint8_t New_GPS_Data = 0; 
uint8_t GPS_Frame_Counter = 0;
uint8_t Record_Save_Timer = 0;

struct GPS_DATA_STRU{
  uint16_t Speed;
  int32_t Altitude;
  uint32_t Distance;
  uint32_t Trip;
};
float tripSum = 0;
uint32_t Distance_Hor;

GPS_DATA_STRU Record_Data;
GPS_DATA_STRU Realtime_Data;

extern uint8_t GPS_numSat;
extern uint16_t GPS_speed;  
extern int16_t GPS_altitude;   
extern int16_t Home_alt;

uint8_t Home_isSet;

uint8_t GPS_State;

uint8_t GPS_Lost_Timeout = 0;

unsigned long previous_millis_low=0;
unsigned long previous_millis_med =0;
unsigned long previous_millis_high =0;

uint8_t Current_Page;

enum Page_NUM{
  PAGE_A = 0,
  PAGE_B,
  PAGE_C
};

uint8_t KEY_A_Timer = 0;

uint8_t KEY_A_Last = 1;

uint8_t KEY_A_This = 1;

uint8_t KEY_A_Pressed = 0;

uint8_t KEY_A_longPressed = 0;

uint8_t counter = 0;

//unsigned long watch_start, watch_stop, watch_period;

void record_browser(){

}

void setup(){
  Home_isSet = 0;
  EEPROM.get(RECORD_EEPROM_ADD, Record_Data);
  Realtime_Data.Speed = 0;
  Realtime_Data.Altitude = -9999;
  Realtime_Data.Distance = 0;
  Realtime_Data.Trip = 0;
  
  GPS_State = GPS_STATE_WAIT4GPS;
  Current_Page = PAGE_A;
  
  //LED pin config
  pinMode(LedRed, OUTPUT);  
  pinMode(LedGre, OUTPUT);  
  digitalWrite(LedRed,HIGH);
  digitalWrite(LedGre,HIGH);
  
  //button with external 10K pull-up resistor
  pinMode(KEY_A,INPUT);
  //pinMode(KEY_B,INPUT);
  //pinMode(KEY_C,INPUT);

  GPS_SerialInit();
  
  digitalWrite(LedRed,LOW);
  digitalWrite(LedGre,LOW);
  
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // address is 3C for my 12864 OLED
  //Show startup screen
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(15,25);
  display.println("Laze GPS");
  display.display();
  display.setTextSize(1); //use small size text after that
  delay(750);
  //Show best record
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("Best Record:");
  display.setCursor(0,12);
  display.print("Speed   ");
  display.print(Record_Data.Speed);
  display.setCursor(0,24);
  display.print("Alt.    ");
  display.print(Record_Data.Altitude);
  display.setCursor(0,36);
  display.print("Dist.   ");
  display.print(Record_Data.Distance);
  display.setCursor(0,48);
  display.print("Trip.   ");
  display.print(Record_Data.Trip);
  display.display();
  
  while(counter++ < 30)
  {
    if(KEY_A_DOWN)
    {
      record_browser();
      break;
    }
    delay(100); 
  }
}

void loop(){
  unsigned long currentMillis = millis();
  serialCom();
  
  //high speed task
  if((currentMillis - previous_millis_high) >= hi_speed_cycle)  //20Hz
  {
    previous_millis_high += hi_speed_cycle;
    
    //LED handler
    if(GPS_update)
      digitalWrite(LedRed,HIGH);
    else
      digitalWrite(LedRed,LOW);
    if(GPS_FIX)
      digitalWrite(LedGre,HIGH);
    else
      digitalWrite(LedGre,LOW);

    //Key A
    if(KEY_A_DOWN)
    {
      KEY_A_This = 0;
      KEY_A_Timer++;
    }
    else
      KEY_A_This = 1;  
    if((KEY_A_Last)&&(!KEY_A_This))//下降沿清零timer
      KEY_A_Timer = 0;
    if((!KEY_A_Last)&&(KEY_A_This))//上升沿则发送信号量
    {
      if(KEY_A_Timer>15)
        KEY_A_longPressed = 1;
      else 
        KEY_A_Pressed = 1;
      KEY_A_Timer = 0;
    }
    KEY_A_Last = KEY_A_This;
    
  }
  
  //med speed task
  if((currentMillis - previous_millis_med) >= med_speed_cycle) //5Hz
  {
    previous_millis_med += med_speed_cycle;
    
    //short press A could wake up OLED
    if(KEY_A_Pressed)
    {
      KEY_A_Pressed = 0;
      
      display.begin(SSD1306_SWITCHCAPVCC, 0x3C); 
      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setTextSize(1); //use small size text
      delay(800);
    }
    
    //long press button A could reset GPS record and reset home location 
    if(KEY_A_longPressed)
    {
      KEY_A_longPressed = 0;
      
      Record_Data.Speed = 0;
      Record_Data.Altitude = -9999;
      Record_Data.Distance = 0;
      Record_Data.Trip = 0;
      tripSum = 0;
      Realtime_Data.Trip = 0;
      
      EEPROM.put(RECORD_EEPROM_ADD,Record_Data);
 
      Home_isSet = 0;
    }
   
    
    //update display value when new valid GPS frame came
    if( GPS_FIX && GPS_Frame )
    {
      GPS_Frame = 0;
      GPS_Frame_Counter++;
      if(GPS_Frame_Counter>=3)
      {
        GPS_Frame_Counter = 3;
        New_GPS_Data = 1;
        
        if(!Home_isSet) 
        {
          Home_isSet = 1;
          Set_Home_Coordinate();
          GPS_calc_longitude_scaling();
        }
        
        Realtime_Data.Speed = GPS_speed * 0.036;
        Realtime_Data.Altitude = GPS_altitude - Home_alt;
        Distance_Hor = Get_Target_distance_hor();
        Realtime_Data.Distance = sqrt(sq(Distance_Hor) + sq(Realtime_Data.Altitude));
        tripSum += GPS_speed * 0.01; //trip sum in meter
        Realtime_Data.Trip = (uint32_t)tripSum;
        
        
        
        if(Realtime_Data.Speed > Record_Data.Speed)
          Record_Data.Speed = Realtime_Data.Speed;
        if(Realtime_Data.Altitude > Record_Data.Altitude)
          Record_Data.Altitude = Realtime_Data.Altitude;
        if(Realtime_Data.Trip > Record_Data.Trip)
          Record_Data.Trip = Realtime_Data.Trip;
        if(Realtime_Data.Distance > Record_Data.Distance)
          Record_Data.Distance = Realtime_Data.Distance;
      }
    }

    //corrding to GPS_FIX, switch the state machine
    if((GPS_State == GPS_STATE_WAIT4FIX) && GPS_FIX)
      GPS_State = GPS_STATE_FIXED;
    else if((GPS_State == GPS_STATE_FIXED) && !GPS_FIX)
      GPS_State = GPS_STATE_WAIT4FIX;
    
    //OLED display
    switch(Current_Page){
      case(PAGE_A):{
        if(GPS_State == GPS_STATE_WAIT4GPS) //no GPS is connected
        {
          display.clearDisplay();
          display.setCursor(25,28);
          display.println("Wait for GPS..");
        }
        else if(GPS_State == GPS_STATE_WAIT4FIX)//have GPS data but not fixed yet
        {
          display.clearDisplay();
          display.setCursor(25,28);
          display.println("Wait for FIX..");
        }
        else if(GPS_State == GPS_STATE_LOST)  //used to have GPS data but lost sometime
        {
          display.clearDisplay();
          display.setCursor(25,28);
          display.println("GPS LOST..");
        }
        else if( GPS_FIX && New_GPS_Data )//GPS fixed and have new frame
        {
          New_GPS_Data = 0;
          //GPS_Frame_Counter++;
          /*
          if(!Home_isSet) 
          {
            Home_isSet = 1;
            Set_Home_Coordinate();
            GPS_calc_longitude_scaling();
          }
          */

          display.clearDisplay();
          
          display.setCursor(0,0);
          if(Realtime_Data.Speed>999)
            Realtime_Data.Speed = 999;
          display.print("Speed ");
          display.print(Realtime_Data.Speed);
          display.setCursor(85,0);
          display.print(Record_Data.Speed);
          
          display.setCursor(0,12);
          if(Realtime_Data.Altitude > 9999)
            Realtime_Data.Altitude = 9999;
          display.print("Alt   ");
          display.print(Realtime_Data.Altitude);
          display.setCursor(85,12);
          display.print(Record_Data.Altitude);
          
          display.setCursor(0,24);
          if(Realtime_Data.Distance > 99999)
            Realtime_Data.Distance = 99999;
          display.print("Dist. ");
          display.print(Realtime_Data.Distance);
          display.setCursor(85,24);
          display.print(Record_Data.Distance); 
          
          display.setCursor(0,36);
          if(Realtime_Data.Trip > 99999)
            Realtime_Data.Trip = 99999;
          display.print("Trip  ");
          display.print(Realtime_Data.Trip);
          display.setCursor(85,36);
          display.print(Record_Data.Trip);
        }
        display.setCursor(0,48);
        display.print("Sat * ");
        display.print(GPS_numSat);
        
        display.display();  //only display one time here

        break;
      }
      //Page B show max record after power up
      case(PAGE_B):{

        break;
      }
      //debug page?
      case(PAGE_C):{

        break;
      }
      default:
        break;
    } 
  }
  
  //low speed task 2Hz
  if((currentMillis - previous_millis_low) >= lo_speed_cycle) 
  {
    previous_millis_low += lo_speed_cycle; 
    
    Record_Save_Timer++;
    if(Record_Save_Timer>=EEPROM_SAVE_PERIOD)
    {
      Record_Save_Timer = 0;
      if(GPS_FIX)
        EEPROM.put(RECORD_EEPROM_ADD,Record_Data);
    }

    if((GPS_State == GPS_STATE_WAIT4FIX)||(GPS_State == GPS_STATE_FIXED))
      GPS_Lost_Timeout++;
    if(GPS_Lost_Timeout>5)
    {  
      GPS_State = GPS_STATE_LOST;
      GPS_Frame_Counter = 0;
      GPS_FIX = 0;
      GPS_numSat = 0;
    }
  }
  
}


