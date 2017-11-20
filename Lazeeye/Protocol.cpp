#include "Arduino.h"
#include "config.h"
#include "Protocol.h"
#include "Serial.h"
#include "GPS.h"

static uint8_t CURRENTPORT=0;

void serialCom() {
  //uint8_t c,cc,port,state,bytesTXBuff;
  uint8_t c,cc,bytesTXBuff;
  static uint8_t offset[UART_NUMBER];
  static uint8_t dataSize[UART_NUMBER];
  static uint8_t c_state[UART_NUMBER];
  uint32_t timeMax; // limit max time in this function in case of GPS

  timeMax = micros();
  
  cc = SerialAvailable(GPS_SERIAL);
  
  while (cc--) {
    bytesTXBuff = SerialUsedTXBuff(GPS_SERIAL); // indicates the number of occupied bytes in TX buffer
    if (bytesTXBuff > TX_BUFFER_SIZE - 50 ) return; // ensure there is enough free TX buffer to go further (50 bytes margin)
    c = SerialRead(GPS_SERIAL);
    //static uint32_t GPS_last_frame_seen; //Last gps frame seen at this time, used to detect stalled gps communication
    if (GPS_newFrame(c)) {
      //We had a valid GPS data frame, so signal task scheduler to switch to compute
      if (GPS_update == 1) GPS_update = 0; 
      else GPS_update = 1; //Blink GPS update
      
      if ((GPS_State == GPS_STATE_WAIT4GPS) || (GPS_State == GPS_STATE_LOST)) 
        GPS_State = GPS_STATE_WAIT4FIX; //切换状态机
      
      GPS_Lost_Timeout = 0; //清零GPS超时计时器
      
      //GPS_last_frame_seen = timeMax;
      GPS_Frame = 1;
      
    }

    // Check for stalled GPS, if no frames seen for 1.2sec then consider it LOST
    //这是旧的GPS丢失机制，这里就先不用了。
    /*if ((timeMax - GPS_last_frame_seen) > GPS_TIMEOUT) {  //注意，由于GPS刷新率降到1.5s一次，所以这里要改，不然判定信号丢失
      //No update since 1800ms clear fix...
      GPS_FIX = 0;    //注意这里已经做了失帧检测在GPS_FIX里面，只要GPS_FIX==1,那么当前数据有效，可以刷LCD
      GPS_numSat = 0;
    }
    */
    if (micros()-timeMax>250) return;  // Limit the maximum execution time of serial decoding to avoid time spike
  } // while
}
