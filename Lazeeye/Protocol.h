#ifndef PROTOCOL_H_
#define PROTOCOL_H_

enum GPS_STATE{
  GPS_STATE_WAIT4GPS,  //上电无GPS
  GPS_STATE_WAIT4FIX,     //接收到GPS帧
  GPS_STATE_FIXED,          //已定位
  GPS_STATE_LOST          //GPS信号丢失
};

extern uint8_t GPS_State;

void serialCom();

#endif
