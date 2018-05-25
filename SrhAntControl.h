  /*

  SrhAntControl.h - Arduino header for communicating controling Srh antennas.*/

#ifndef SrhAntControl_H

#define SrhAntControl_H

// Modbus timeout [milliseconds]
//static const uint16_t ku16MBResponseTimeout          = 100; ///< Modbus timeout [milliseconds]
     
const byte PPS_Pin = 2;
static const unsigned int timePort = 8888;      // 
static const unsigned int antPort  = 9998;      //
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; //Задаем mac-адресс

unsigned long ntpUnixTime;
const unsigned long seventyYears = 2208988800UL;

const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];

struct Srh_Net_Packet {
  char       Signature[15];
  uint16_t   Payload;
  uint16_t   Cmd;
  uint16_t   Value;
  long       TimeStamp1;
  long       TimeStamp2;
} ;
struct Srh_State_Packet {
  char       Signature[15];
  uint16_t   Payload;
  uint16_t   Cmd;
  int16_t    Value;
  long       TimeStamp1;
  long       TimeStamp2;
  uint16_t   AntAzState;
  uint16_t   AntAzFreeq;
  uint16_t   AntElState;
  uint16_t   AntElFreeq;
} ;
struct Srh_Gps_Packet {
  char       Signature[17];
  uint16_t   Payload;
  long       TimeStamp1;
  long       TimeStamp2;
  uint32_t   Longitude;
  uint32_t   Latitude;
  float      Altitude;
  //uint16_t   Extra;
} ;
struct Srh_State_Packet_neo {
  char       Signature[13];
  uint16_t   Payload;
  uint16_t   AntAzState;
  uint16_t   AntAzFreeq;
  uint16_t   AntElState;
  uint16_t   AntElFreeq;
  long       TimeStamp1;
  long       TimeStamp2;
} ;

byte AntPacketBuffer[sizeof(Srh_Net_Packet)];
byte GpsPacketBuffer[sizeof(Srh_Gps_Packet)];

static const uint16_t ku16MBResponseTimeout          = 500; ///< Modbus timeout [milliseconds]
#define Max_485_enable 7 //pin 7 at arduino due
void preTransmission() {
  digitalWrite(Max_485_enable, 1);
}
void postTransmission() {
  digitalWrite(Max_485_enable, 0);
}
#endif
