#include <DueTimer.h>
#include <ModbusMaster.h>
#include "SrhAntControl.h"
#include <neotimer.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

ModbusMaster RTU_Azimuth;
ModbusMaster RTU_Elev;

Neotimer SysTimer    = Neotimer(500);
Neotimer ModbusTimer = Neotimer(250);
Neotimer NTPTimer    = Neotimer(1000);

IPAddress TimeServer(192, 168, 0, 2); // time-a.timefreq.bldrdoc.gov NTP сервер (132, 163, 4, 101)
IPAddress ip(192, 168, 0, 168);// configure local IP
IPAddress WhoRequestedIP;
EthernetUDP UdpNTP;
EthernetUDP UdpAnt;

Srh_State_Packet SrhStatePacket;
Srh_Gps_Packet   SrhGpsPacket;
Srh_Net_Packet   Packet;
byte StatePacketBuffer[sizeof(Srh_State_Packet)];

int AzCommand = 0x043C;
int AzSpeed   = 0;
int ElCommand = 0x047C;
int ElSpeed   = 0x0100;
int e,a, i = 0;
bool PPSflag;
bool TimeSourceFlag = true;
uint32_t millisZero,msec=0;

unsigned long SendNTPpacket(IPAddress & address) {
  if (TimeSourceFlag){
    ntpUnixTime++;
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    packetBuffer[0] = 0b11100011;
    packetBuffer[1] = 0;
    packetBuffer[2] = 6;
    packetBuffer[3] = 0xEC;
    packetBuffer[12]  = 49;
    packetBuffer[13]  = 0x4E;
    packetBuffer[14]  = 49;
    packetBuffer[15]  = 52;
    UdpNTP.beginPacket(address, 123);
    UdpNTP.write(packetBuffer, NTP_PACKET_SIZE);
    UdpNTP.endPacket();
  }
}
void NTPHandler() {
  if ( UdpNTP.parsePacket() == NTP_PACKET_SIZE) {
    UdpNTP.read(packetBuffer, NTP_PACKET_SIZE);
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    ntpUnixTime = secsSince1900 - seventyYears; // конвертируем время в Unix
      //rtc.setClock(ntpUnixTime);
/*      Serial.print("I'v get at  ");
      Serial.print((ntpUnixTime  % 86400L) / 3600);
      Serial.print(':');
      if ( ((ntpUnixTime % 3600) / 60) < 10 ) {
        Serial.print('0');
      }
      Serial.print((ntpUnixTime  % 3600) / 60);
      Serial.print(':');
      if ( (ntpUnixTime % 60) < 10 ) {
        Serial.print('0');
      }
      Serial.print(ntpUnixTime % 60);
      Serial.print('.');
      Serial.println( millis() - millisZero);*/
  }
}

void PPS_Handler(){
  millisZero = millis();
  PPSflag = true;
}
void sendStatePacket(IPAddress & WhoRequestedIP) {
//  Serial.print("/'I've sent to  ");
//  Serial.print(WhoRequestedIP);
  SrhStatePacket.Payload = 8;
  SrhStatePacket.TimeStamp1 = ntpUnixTime;//get Epoch from RTC
  SrhStatePacket.Signature[0] = 'S';
  SrhStatePacket.Signature[1] = 'T';
  SrhStatePacket.Signature[2] = 'A';
  SrhStatePacket.Signature[3] = 'T';
  SrhStatePacket.Signature[4] = 'E';
  memcpy(StatePacketBuffer, &SrhStatePacket, sizeof(Srh_State_Packet));
  UdpAnt.beginPacket(WhoRequestedIP, antPort);
  UdpAnt.write(StatePacketBuffer, sizeof(Srh_State_Packet));
  UdpAnt.endPacket();
}
void PacketHandler() {
  if (UdpAnt.parsePacket() == 30) {
    UdpAnt.read(AntPacketBuffer, sizeof(Srh_Net_Packet));
    uint16_t Cmd   = word(AntPacketBuffer[18], AntPacketBuffer[19]);
    uint16_t Value = word(AntPacketBuffer[20], AntPacketBuffer[21]);
    switch (Cmd) {
      case 100: {
          //Serial.println ("STATE requested");
          IPAddress WhoRequestedIP = UdpAnt.remoteIP();
          sendStatePacket(WhoRequestedIP);
        } break;
      case 101: {
          Serial.print ("Azimuth STOP ");
          AzCommand = 0x043C;
          AzSpeed   = 0x0;
        } break;
      case 102: {
          Serial.print ("Azimuth");
          AzCommand = 0x047C;
          AzSpeed = Value;
        } break;
/*      case 103: {
          Serial.print ("BACKWARD");
          AzCommand = 0x047C;
          AzSpeed = Value;
        } break;*/
      case 104: {
          Serial.print ("Hour angle ");
          ElCommand = 0x047C;
          ElSpeed = Value;
          Serial.println (ElSpeed);
        } break;
/*      case 105: {
          Serial.print ("DOWN");
          ElCommand = 0x047C;
          ElSpeed = Value;
        } break;*/
      case 106: {
          Serial.println ("Elevation STOP");
          ElCommand = 0x043C;
          ElSpeed   = 0x0;
        } break;
      case 200: {
          Serial.print ("GPS requested - ");
          IPAddress WhoRequestedIP = UdpAnt.remoteIP();
          Serial.println (ElSpeed);
          //sendGpsPacket(WhoRequestedIP);
        } break;
    }
    //Serial.print(" from: ");
    //Serial.println(UdpAnt.remoteIP());
    UdpAnt.stop();
    UdpAnt.begin(antPort);
  }
}

void SendToVLT_Azimuth() {
  uint8_t result;
  RTU_Azimuth.setTransmitBuffer(0, AzCommand);
  RTU_Azimuth.setTransmitBuffer(1, AzSpeed);
  result = RTU_Azimuth.writeMultipleCoils(0, 32);
  if (result != 0) Serial.print("RTU_Azimuth.writeMultipleCoils"); 
  delay(20);
  GetFromVLT_Azimuth();
}
void GetFromVLT_Azimuth() {
  uint8_t result;
  result = RTU_Azimuth.readCoils(32, 32);
  if (result == RTU_Azimuth.ku8MBSuccess)
  {
    //Serial.println(result);
    SrhStatePacket.AntAzState = RTU_Azimuth.getResponseBuffer(0);
    SrhStatePacket.AntAzFreeq = RTU_Azimuth.getResponseBuffer(1);
    //Serial.print(AntAzState, HEX);
    //Serial.print("Azimuth speed ");
    //Serial.println(SrhStatePacket.AntAzFreeq);
  }else SrhStatePacket.AntAzState = -3848; // If VLT is not connected or modbus is down... (~SrhStatePacket.AntAzState = -3848)
}
void SendToVLT_Elevation() {
  uint8_t result;
  RTU_Elev.setTransmitBuffer(0, ElCommand);
  RTU_Elev.setTransmitBuffer(1, ElSpeed);
  result = RTU_Elev.writeMultipleCoils(0, 32);
  if (result != 0) Serial.println("RTU_Elev.writeMultipleCoils ERROR");
  delay(20);
  GetFromVLT_Elevation();
}
void GetFromVLT_Elevation() {
  uint8_t result;
  result = RTU_Elev.readCoils(32, 32);
  if (result == RTU_Elev.ku8MBSuccess){
    //Serial.println(result);
    SrhStatePacket.AntElState = RTU_Elev.getResponseBuffer(0);
    SrhStatePacket.AntElFreeq = RTU_Elev.getResponseBuffer(1);
    //Serial.print(~SrhStatePacket.AntElState);
    Serial.println(SrhStatePacket.AntElFreeq);
  }else SrhStatePacket.AntElState = -3848; // If VLT is not connected or modbus is down... (~SrhStatePacket.AntAzState = -3848)
}
void setup() {
  pinMode (PPS_Pin, INPUT_PULLUP);
  attachInterrupt (digitalPinToInterrupt (PPS_Pin), PPS_Handler, RISING);
  Serial.begin(9600);// Open serial communications and wait for port to open:
  while (!Serial) {} ; // wait for serial port to connect. Needed for native USB port only
  pinMode(Max_485_enable, OUTPUT);
  digitalWrite(Max_485_enable, 0);
  Serial1.begin(4800); //Open RX485 serial communications: settings VLT 8-32
  RTU_Elev.begin(2, Serial1); //ModBus slave ID is 1 - Azimuth, 2 - HAngle
  RTU_Elev.preTransmission(preTransmission);
  RTU_Elev.postTransmission(postTransmission);
  RTU_Azimuth.begin(1, Serial1); //ModBus slave ID is 1 - Azimuth, 2 - HAngle
  RTU_Azimuth.preTransmission(preTransmission);
  RTU_Azimuth.postTransmission(postTransmission);
  SPI.begin(4); // initialize the bus for a Ethernet device on pin 4 
  //SPI.begin(10); // initialize the bus for a SD-card device on pin 10 
  Ethernet.begin(mac, ip);
  Serial.println(Ethernet.localIP());
  UdpNTP.begin(timePort);
  UdpAnt.begin(antPort);
}
void loop() {
  if (PPSflag){ //PPS handler. To get miliseconds use: msec = millis() - millisZero;
    PPSflag = false;
/*      msec = millis() - millisZero;
      Serial.print("PPS ");
      Serial.println(msec);*/
    }
   if(SysTimer.repeat()){ //"Calling this periodically each 1/2 second"
      ModbusTimer.start();
      SendToVLT_Azimuth();
    }
   if (ModbusTimer.done()){//"Calling this after SysTimer in 100 msec"
      ModbusTimer.stop();
      ModbusTimer.reset();
      SendToVLT_Elevation();
      NTPHandler();
    }
   if(NTPTimer.repeat()){ //"Calling this periodically each 1 second"
      SendNTPpacket(TimeServer);
    }
  PacketHandler();  
   
}
