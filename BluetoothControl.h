#ifndef _BT_CONTROL_
#define _BT_CONTROL_
#include <Arduino.h>

#ifdef _BT_SERIAL_HARD
#include <HardwareSerial.h>
HardwareSerial BtPort = Serial1;//cambia el nombre del puerto serial
#else
#include <SoftwareSerial.h>
SoftwareSerial BtPort;
#endif

uint8_t bt_init();
uint8_t bt_config();
uint8_t bt_recive(float* position);



#endif
