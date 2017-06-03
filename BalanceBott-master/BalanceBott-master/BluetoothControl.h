#ifndef _BT_CONTROL_
#define _BT_CONTROL_

#ifdef _BT_SERIAL_HARD
HardwareSerial BtPort = Serial1;//cambia el nombre del puerto serial
#else
SoftwareSerial BtPort;

uint8_t bt_init();
uint8_t bt_config();
uint8_t bt_recive(float* position);



#endif
