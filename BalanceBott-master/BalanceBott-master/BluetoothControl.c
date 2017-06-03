#include "BluethootControl.h"

uint8_t bt_recive(float* position)
{
	uint8_t pckt_lenght = 4;
	uint8_t data[pckt_lenght];
	if (BtPort.available() > 0)
	{
		BtPort.readBytes(data, pckt_lenght);
		for(uint8_t i = 0; i<pckt_lenght; i+=2)
		{
			position[i] = (float)(data[i]<<0|data[i+1]<<8);
		}
		/* //si es un float de 4 bytes el que se transmite de forma LSB
		for(uint8_t i = 0; i<pckt_lenght; i+=4)
		{
			position[i] = (float)(data[i]<<0|data[i+1]<<8|data[i+2]<<16|data[i+3]<<24);
		}
		/*/
		return 1;
	}
	//aqui va el codigo para ver que es lo que va leyendo
	//debe recivir el valor de dos variables x e y
	return 0;
}

uint8_t bt_config()
{

}

uint8_t bt_init()
{

}
