#define MPU6500 1
#define MOTION_DRIVER_TARGET_ARDUINO 1
#define __AVR_ATmega2560__ 1

#include <stdbool.h>
#include <stdint.h>
#include <Arduino.h>
#include "Source\PID\PID.h"
#include "Source\I2Cdev\I2Cdev.h"
#include "Source\helper3Dmath\MahonyAHRS.h"

#include "eMPL\inv_mpu.h"
#include "eMPL\inv_mpu_dmp_motion_driver.h"


#include <HardwareSerial.h>
#include <SoftwareSerial.h>

#define _debug_init(x)	Serial.begin(x)
#define _debug_print(x) Serial.print(x)


const unsigned char signal_pin = 12;

// Necesario para el MPU
const char dmp_features =
	DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_CAL_GYRO |
	DMP_FEATURE_GYRO_CAL;

unsigned const short mpu_rate_hz = 50;
unsigned char sensors = // los sensores que se usaran y dosponibles en el modulo
#ifdef MPU9250
	INV_XYZ_COMPASS |
#endif // MPU9250
	INV_XYZ_ACCEL | INV_XYZ_GYRO;
//checar datasheet para ver que escalas y a que bits se debe asignar
const enum accel_fsr_e accel_fsr = INV_FSR_2G;//es para la escala que se usara de +-2g
const enum gyro_fsr_e gyro_fsr = INV_FSR_250DPS;//es para la escala que se usara de +-250dgs
//struct necesaria para activarla interrupcion apartir de la libreria
struct int_param_s int_param = { .cb = mpuDataReady, .pin = 20 }; //es para cuando ocurra una interrupcion y el pin al que esta unida

char gyro_orientation[] = { 1, 0, 0,
0, 1, 0,
0, 0, 1 };


//constantes del PID
PID position;

boolean dmpReady = 0;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

						// orientation/motion vars
QuaternionStruct q;           // [w, x, y, z]         quaternion container
Vector3F gravity;    // [x, y, z]            gravity vector
Vector3U16 gyro;        // [x, y, z]            gyro vector
EulerAnglesStruct ypr;           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


int speed_real_l, speed_real_r;
int pwm, pwm_l, pwm_r;
int Turn_Need, Speed_Need;
float angle, angular_rate;

byte buf_tmp = 0;
uint8_t i2cData[14]; // Buffer for I2C data

					 //Interrupts
volatile boolean mpuInterrupt = 0;     // indicates whether MPU interrupt pin has gone high
void mpuDataReady(void)
{
	mpuInterrupt = 1;
}

uint8_t setup_mpu(void)
{
	uint8_t result;
	result = mpu_init(&int_param);
	if (result)
	{

	}
	mpu_set_gyro_fsr(gyro_fsr);
	mpu_set_accel_fsr(accel_fsr);

	//int mpu_set_lpf(unsigned short lpf) ;
	mpu_set_sample_rate(mpu_rate_hz);
	//int mpu_set_compass_sample_rate(unsigned short rate);
	mpu_configure_fifo(sensors);
	mpu_set_sensors(sensors);


	dmp_load_motion_driver_firmware();
	dmp_enable_feature(dmp_features);
	dmp_set_fifo_rate(mpu_rate_hz);
	dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
	mpu_set_dmp_state(1);
	dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);

	dmpReady = 1;
	return 0;
}
void setup()
{
	Wire.begin();// join I2C bus (I2Cdev library doesn't do this automatically)
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
	
	_debug_init(9600);

	// initialize device
	setup_mpu();

	////////////////////////////////////////
	// modificar deacuerdo al dispositivo que tengo despues de probarlo
	// supply your own gyro offsets here, scaled for min sensitivity


	// make sure it worked (returns 0 if so)
	if (devStatus == 0)
	{
		// get expected DMP packet size for later comparison
		packetSize = 0;
	}
	else
	{
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		_debug_print(("DMP Initialization failed (code "));
		_debug_print(devStatus);
		_debug_print((")\r\n"));
	}
}

void loop()
{

	// if programming failed, don't try to do anything
	if (!dmpReady) return;


	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024)
	{
		// reset so we can continue cleanly
		_debug_print(F("FIFO overflow!"));
		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (mpuIntStatus & 0x02)
	{

	}
	/* add main program code here */

}
