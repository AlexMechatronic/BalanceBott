
#define MPU6500 1
#define EMPL_TARGET_ARDUINO 1
#define __AVR_ATmega2560__ 1

#include <Arduino.h>
#include <Source\PID\PID.h>
#include <Source\I2Cdev\I2Cdev.h>

#include <Source\helper3Dmath\helper_3dmath.h>
#include <eMPL\inv_mpu.h>
#include <eMPL\inv_mpu_dmp_motion_driver.h>
#include <stdint.h>
#include <HardwareSerial.h>
#include <SoftwareSerial\src\SoftwareSerial.h>


const unsigned char signal_pin = 12;

// Necesario para el MPU
const char dmp_features =
DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_CAL_GYRO |
DMP_FEATURE_GYRO_CAL;
unsigned const short mpu_rate_hz = 100;
unsigned char sensors = // los sensores que se usaran y dosponibles en el modulo
INV_XYZ_ACCEL | INV_XYZ_GYRO;
//checar datasheet para ver que escalas y a que bits se debe asignar
unsigned const char accel_fsr = 0;//es para la escala que se usara
unsigned const char gyro_fsr = 0;//es para la escala que se usara
uint8_t int_pin = 20;
struct int_param_s int_param;

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
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
int16_t gyro[3];        // [x, y, z]            gyro vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


int speed_real_l, speed_real_r;
int pwm, pwm_l, pwm_r;
int Turn_Need, Speed_Need;
float angle, angular_rate;
boolean blinkState = false;
int rx_count = 0;
byte buf_tmp = 0;
uint8_t i2cData[14]; // Buffer for I2C data

uint8_t setup_mpu(void)
{
	int_param.pin = int_pin;
	mpu_init(&int_param);
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

//Interrupts
volatile boolean mpuInterrupt = 0;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
	mpuInterrupt = 1;
}

volatile boolean btInterrupt = 0;
void btDataReady()
{
	btInterrupt = 1;
}


void setup()
{
	BtPort.begin(9600);
	Wire.begin();// join I2C bus (I2Cdev library doesn't do this automatically)
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
			   //  Serial.begin(9600);

			   // initialize device
	if (setup_mpu())
	{
		Serial.println("mpu no inicializada correctamemte");
	}

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
		Serial.print(("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println((")"));
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
		Serial.println(F("FIFO overflow!"));
		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (mpuIntStatus & 0x02)
	{

	}
	/* add main program code here */

}

/*se debe transmitir de bit menos significativo al mas significativo
@return 1 si 
*/

