
#define MPU6500 1
#define EMPL_TARGET_ARDUINO 1

#include <Arduino.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <helper_3dmath.h>

const unsigned char signal_pin = 12;

// Necesario para el MPU
const char dmp_features = 
	DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_SEND_CAL_GYRO |
	DMP_FEATURE_GYRO_CAL;
unsigned const short mpu_rate_hz = 100;
unsigned char sensors = // los sensores que se usaran y dosponibles en el modulo
	INV_XYZ_ACCEL|INV_XYZ_GYRO;
	//checar datasheet para ver que escalas y a que bits se debe asignar
unsigned const char accel_fsr = 0;//es para la escala que se usara
unsigned const char gyro_fsr = 0;//es para la escala que se usara
struct int_param_s int_param = { .pin = 20 };


static struct platform_data_s gyro_pdata = {
	.orientation = { 1, 0, 0,
					 0, 1, 0,
					 0, 0, 1}
};


//constantes del PID
const float kp;
const float ki;
const float kd;

bool dmpReady = false;  // set true if DMP init was successful
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
bool blinkState = false;
int rx_count = 0;
byte buf_tmp = 0;
uint8_t i2cData[14]; // Buffer for I2C data

uint8_t setup_mpu(void)
{
	mpu_init(&int_param) ;
	mpu_set_gyro_fsr(gyro_fsr);
	mpu_set_accel_fsr(accel_fsr);
	//int mpu_set_lpf(unsigned short lpf) ;
	mpu_set_sample_rate(mpu_rate_hz) ;
	//int mpu_set_compass_sample_rate(unsigned short rate);
	mpu_configure_fifo(sensors) ;
	mpu_set_sensors(sensors) ;
	
	dmp_load_motion_driver_firmware();
	dmp_enable_feature(dmp_features);
	dmp_set_fifo_rate(mpu_rate_hz);
	dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
	mpu_set_dmp_state(1);
	dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
	set_int_enable(1);
	dmp_ready=1;
	return 0;
}

static inline void run_self_test(void)
{
	int result;
	long gyro[3], accel[3];

#if defined (MPU6500) || defined (MPU9250)
	result = mpu_run_6500_self_test(gyro, accel, 0);
#elif defined (MPU6050) || defined (MPU9150)
	result = mpu_run_self_test(gyro, accel);
#endif
	if (result == 0x7) {
	MPL_LOGI("Passed!\n");
		MPL_LOGI("accel: %7.4f %7.4f %7.4f\n",
					accel[0]/65536.f,
					accel[1]/65536.f,
					accel[2]/65536.f);
		MPL_LOGI("gyro: %7.4f %7.4f %7.4f\n",
					gyro[0]/65536.f,
					gyro[1]/65536.f,
					gyro[2]/65536.f);
		/* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

#ifdef USE_CAL_HW_REGISTERS
		/*
		 * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
		 * instead of pushing the cal data to the MPL software library
		 */
		unsigned char i = 0;

		for(i = 0; i<3; i++) {
			gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
			accel[i] *= 4096.f; //convert to +-8G
			accel[i] = accel[i] >> 16;
			gyro[i] = (long)(gyro[i] >> 16);
		}

		mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
		mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
		mpu_set_accel_bias_6050_reg(accel);
#endif
#else
		/* Push the calibrated data to the MPL library.
		 *
		 * MPL expects biases in hardware units << 16, but self test returns
		 * biases in g's << 16.
		 */
		unsigned short accel_sens;
		float gyro_sens;

		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		inv_set_accel_bias(accel, 3);
		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long) (gyro[0] * gyro_sens);
		gyro[1] = (long) (gyro[1] * gyro_sens);
		gyro[2] = (long) (gyro[2] * gyro_sens);
		inv_set_gyro_bias(gyro, 3);
#endif
	}
	else {
			if (!(result & 0x1))
				MPL_LOGE("Gyro failed.\n");
			if (!(result & 0x2))
				MPL_LOGE("Accel failed.\n");
			if (!(result & 0x4))
				MPL_LOGE("Compass failed.\n");
	 }

}

//Interrupts
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
	mpuInterrupt = true;
}

volatile bool btInterrupt = false;
void btDataReady()
{
	btInterrupt = true;
}


void setup()
{
	Wire.begin();// join I2C bus (I2Cdev library doesn't do this automatically)
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
			   //  Serial.begin(9600);

			   // initialize device
	if(mpu_init())
	{
		Serial.println("mpu no inicializada correctamemte");
	}
	
	////////////////////////////////////////
	// modificar deacuerdo al dispositivo que tengo despues de probarlo
	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(10);
	mpu.setYGyroOffset(7);
	mpu.setZGyroOffset(14);
	mpu.setZAccelOffset(900); // 1688 factory default for my test chip

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
	

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024)
	{
		// reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.println(F("FIFO overflow!"));
		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (mpuIntStatus & 0x02)
	{

	}
  /* add main program code here */

}

uint8_t bt_recive(char* position)
{
	//aqui va el codigo para ver que es lo que va leyendo
	//debe recivir el valor de dos variables x e y
}

uint8_t bt_config()
{
	
}

uint8_t bt_init()
{
	
}

