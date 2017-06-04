
#include <stdbool.h>
#include <stdint.h>
#include <Arduino.h>
#include "Source\PID\PID.h"
#include "Source\helper3Dmath\MahonyAHRS.h"

#include "eMPL\inv_mpu.h"
#include "eMPL\inv_mpu_dmp_motion_driver.h"


#define _debug_init(x)  Serial.begin(x)
#define _debug_print(x) Serial.print(x)


#define GYRO_FSR_IN_DGR 200
#define ACCEL_FSR_IN_G 2
#define ACCEL_SENS    0
#define GYRO_SENS       ( 131.0f * 250.f / (float)GYRO_FSR_IN_DGR )
#define QUAT_SENS       1073741824.f //2^30

#define EPSILON         0.0001f
#define PI_2            1.57079632679489661923f


const unsigned char signal_pin = 12;

// Necesario para el MPU
const char dmp_features =
  DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_CAL_GYRO |
  DMP_FEATURE_GYRO_CAL;

unsigned const short mpu_rate_hz = 50;
short sensors_ = // los sensores que se usaran y dosponibles en el modulo
#ifdef MPU9250
  INV_XYZ_COMPASS |
#endif // MPU9250
  INV_XYZ_ACCEL | INV_XYZ_GYRO;
//checar datasheet para ver que escalas y a que bits se debe asignar

char gyro_orientation[] = { 1, 0, 0,
0, 1, 0,
0, 0, 1 };


//constantes del PID
PID position;

boolean dmpReady = 0;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
unsigned char fifoCount;     // count of all bytes currently in FIFO
//uint8_t fifoBuffer[64]; // FIFO storage buffer

            // orientation/motion vars
//QuaternionStruct q_wheelL, q_wheelR; //quaternions who have the description of the position of the contact of the wheel and ground
QuaternionStruct q_;
Vector3I16 acc_;    // [x, y, z]            gravity vector
Vector3I16 gyro_;        // [x, y, z]            gyro vector
Vector3F gyro;
Vector3F acc;
EulerAnglesStruct ypr;           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

unsigned long timeStamp_ = 0;

int speed_real_l, speed_real_r;
int pwm, pwm_l, pwm_r;
int Turn_Need, Speed_Need;
float angle, angular_rate;

unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set
unsigned int ret;

           //Interrupts
volatile boolean mpuInterrupt = 0;     // indicates whether MPU interrupt pin has gone high
void mpuDataReady(void)
{
  mpuInterrupt = 1;
}

uint8_t setup_mpu(void)
{
  //Fastwire::setup(400, 0);
  mpu_select_device(0);
  mpu_init_structures();

  ret = mpu_init(NULL);
#ifdef MPU_DEBUG
  if (ret) return 10 + ret;
#endif

  ret = mpu_set_sensors(sensors_);
#ifdef MPU_DEBUG
  if (ret) return 20 + ret;
#endif

  ret = mpu_set_gyro_fsr(GYRO_FSR_IN_DGR);
#ifdef MPU_DEBUG
  if (ret) return 30 + ret;
#endif

  ret = mpu_set_accel_fsr(ACCEL_FSR_IN_G);
#ifdef MPU_DEBUG
  if (ret) return 40 + ret;
#endif

  mpu_get_power_state((unsigned char *)&ret);
#ifdef MPU_DEBUG
  if (!ret) return 50 + ret;
#endif

  ret = mpu_configure_fifo(sensors_);
#ifdef MPU_DEBUG
  if (ret) return 60 + ret;
#endif

  dmp_select_device(0);
  dmp_init_structures();

  ret = dmp_load_motion_driver_firmware();
#ifdef MPU_DEBUG
  if (ret) return 80 + ret;
#endif

  ret = dmp_set_fifo_rate(mpu_rate_hz);
#ifdef MPU_DEBUG
  if (ret) return 90 + ret;
#endif

  ret = mpu_set_dmp_state(1);
#ifdef MPU_DEBUG
  if (ret) return 100 + ret;
#endif

  ret = dmp_enable_feature(dmp_features);
#ifdef MPU_DEBUG
  if (ret) return 110 + ret;
#endif
  dmpReady = 1;
  return 0;
}

void setup()
{
  Serial.begin(9600);
  // initialize device
#ifdef MPU_DEBUG
  switch (setup_mpu())
  {
  case 11:
  case 21:
  case 31:
  case 41:
  case 51:
  case 61:
  case 71:
  case 81:
  case 91:
  case 101:
  case 111:
  default:
    break;
  }
#else // MPU_DEBUG
  setup_mpu();
#endif
  ////////////////////////////////////////
  // modificar deacuerdo al dispositivo que tengo despues de probarlo
  // supply your own gyro offsets here, scaled for min sensitivity


}

void loop()
{

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  do {
    ret = dmp_read_fifo(gyro_.vector, acc_.vector, q_.quaternionL, &timeStamp_, &sensors_, &fifoCount);
    /* will return:
    0 - if ok
    1 - no packet available
    2 - if BIT_FIFO_OVERFLOWN is set
    3 - if frame corrupted
    <0 - if error
    */
    if (ret != 0) break;//verificarsi es el comando para salir del loop
  } while (fifoCount>1);
  
  switch (ret) {
  case 0: c++; break;
  case 1: np++; return;
  case 2: err_o++; return;
  case 3: err_c++; return;
  default:
    Serial.print("READ ERROR!  ");
    Serial.println(ret);
    return;
  }

  q_.Quaternion.w = (float)q_.quaternionL[0] / (float)QUAT_SENS;
  q_.Quaternion.x = (float)q_.quaternionL[1] / (float)QUAT_SENS;
  q_.Quaternion.y = (float)q_.quaternionL[2] / (float)QUAT_SENS;
  q_.Quaternion.z = (float)q_.quaternionL[3] / (float)QUAT_SENS;

  /* need to adjust signs and do the wraps depending on the MPU mount orientation */
  /* if axis is no centered around 0 but around i.e 90 degree due to mount orientation */
  /* then do:  mympu.ypr[x] = wrap_180(90.f+rad2deg(mympu.ypr[x])); */
  ypr = getEulerAngles(q_);

  /* need to adjust signs depending on the MPU mount orientation */
  gyro.Vector.x = -(float)gyro_.vector[2] / (float)GYRO_SENS;
  gyro.Vector.y = (float)gyro_.vector[1] / (float)GYRO_SENS;
  gyro.Vector.z = (float)gyro_.vector[0] / (float)GYRO_SENS;

  acc.Vector.x = (float)acc_.vector[0] / (float)ACCEL_SENS;
  acc.Vector.y = (float)acc_.vector[1] / (float)ACCEL_SENS;
  acc.Vector.z = (float)acc_.vector[2] / (float)ACCEL_SENS;

  if (!(c % 25)) {
    Serial.print(np); Serial.print("  "); Serial.print(err_c); Serial.print(" "); Serial.print(err_o);
    Serial.print(" Y: "); Serial.print(ypr.Angles.yaw);
    Serial.print(" P: "); Serial.print(ypr.Angles.pitch);
    Serial.print(" R: "); Serial.print(ypr.Angles.roll);
    Serial.print("\tgx: "); Serial.print(gyro.Vector.x);
    Serial.print(" gy: "); Serial.print(gyro.Vector.y);
    Serial.print(" gz: "); Serial.println(gyro.Vector.z);
  }
}


