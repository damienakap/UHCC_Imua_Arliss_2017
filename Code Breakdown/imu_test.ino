
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

static int imu_accel_max;
static int imu_accel_min;
static int imu_accel_res;
static int imu_gyro_max;
static int imu_gyro_min;
static int imu_gyro_res;
static int imu_mag_max;
static int imu_mag_min;
static int imu_mag_res;
static int imu_bmp_max;
static int imu_bmp_min;
static int imu_bmp_res;

static float imu_accel_x;
static float imu_accel_y;
static float imu_accel_z;
static float imu_mag_x;
static float imu_mag_y;
static float imu_mag_z;
static float imu_gyro_x;
static float imu_gyro_y;
static float imu_gyro_z;
static float imu_bmp_pressure;
static float imu_bmp_temp;
static float imu_seaLevelPressure;

//define reusable sensor and event vars/pointers
sensor_t sensor;
sensors_event_t event;

// needed for Serial to work on Feather M0 Adalogger
#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

float updateAltitude(){
  bmp.getEvent(&event);
  bmp.getTemperature(&imu_bmp_temp);
  return bmp.pressureToAltitude(imu_seaLevelPressure,event.pressure,imu_bmp_temp);
}

void updateAccel(){
  accel.getEvent(&event);
  imu_accel_x = event.acceleration.x; // m/s^2
  imu_accel_y = event.acceleration.y; // m/s^2
  imu_accel_z = event.acceleration.z; // m/s^2
  return;
}

void updateGyro(){
  mag.getEvent(&event);
  imu_mag_x = event.magnetic.x; // uT
  imu_mag_y = event.magnetic.y; // uT
  imu_mag_z = event.magnetic.z; // uT
  return;
}

void updateMag(){
  gyro.getEvent(&event);
  imu_gyro_x = event.gyro.x; // uT
  imu_gyro_y = event.gyro.y; // uT
  imu_gyro_z = event.gyro.z; // uT
  return;
}

void updateBpm(){
  bmp.getEvent(&event);
  imu_bmp_pressure = event.pressure; // hPa
  imu_bmp_temp; // C
  bmp.getTemperature(&imu_bmp_temp);
  imu_seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
  return;
}

void setup(void){
  Serial.begin(115200);

  /* Initialise the sensors */
  if(!accel.begin()){
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin()){
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin()){
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  if(!gyro.begin()){
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  // get sensor constraints
  accel.getSensor(&sensor);
  imu_accel_max = sensor.max_value; // m/s
  imu_accel_min = sensor.min_value; // m/s
  imu_accel_res = sensor.resolution; // m/s^2
  
  gyro.getSensor(&sensor);
  imu_gyro_max = sensor.max_value; // rad/s
  imu_gyro_min = sensor.min_value; // rad/s
  imu_gyro_res = sensor.resolution; // rad/s^2
  
  mag.getSensor(&sensor);
  imu_mag_max = sensor.max_value; // uT
  imu_mag_min = sensor.min_value; // uT
  imu_mag_res = sensor.resolution; // uT

  bmp.getSensor(&sensor);
  imu_bmp_max = sensor.max_value; // hPa
  imu_bmp_min = sensor.min_value; // hPa
  imu_bmp_res = sensor.resolution; // hPa

  // initail sensor value read
  updateAccel();
  updateGyro();
  updateMag();
  updateBpm();
}

// Test sensors
void loop(void){
  updateAccel();
  Serial.print("z: ");Serial.println(imu_accel_z);
  delay(100);
}
