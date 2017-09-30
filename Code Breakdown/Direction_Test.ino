
/*
 * This is code to test the 
 * the gps and imu to get
 * the direction to the target
 * 
 * There is also con on the
 * dead switch thats isn't
 * necessary for the test.
 */

//-----Start-----//

/*Dead Switch*/
const int deadSwitchPin = 6;
/*sd card*/
#include <SPI.h>
#include <SD.h>

const int chipSelect = 4;

/*GPS Vars*/
#include <Adafruit_GPS.h>
#include <math.h>

#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);

#define GPSECHO  true
uint32_t mission_timer = millis();
uint32_t timer = millis();
/* GPS pos
 *  lawn: 21.4068125, -157.8119114, 400m
 *  Waiahole forest reserve: 21.4144068, -157.8281146, 1500m
 *  windward mall: 21.4195172, -157.8056709, 1400m
*/

const double r_earth = 6371; // km
const double gps_target_latitude_deg = 40.8735733032;
const double gps_target_longitude_deg = -119.1088027954;

const double pi = 3.141592653;

const boolean usingInterrupt = false;
void useInterrupt(boolean);

const int gps_ping_time = 100; // ms

static int gps_hour;
static int gps_minute;
static int gps_second;
static int gps_millisecond;
static int gps_day;
static int gps_month;
static int gps_year;

static int gps_fix;
static int gps_fix_quality;
static double gps_latitude;
static double gps_longitude;
static float gps_lat;
static float gps_lon;
static double gps_latitude_deg;
static double gps_longitude_deg;
static float gps_angle; // relative to last position
static float gps_speed; //knots
static float gps_altitude;
static float gps_satellites;


/*IMU Vars*/
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

//-----Get Sensor Value Range-----
sensor_t sensor;
sensors_event_t event;

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

double toRadians(double deg){
  return (deg/(double)180.0)*pi;
}

double getDist1D(double x, double y){
  return pow(pow(x,2)+pow(y,2),0.5);
}

void logToSD(String dataString){
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.flush();
    dataFile.close();
    Serial.println(dataString);
  }else{
    Serial.println("error opening datalog.txt");
  }
}

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

  pinMode(deadSwitchPin, INPUT_PULLUP);

  Serial.print("Initializing SD card...");
  if(!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
  }
  Serial.println("card initialized.");
  
  /* Initialise GPS */
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

  gps_hour = GPS.hour;
  gps_minute = GPS.minute;
  gps_second = GPS.seconds;
  gps_millisecond = GPS.milliseconds;
  gps_day = GPS.day;
  gps_month = GPS.month;
  gps_year = GPS.year;
  
  /* Initialise IMU */
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

  updateAccel();
  updateGyro();
  updateMag();
  updateBpm();
}

void loop(void){
  char c = GPS.read();
  if (!digitalRead(deadSwitchPin)) {
    logToSD( String((int)(millis()-mission_timer)) + " , Dead Switch is Connected.");
  }
  if (GPS.newNMEAreceived()){
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }
  if (timer > millis())  timer = millis();
  
  if (millis() - timer > gps_ping_time) { 
    timer = millis();

    updateAccel();
    updateGyro();
    updateMag();
    updateBpm();
    
    gps_hour =        GPS.hour;
    gps_minute =      GPS.minute;
    gps_second =      GPS.seconds;
    gps_millisecond = GPS.milliseconds;  
    gps_day =        GPS.day;
  
    gps_fix =         GPS.fix;
    gps_fix_quality = GPS.fixquality;
    gps_satellites =    (int)GPS.satellites;
    //Serial.print("Fix: ");Serial.print(gps_fix),Serial.print(", ");Serial.println(gps_fix);
    logToSD( String((int)(millis()-mission_timer)) +" , "+ String(gps_fix) +" , "+ String(gps_fix_quality)+" , "+ String((int)gps_satellites));
    if (gps_fix){
      gps_latitude =      GPS.latitude;
      gps_longitude =     GPS.longitude;
      gps_lat =           GPS.lat;
      gps_lon =           GPS.lon;
      gps_latitude_deg =  GPS.latitudeDegrees;
      gps_longitude_deg = GPS.longitudeDegrees;
      gps_angle =         GPS.angle;
      gps_speed =         GPS.speed; //knots
      gps_altitude =      GPS.altitude;
      
      //Serial.print("GPS position: ");Serial.print(gps_latitude,DEC);Serial.print(" ,");Serial.println(gps_longitude, DEC);
      //Serial.print("GPS position deg: ");Serial.print(gps_latitude_deg,DEC);Serial.print(" ,");Serial.println(gps_longitude_deg, DEC);
      //Serial.print("GPS angle: ");Serial.println(gps_angle);
      //Serial.print("GPS speed: ");Serial.println(gps_speed);
      //Serial.print("GPS altitude: ");Serial.println(gps_altitude);
      //Serial.print("IMU mag heading angle: ");Serial.println(atan2(imu_mag_y,imu_mag_x));

      // convert longitude to x pos and latitude to y pos
      double x = r_earth*toRadians(gps_target_latitude_deg-gps_latitude_deg);
      double y = r_earth*toRadians(gps_target_longitude_deg-gps_longitude_deg);
      
      //Serial.print("Distance to target: ");Serial.println(getDist1D(x,y)*1000, DEC);
      //Serial.print("GPS target position difference: ");Serial.print(x, DEC);Serial.print(", ");Serial.println(y, DEC);
      //Serial.print("GPS target position angle: ");Serial.println(atan2(x,y),DEC);//atan2( x, y), DEC);
      
      // use x,y pos and magnetometer to get angle to target
      double angle_delta = atan2( x, y)-atan2(imu_mag_x,imu_mag_y);
      if(angle_delta>pi){
        angle_delta = -pi-pi+angle_delta;
      }
      if(angle_delta<-pi){
        angle_delta = pi+pi+angle_delta;
      }
      //Serial.print("Angle to target: ");Serial.println(angle_delta, DEC);
      logToSD( String((int)(millis()-mission_timer)) +" , "+ String(gps_latitude_deg,DEC) +" , "+  String(gps_longitude_deg,DEC) +" , "+ String(gps_altitude) +" , "+ String((float)angle_delta) + " , " + String((float)getDist1D(x,y)*1000.0));
      float angle_thresh = 0.4;
      float dist_thresh = 0.001;

      // basic direction to target location
      /*
      if(getDist1D(x,y)> dist_thresh){
        if(angle_delta<0){
          if(angle_delta > -angle_thresh){
            Serial.println("Go Forwards");
          }else{
            Serial.println("Turn Right");
          }
        }else{
          if(angle_delta < angle_thresh){
            Serial.println("Go Forwards");
          }else{
            Serial.println("Turn Left");
          }
        }
      }else{
        Serial.println("Over Target");
      }
      
    */
    }
  }
}

//-----End-----//                                                                                                                                                                                                                                                                                                                                                                       

