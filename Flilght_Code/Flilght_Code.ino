
/*
 * Due to time constaints, this code is 
 * likely to be flawed.
 * 
 * Use as a reference only.
 * 
 * line 538 for mission code
 * 
 */
 
/*Dead Switch*/
const int deadSwitchPin = 6;
static bool deadSwitchOff = false;
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
static uint32_t mission_timer = millis();
static uint32_t landingTimer = 0;
static uint32_t ejection_timer = 0;
static uint32_t gpsTimer = millis();

const double r_earth = 6371; // km
const double gps_target_latitude_deg = 40.8735733032;
const double gps_target_longitude_deg = -119.1088027954;
const double gps_ground_altitude = 1185.5999755859;

const double pi = 3.1415926535897932384626433832795;

boolean usingInterrupt = false;
void useInterrupt(boolean);

int gps_ping_time = 500; // ms

int gps_hour;
int gps_minute;
int gps_second;
int gps_millisecond;
int gps_day;
int gps_month;
int gps_year;

int gps_fix;
int gps_fix_quality;
double gps_latitude;
double gps_longitude;
float gps_lat;
float gps_lon;
double gps_latitude_deg;
double gps_longitude_deg;
float gps_angle;
float gps_speed; //knots
float gps_altitude;
float gps_satellites;

static double gps_dist_to_targ = 0.0;
static double gps_angle_delta = 0.0;
static double gps_dist_delta_x = 0.0;
static double gps_dist_delta_y = 0.0;

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

#include <Servo.h>
#include <math.h>

Servo ESC0;
Servo ESC1;
Servo ESC2;
Servo ESC3;

static int done = 0;
static uint32_t pidTimer = millis();

static uint32_t loop_time_delta = millis();
static uint32_t pid_time_last = micros();

static int speed_current = 0;
static int speed_hover = 1450;

const uint32_t test_run_time = 360000;
const uint32_t test_land_time = 0;
static bool ESC_is_active = true;

static int dont = 0;
// Yaw
const float pid_p_gain_yaw = 32.0;
const float pid_i_gain_yaw = 60;
const float pid_d_gain_yaw = 0.0;
const float pid_i_max_yaw = 200;
const int pid_max_yaw = 200;
// Pitch
const float pid_p_gain_pitch = 12.0;
const float pid_i_gain_pitch = 90.0;
const float pid_d_gain_pitch = 0.14;
const float pid_i_max_pitch = 400;
const int pid_max_pitch = 400;
// Roll
float pid_p_gain_roll = pid_p_gain_pitch;
float pid_i_gain_roll = pid_i_gain_pitch;
float pid_d_gain_roll = pid_d_gain_pitch;
float pid_i_max_roll = pid_i_max_pitch;
int pid_max_roll = pid_max_pitch;

// desired movement in rads/s
// Yaw
static float pid_out_yaw = 0;
static float pid_delta_last_yaw = 0;
static float pid_set_yaw = 0;
static float pid_gyro_input_yaw = 0;
static float pid_delta_rate_last_yaw = 0;
static float pid_gyro_total_yaw = 0;
static float pid_accel_yaw = 0;
// Pitch
static float pid_out_pitch = 0;
static float pid_delta_last_pitch = 0;
static float pid_set_pitch = 0;
static float pid_gyro_input_pitch = 0;
static float pid_delta_rate_last_pitch = 0;
static float pid_gyro_total_pitch = 0.0;
static float pid_accel_pitch = 0;
// Roll
static float pid_out_roll = 0;
static float pid_delta_last_roll = 0;
static float pid_set_roll = 0;
static float pid_gyro_input_roll = 0;
static float pid_delta_rate_last_roll = 0;
static float pid_gyro_total_roll = 0.0;
static float pid_accel_roll = 0;

static int pid_thrust = 0;
static bool pid_is_inverted = false;
const float pid_gyro_correction_factor = 0.002;
//battery monitor vars
const int batVoltagePin = A4;
const float r1 = 2.55;
const float r2 = 1;
const float diodeComp = 0.6;
static float batVoltage = 0.0;

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

//imu info vars
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
//imu values
static float imu_accel_x;
static float imu_accel_y;
static float imu_accel_z;
static float imu_accel_calib_x = 0.0;
static float imu_accel_calib_y = 0.0;
static float imu_accel_calib_z = 0.0;

static float imu_gyro_x;
static float imu_gyro_y;
static float imu_gyro_z;
static float imu_gyro_calib_x = 0.0;
static float imu_gyro_calib_y = 0.0;
static float imu_gyro_calib_z = 0.0;

static float imu_mag_x;
static float imu_mag_y;
static float imu_mag_z;
static float imu_bmp_pressure;
static float imu_bmp_temp;
static float imu_seaLevelPressure;

sensor_t sensor;
sensors_event_t event;

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

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


double toRadians(double deg){
  return (deg/(double)180.0)*pi;
}

double getDist1D(double x, double y){
  return pow(pow(x,2)+pow(y,2),0.5);
}

float updateAltitude(){
  bmp.getEvent(&event);
  bmp.getTemperature(&imu_bmp_temp);
  return bmp.pressureToAltitude(imu_seaLevelPressure,event.pressure,imu_bmp_temp);
}
//6977071013
void updateAccel(){
  accel.getEvent(&event);
  imu_accel_x = event.acceleration.x + imu_accel_calib_x;
  imu_accel_y = event.acceleration.y + imu_accel_calib_y;
  imu_accel_z = event.acceleration.z + imu_accel_calib_z;
  return;
}

void updateMag(){
  mag.getEvent(&event);
  imu_mag_x = event.magnetic.x;
  imu_mag_y = event.magnetic.y;
  imu_mag_z = event.magnetic.z;
  return;
}

void updateGyro(){
  gyro.getEvent(&event);
  imu_gyro_x = event.gyro.x - imu_gyro_calib_x;
  imu_gyro_y = event.gyro.y - imu_gyro_calib_y;
  imu_gyro_z = event.gyro.z - imu_gyro_calib_z;
  return;
}

void updateBpm(){
  bmp.getEvent(&event);
  imu_bmp_pressure = event.pressure;
  imu_bmp_temp; bmp.getTemperature(&imu_bmp_temp);
  imu_seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
  return;
}

void updateBatVoltage(){
  batVoltage = ((r1+r2)*analogRead(batVoltagePin)*3.3/1023/r2)+diodeComp;
  if(batVoltage < 1.0) batVoltage = 0;
  return;
}
void pidResetAngles(){
  pid_gyro_total_pitch = 0;
  pid_delta_last_pitch = 0;
}
void pidCalculate(){
  updateGyro();
  updateAccel();
  updateMag();
  float pid_time_delta = (micros() - pid_time_last);
  
  if(pid_time_delta >= 4000){
    pid_time_delta /= 1000000;
    pid_time_last = micros();
    
  // Integrate gyro angle
  if(dont<2){
    dont+=1;
    pid_thrust = 1000;
  }else{
    pid_gyro_total_yaw += (imu_gyro_z*pid_time_delta);
    pid_gyro_total_pitch += (imu_gyro_y*pid_time_delta);
    pid_gyro_total_roll += (imu_gyro_x*pid_time_delta);
    
    pid_gyro_total_pitch -= pid_gyro_total_roll * sin(imu_gyro_z*pid_time_delta);
    pid_gyro_total_roll += pid_gyro_total_pitch * sin(imu_gyro_z*pid_time_delta);
    
    if(pid_gyro_correction_factor > 0){
      if(abs(imu_mag_y)>0 || abs(imu_mag_x)>0){
        pid_gyro_total_yaw *= (1-pid_gyro_correction_factor);
        pid_gyro_total_yaw += ((float)atan2(-imu_mag_y,imu_mag_x)*pid_gyro_correction_factor);
      }
      if(abs(imu_accel_x)>0 || abs(imu_accel_z)>0){
        pid_gyro_total_pitch *= ((float)1.0-pid_gyro_correction_factor);
        pid_gyro_total_pitch += ((float)atan2(-imu_accel_x,imu_accel_z)*pid_gyro_correction_factor);
        pid_gyro_total_roll *= ((float)1.0-pid_gyro_correction_factor);
        pid_gyro_total_roll += ((float)atan2(imu_accel_y,imu_accel_z)*pid_gyro_correction_factor);  
      }
      
    }
    pid_gyro_total_pitch -= pid_gyro_total_roll * sin(imu_gyro_z*pid_time_delta);
    pid_gyro_total_roll += pid_gyro_total_pitch * sin(imu_gyro_z*pid_time_delta);
    
  }
  
  // Delta angle
  float pid_delta_yaw = (pid_gyro_total_yaw - pid_set_yaw);
  float pid_delta_pitch = (pid_gyro_total_pitch - pid_set_pitch);
  float pid_delta_roll = (pid_gyro_total_roll - pid_set_roll);
  // Angle rates
  float pid_delta_rate_yaw = 0;
  float pid_delta_rate_pitch = 0;
  float pid_delta_rate_roll = 0;
  
  pid_delta_rate_yaw = (pid_delta_yaw - pid_delta_last_yaw)/pid_time_delta;
  pid_delta_rate_pitch = (pid_delta_pitch - pid_delta_last_pitch)/pid_time_delta;
  pid_delta_rate_roll = (pid_delta_roll - pid_delta_last_roll)/pid_time_delta;
  
  // PID Controller Start
    // Yaw
    float pid_p_out_yaw = pid_delta_rate_yaw*pid_p_gain_yaw;
    float pid_i_out_yaw = pid_delta_yaw * pid_i_gain_yaw;
    float pid_d_out_yaw = (pid_delta_rate_yaw - pid_delta_rate_last_yaw)*pid_d_gain_yaw/pid_time_delta;
    pid_out_yaw = (pid_p_out_yaw + pid_i_out_yaw + pid_d_out_yaw);
    // clip yaw between +/- pid_max_yaw
    if(pid_out_yaw>pid_max_yaw){pid_out_yaw=pid_max_yaw;}
    if(pid_out_yaw<-pid_max_yaw){pid_out_yaw=-pid_max_yaw;}

    pid_delta_rate_last_yaw = pid_delta_rate_yaw;
    pid_delta_last_yaw = pid_delta_yaw;
    
    // Pitch
    float pid_p_out_pitch = pid_delta_rate_pitch*pid_p_gain_pitch;
    float pid_i_out_pitch = pid_delta_pitch * pid_i_gain_pitch;
    float pid_d_out_pitch = (pid_delta_rate_pitch - pid_delta_rate_last_pitch)*pid_d_gain_pitch/pid_time_delta;
    if(pid_i_out_pitch>pid_max_pitch){pid_i_out_pitch=pid_max_pitch;}
    if(pid_i_out_pitch<-pid_max_pitch){pid_i_out_pitch=-pid_max_pitch;}
    
    pid_out_pitch = (pid_p_out_pitch + pid_i_out_pitch + pid_d_out_pitch);
    if(pid_out_pitch>pid_max_pitch){pid_out_pitch=pid_max_pitch;}
    if(pid_out_pitch<-pid_max_pitch){pid_out_pitch=-pid_max_pitch;}

    pid_delta_rate_last_pitch = pid_delta_rate_pitch;
    pid_delta_last_pitch = pid_delta_pitch;
    // Roll
    float pid_p_out_roll = pid_delta_rate_roll*pid_p_gain_roll;
    float pid_i_out_roll = pid_delta_roll * pid_i_gain_roll;
    float pid_d_out_roll = (pid_delta_rate_roll - pid_delta_rate_last_roll)*pid_d_gain_roll/pid_time_delta;
    if(pid_i_out_roll>pid_max_pitch){pid_i_out_roll=pid_max_pitch;}
    if(pid_i_out_roll<-pid_max_pitch){pid_i_out_roll=-pid_max_pitch;}
    
    pid_out_roll = (pid_p_out_roll + pid_i_out_roll + pid_d_out_roll);
    if(pid_out_roll>pid_max_roll){pid_out_roll=pid_max_roll;}
    if(pid_out_roll<-pid_max_roll){pid_out_roll=-pid_max_roll;}

    pid_delta_rate_last_roll = pid_delta_rate_roll;
    pid_delta_last_roll = pid_delta_roll;
  // PID End
  
  updateBatVoltage();
  int batMulti = 1000 - (1000*batVoltage/12.6);
  ESC0.writeMicroseconds((pid_thrust + pid_out_pitch - pid_out_roll - pid_out_yaw + batMulti));
  ESC1.writeMicroseconds((pid_thrust + pid_out_pitch + pid_out_roll + pid_out_yaw + batMulti));
  ESC2.writeMicroseconds((pid_thrust - pid_out_pitch - pid_out_roll + pid_out_yaw + batMulti));
  ESC3.writeMicroseconds((pid_thrust - pid_out_pitch + pid_out_roll - pid_out_yaw + batMulti));
  }
  return;
}

void setup() {
  Serial.begin(115200);

  pinMode(deadSwitchPin, INPUT_PULLUP);

  Serial.print("Initializing SD card...");
  if(!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
  }
  Serial.println("card initialized.");

  logToSD("New Run\n\n");
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(2000);
  mySerial.println(PMTK_Q_RELEASE);
  delay(1000);
  gps_hour = GPS.hour;
  gps_minute = GPS.minute;
  gps_second = GPS.seconds;
  gps_millisecond = GPS.milliseconds;
  gps_day = GPS.day;
  gps_month = GPS.month;
  gps_year = GPS.year;
  
  ESC0.attach(10);
  ESC1.attach(11);
  ESC2.attach(12);
  ESC3.attach(13);
  double time_last = millis();
  while(millis()-time_last < 6000){
    ESC0.writeMicroseconds(2000);
    ESC1.writeMicroseconds(2000);
    ESC2.writeMicroseconds(2000);
    ESC3.writeMicroseconds(2000);
  }
  while(millis()-time_last < 10000){
    ESC0.writeMicroseconds(1000);
    ESC1.writeMicroseconds(1000);
    ESC2.writeMicroseconds(1000);
    ESC3.writeMicroseconds(1000);
  }
  while(millis()-time_last < 100){
    ESC0.writeMicroseconds(1100);
    ESC1.writeMicroseconds(1100);
    ESC2.writeMicroseconds(1100);
    ESC3.writeMicroseconds(1100);
  }

  updateBatVoltage();
  
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
  
  accel.getSensor(&sensor);
  imu_accel_max = sensor.max_value;
  imu_accel_min = sensor.min_value;
  imu_accel_res = sensor.resolution;
  
  gyro.getSensor(&sensor);
  imu_gyro_max = sensor.max_value;
  imu_gyro_min = sensor.min_value;
  imu_gyro_res = sensor.resolution;
  
  mag.getSensor(&sensor);
  imu_mag_max = sensor.max_value;
  imu_mag_min = sensor.min_value;
  imu_mag_res = sensor.resolution;

  bmp.getSensor(&sensor);
  imu_bmp_max = sensor.max_value;
  imu_bmp_min = sensor.min_value;
  imu_bmp_res = sensor.resolution;

  updateAccel();
  updateGyro();
  updateMag();
  updateBpm();
  pidTimer = millis();
  mission_timer = millis();
  gpsTimer = millis();
  deadSwitchOff = digitalRead(deadSwitchPin);
}

void loop() {
  deadSwitchOff = digitalRead(deadSwitchPin);
  char c = GPS.read();
  
  if (GPS.newNMEAreceived()){
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }
  if (gpsTimer > millis())  gpsTimer = millis();
  
  if (millis() - gpsTimer > gps_ping_time) { 
    gpsTimer = millis();

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
    logToSD( String((int)(millis()-mission_timer)) +" , Fix , "+ String((int)gps_fix) +" , "+ String((int)gps_fix_quality)+" , "+ String((int)gps_satellites)+" , "+String((float)batVoltage) +" , "+String((int)deadSwitchOff));
    logToSD( String((int)(millis()-mission_timer)) +" , data , "+ String(imu_bmp_pressure, DEC) +" , "+ String(imu_bmp_temp, DEC));
    if(gps_fix){
      gps_latitude =      GPS.latitude;
      gps_longitude =     GPS.longitude;
      gps_lat =           GPS.lat;
      gps_lon =           GPS.lon;
      gps_latitude_deg =  GPS.latitudeDegrees;
      gps_longitude_deg = GPS.longitudeDegrees;
      gps_angle =         GPS.angle;
      gps_speed =         GPS.speed; //knots
      gps_altitude =      GPS.altitude;
      gps_satellites =    GPS.satellites;
      
      gps_dist_delta_y = r_earth*toRadians(gps_target_latitude_deg-gps_latitude_deg);
      gps_dist_delta_x = r_earth*toRadians(gps_target_longitude_deg-gps_longitude_deg);
      gps_angle_delta = atan2( gps_dist_delta_x, gps_dist_delta_y)-atan2(imu_mag_x,imu_mag_y);
      if(gps_angle_delta>pi){
        gps_angle_delta = -pi-pi+gps_angle_delta;
      }
      if(gps_angle_delta<-pi){
        gps_angle_delta = pi+pi+gps_angle_delta;
      }
      gps_dist_to_targ = (float)getDist1D(gps_dist_delta_x,gps_dist_delta_y)*1000.0;
      Serial.print("Dist delta ");Serial.print(gps_dist_delta_x*1000.0);Serial.print(" , ");Serial.println(gps_dist_delta_y*1000.0);
      logToSD( String((int)(millis()-mission_timer)) +" , GPS , "+ String(gps_latitude_deg, DEC) +" , "+  String(gps_longitude_deg,DEC) +" , "+ String(gps_altitude, DEC) +" , "+ String((float)gps_angle_delta) + " , " + String(gps_dist_to_targ));
      logToSD( String((int)(millis()-mission_timer)) +" , accel , "+ String(imu_accel_x) +" , "+ String(imu_accel_y)+" , "+ String(imu_accel_z) );
    }
  }
  
  // Mission Code
  /* 
   *  This code is likely flawed. Use as reference only.
   *  
   * logic:
   *  - wait for dead switch.
   *  - wait 10s
   *  - increase motors from 0% to 30% (1000 to 1300)
   *    over 10s
   *  - run motors at 35% for 30s
   *  - run motors at 35% until gps lock
   *  - on gps lock got to target pos and stay at
   *    altitue 200m.
   *  - if 4m from target, set assigned angle and 
   *    motor thrust to be proportional to the distance 
   *    to target.
   *  - if at ground altitue for 2 minutes, turn off motors.
   *    
   */
  if(!done){
    if(deadSwitchOff){
      if(ejection_timer <= 0){
        ejection_timer = millis();
      }
      uint32_t t = millis() - ejection_timer;
      if(t < 10000){
        pid_thrust = 1000;
        ESC0.writeMicroseconds(1000);
        ESC1.writeMicroseconds(1000);
        ESC2.writeMicroseconds(1000);
        ESC3.writeMicroseconds(1000);
      }else if(t<20000){
        pid_thrust = 1000 + (300*(t-1000)/1000);
        if(pid_thrust>1300)pid_thrust=1300;
        ESC0.writeMicroseconds(pid_thrust);
        ESC1.writeMicroseconds(pid_thrust);
        ESC2.writeMicroseconds(pid_thrust);
        ESC3.writeMicroseconds(pid_thrust);
      }else if(t < 30000){
        pid_thrust=1350;
        pidCalculate(); 
      }else{
        if(gps_fix){
          double cruse_altitude = gps_ground_altitude + 200;
          if(gps_dist_to_targ < 10){
            cruse_altitude = gps_ground_altitude + (200*gps_dist_to_targ/10) + 1;
          }
          
          if(gps_dist_to_targ <= 2 || gps_altitude >  cruse_altitude + 0.5){
            pid_thrust=1350;
          }else if(gps_altitude < cruse_altitude - 0.5){
            pid_thrust=1500;
          }else{
            pid_thrust=1450;
          }

          if(gps_dist_delta_x < 0){
            if(gps_dist_delta_x>-4.0){
              pid_set_roll = toRadians(-15)*(gps_dist_delta_x/(-4));
            }else{
              pid_set_roll = toRadians(-15);
            }
          }else if(gps_dist_delta_x > 0){
            if(gps_dist_delta_x<4.0){
              pid_set_roll = toRadians(15)*(gps_dist_delta_x/4);
            }else{
              pid_set_roll = toRadians(15);
            }
          }

          if(gps_dist_delta_y < 0){
            if(gps_dist_delta_y>-4.0){
              pid_set_pitch = toRadians(-15)*(gps_dist_delta_y/(-4));
            }else{
              pid_set_pitch = toRadians(-15);
            }
          }else if(gps_dist_delta_y > 0){
            if(gps_dist_delta_y<4.0){
              pid_set_pitch = toRadians(15)*(gps_dist_delta_y/4);
            }else{
              pid_set_pitch = toRadians(15);
            }
          }
          if(gps_dist_to_targ < 2 && gps_altitude <= gps_ground_altitude + 0.5 ){
            if(landingTimer<=0){landingTimer=millis();}
            if(millis()-landingTimer > 120000){
              done = true;
              ESC0.writeMicroseconds(1000);
              ESC1.writeMicroseconds(1000);
              ESC2.writeMicroseconds(1000);
              ESC3.writeMicroseconds(1000);
            }else{
              pidCalculate();
            }
          }else{
            landingTimer = 0;
            pidCalculate();
          }
        }else{
          pid_thrust=1350;
          pidCalculate();
        }
      }
    }else{
      ESC0.writeMicroseconds(1000);
      ESC1.writeMicroseconds(1000);
      ESC2.writeMicroseconds(1000);
      ESC3.writeMicroseconds(1000);
    }
  }else{
    Serial.println("Done");
    ESC0.writeMicroseconds(1000);
    ESC1.writeMicroseconds(1000);
    ESC2.writeMicroseconds(1000);
    ESC3.writeMicroseconds(1000);
  }
  // Mission Code END
}

