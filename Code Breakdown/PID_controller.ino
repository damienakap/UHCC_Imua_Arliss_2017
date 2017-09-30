
/*
 * This is the Proportional-Integral-derivative(PID) controller.
 * There are some improvements that are needed.
 * 
 * There is a pid controller for each axis(pitch,roll,yaw).
 * 
 * P-Controller: Output proportional to change in instantaneous
 *  angle.
 *  Ex. If the quadcoppter instantaneously rotates 45 degrees.
 *  the p controller output is (45deg)*P_gain
 *  
 * I-Controller: Output proportional to the difference in the 
 * angle between the quadcoppter and the upward vector due to 
 * gravity
 * Ex. Let the up vector allways be 0 degrees. If is at a 90 degree
 * angle to the up vector, the I controller outsput is (90deg)*I_gain
 * 
 * D-Controller: Output proportional to change in angular rate.
 *  EX. If the quadcoppter is rotating at 1rpm and pushed to 
 *  rotate at 4rmp. The instantaneos output of the d-controller
 *  is proportional to (4rpm - 1rpm)*D_gain.
 * 
 * Adding the controller output gives the Full PID output.
 * 
 * Notes:
 * 
 * The gain settings are functional, but there may be some
 * funkyness with the controller itself, likely due to how 
 * the controller uses angular rates.
 * 
 * The goal was to be able to set an angle that the controller
 * would adjust for, but this may set high values for the p and
 * d controller when the angle is changed. 
 * (possible solution): ignore p and d of 2 logic tick(loops) 
 *    when adding the values to the pid output for each axis.
 *    allowing only the i controller to be active.
 * 
 * due to the asymmetry in motor thrust that exists, the quadcoppter
 * will always have an initial rotation on start.
 * (quick fix for dtesting): hold the quadcoppter for 5 seconds when 
 *  the motors start, then release the quadcopter for stable flight.
 * (possible solution): idividually test the thrust of each motor on
 *  a scale and input an offset.
 * 
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

#include <Servo.h>
#include <math.h>

const double pi = 3.1415926535897932384626433832795;

Servo ESC0;
Servo ESC1;
Servo ESC2;
Servo ESC3;

static int done = 0;
static uint32_t timer = millis();

static uint32_t loop_time_delta = millis();
static uint32_t pid_time_last = micros();

// Calibration order
//d(start high, lower till smooth, then lower by 25%)
//p(increase by 0.2 until unstable, lower by 50%)
//i(increase by 0.01 until ocillates, lower by 50%)
//p(increase slowly until fast oscilation, lower by a few steps) **

const int speed_hover = 1380;

const uint32_t test_run_time = 360000; // test time in milliseconds
const uint32_t test_land_time = 0;

static bool ESC_is_active = true;

static int dont = 0;
// Yaw
const float pid_p_gain_yaw = 32.0;  //Gain setting for the yaw P-controller.
const float pid_i_gain_yaw = 0;     //Gain setting for the yaw I-controller.
const float pid_d_gain_yaw = 0.0;   //Gain setting for the yaw D-controller.
const float pid_i_max_yaw = 200;
const int pid_max_yaw = 200;        //Maximum output of the yaw PID-controller (+/-)
// Pitch
const float pid_p_gain_pitch = 12.0;  //Gain setting for the pitch P-controller.
const float pid_i_gain_pitch = 90.0;  //Gain setting for the pitch I-controller.
const float pid_d_gain_pitch = 0.14;  //Gain setting for the pitch D-controller.
const float pid_i_max_pitch = 400;
const int pid_max_pitch = 400;        //Maximum output of the pitch PID-controller (+/-)
// Roll
float pid_p_gain_roll = pid_p_gain_pitch;   //Gain setting for the roll P-controller.
float pid_i_gain_roll = pid_i_gain_pitch;   //Gain setting for the roll I-controller.
float pid_d_gain_roll = pid_d_gain_pitch;   //Gain setting for the roll D-controller.
float pid_i_max_roll = pid_i_max_pitch;
int pid_max_roll = pid_max_pitch;           //Maximum output of the roll PID-controller (+/-)

// desired movement in rads/s
// Yaw
static float pid_out_yaw = 0;               // pid output for yaw
static float pid_delta_last_yaw = 0;        // last yaw rate difference between gyro and input rate
static float pid_set_yaw = 0;               // in rads (set yaw from center)
static float pid_gyro_input_yaw = 0;
static float pid_delta_rate_last_yaw = 0;
static float pid_gyro_total_yaw = 0;        // in rads (total yaw from center)
static float pid_accel_yaw = 0;             // in rads (total yaw from accelerometer)
// Pitch
static float pid_out_pitch = 0;              // pid output for pitch
static float pid_delta_last_pitch = 0;       // last pitch rate difference between gyro and input rate
static float pid_set_pitch = (15*pi/180);    // in rads (set pitch from center) -back +forward
static float pid_gyro_input_pitch = 0;
static float pid_delta_rate_last_pitch = 0;
static float pid_gyro_total_pitch = 0.0;      // in rads (total pitch from center)
static float pid_accel_pitch = 0;             // in rads (total pitch from accelerometer)
// Roll
static float pid_out_roll = 0;                // pid output for roll
static float pid_delta_last_roll = 0;         // last roll rate difference between gyro and input rate
static float pid_set_roll = 0;                // in rads (set roll from center) -left +right
static float pid_gyro_input_roll = 0;
static float pid_delta_rate_last_roll = 0;
static float pid_gyro_total_roll = 0.0;       // in rads (total roll from center)
static float pid_accel_roll = 0;              // in rads (total roll from accelerometer)

static int pid_thrust = 0;
static bool pid_is_inverted = false;
// correct gyro with accelorometer for pitch/roll and
// magnetometer for yaw 
const float pid_gyro_correction_factor = 0.002; // set to zero to disable

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

//-----Get Sensor Value Range-----
sensor_t sensor;
sensors_event_t event;

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

float updateAltitude(){
  bmp.getEvent(&event);
  bmp.getTemperature(&imu_bmp_temp);
  return bmp.pressureToAltitude(imu_seaLevelPressure,event.pressure,imu_bmp_temp);
}

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
  
  float pid_time_delta = (micros() - pid_time_last); // milisec to sec
  
  if(pid_time_delta >= 4000){
    pid_time_delta /= 1000000;
    pid_time_last = micros();

  
  if(dont<2){
    // Corrects for imu inacurrate initial values
    dont+=1;
    pid_thrust = 1000;
  }else{
    // Integrate gyro angle
    pid_gyro_total_yaw += (imu_gyro_z*pid_time_delta);
    pid_gyro_total_pitch += (imu_gyro_y*pid_time_delta);
    pid_gyro_total_roll += (imu_gyro_x*pid_time_delta);
    
    // accelerometer to correct gyro angle drift
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
        
        //Serial.print(pid_gyro_total_pitch,DEC);Serial.print(" : ");Serial.println(atan2(-imu_accel_x,imu_accel_z), DEC);
        //Serial.print(pid_gyro_total_roll,DEC);Serial.print(" : ");Serial.println(atan2(imu_accel_y,imu_accel_z), DEC);
      }
    }

    // yawed transfer pitch <-> roll
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
    float pid_i_out_roll = (float)pid_delta_roll * (float)pid_i_gain_roll;
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
  //compensation value for lower battery voltage
  int batMulti = 1000 - (1000*batVoltage/12.6);
  ESC0.writeMicroseconds((pid_thrust + pid_out_pitch - pid_out_roll - pid_out_yaw + batMulti));
  ESC1.writeMicroseconds((pid_thrust + pid_out_pitch + pid_out_roll + pid_out_yaw + batMulti));
  ESC2.writeMicroseconds((pid_thrust - pid_out_pitch - pid_out_roll + pid_out_yaw + batMulti));
  ESC3.writeMicroseconds((pid_thrust - pid_out_pitch + pid_out_roll - pid_out_yaw + batMulti));
  Serial.print("Bat: ");Serial.print(batVoltage);Serial.print(" Bat multi: ");Serial.print(batMulti);Serial.print(" Roll out: ");Serial.print(pid_out_roll);Serial.print(" P: ");Serial.print(pid_p_out_roll);Serial.print(" I: ");Serial.print(pid_i_out_roll);Serial.print(" D: ");Serial.println(pid_d_out_roll);
  //Serial.print("Bat: ");Serial.print(batVoltage);Serial.print(" Yaw out: ");Serial.print(pid_out_yaw);Serial.print(" P: ");Serial.print(pid_p_out_yaw);Serial.print(" I: ");Serial.print(pid_i_out_yaw);Serial.print(" D: ");Serial.println(pid_d_out_yaw);
  //Serial.print("Bat: ");Serial.print(batVoltage);Serial.print(" Pitch out: ");Serial.print(pid_out_pitch);Serial.print(" P: ");Serial.print(pid_p_out_pitch);Serial.print(" I: ");Serial.print(pid_i_out_pitch);Serial.print(" D: ");Serial.println(pid_d_out_pitch);
    
  }
  return;
}

void setup() {
  Serial.begin(115200);
  ESC0.attach(10);//front left CW
  ESC1.attach(11);//front right CCW
  ESC2.attach(12);//back left CCW
  ESC3.attach(13);//back right CW
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
  imu_accel_max = sensor.max_value
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
  
}

void loop() {
  
  if(!done){
    delay(1000);
    Serial.println("Go");
    delay(1000);
    timer = millis();
    while(millis()-timer<test_run_time+test_land_time){
      pid_thrust = speed_hover;
      pidCalculate();
    }
    done = true;
  }else{
    Serial.println("Done");
    ESC0.writeMicroseconds(1000);
    ESC1.writeMicroseconds(1000);
    ESC2.writeMicroseconds(1000);
    ESC3.writeMicroseconds(1000);
  }
  
}

