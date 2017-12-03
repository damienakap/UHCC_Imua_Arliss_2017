// derivative of the example code

#include <Adafruit_GPS.h>

#define mySerial Serial1

Adafruit_GPS GPS(&mySerial);

#define GPSECHO  true

static boolean usingInterrupt = false;
void useInterrupt(boolean);

static int gps_ping_time = 500; // ms

static int gps_hour;
static int gps_minute;
static int gps_second;
static int gps_millisecond;
static int gps_day;
static int gps_month;
static int gps_year;

static bool gps_fix;
static int gps_fix_quality;
static float gps_latitude;
static float gps_longitude;
static float gps_lat;
static float gps_lon;
static float gps_latitude_deg;
static float gps_longitude_deg;
static float gps_angle;
static float gps_speed; //knots
static float gps_altitude;
static float gps_satellites;

// needed for Serial to work on the Feather M0 Adalogger
#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

void setup()  
{
  Serial.begin(115200);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
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
  
}






uint32_t timer = millis();

void loop(){
    // need to 'hand query' the GPS and
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // Print raw string
    if (GPSECHO)
      if (c) Serial.print(c);
  
  if (GPS.newNMEAreceived()){
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;
  }
  if (timer > millis())  timer = millis();

  // called when the time delta reaches the gps_ping_time(500ms)
  if (millis() - timer > gps_ping_time) { 
    timer = millis(); // reset the timer

    gps_hour =        GPS.hour;
    gps_minute =      GPS.minute;
    gps_second =      GPS.seconds;
    gps_millisecond = GPS.milliseconds;  
    gps_day =        GPS.day;
  
    gps_fix =         GPS.fix;
    gps_fix_quality = GPS.fixquality;
    Serial.print("Fix: ");Serial.print(gps_fix);Serial.print(", ");Serial.println(gps_fix_quality);
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
      gps_satellites =    (int)GPS.satellites;
      Serial.print("GPS position: ");Serial.print(gps_latitude,DEC);Serial.print(" ,");Serial.println(gps_longitude);
      Serial.print("GPS position deg: ");Serial.print(gps_latitude_deg,DEC);Serial.print(" ,");Serial.println(gps_longitude_deg);
      Serial.print("GPS angle: ");Serial.println(gps_angle);
      Serial.print("GPS speed: ");Serial.println(gps_speed);
      Serial.print("GPS altitude: ");Serial.println(gps_altitude);
    }
  }
  
}
