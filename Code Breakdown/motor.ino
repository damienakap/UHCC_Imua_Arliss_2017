
#include <Servo.h>


Servo ESC0;
Servo ESC1;
Servo ESC2;
Servo ESC3;

static bool done = false;
const int hover_speed = 1380;
const int ESC_Init = 500;
static uint32_t timer = millis();

void setup() {
  Serial.begin(115200);
  // motor pin assignment
  ESC0.attach(10);//front left CW
  ESC1.attach(11);//front right CCW
  ESC2.attach(12);//back left CCW
  ESC3.attach(13);//back right CW
  
  // sequence required to initialize the
  // little bee heli ESC and set the max
  // and min signal values that correspond to
  // the max and min motor speeds.
  //
  // 1000us=>0%speed 2000us=>100%speed
  
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
  // sequence end

  // this part was used to warm up motors since
  // some motors may start before others. Brings
  // them into sync.
  while(millis()-time_last < 100){
    ESC0.writeMicroseconds(1100);
    ESC1.writeMicroseconds(1100);
    ESC2.writeMicroseconds(1100);
    ESC3.writeMicroseconds(1100);
  }
  // end motor initialize
}

void loop() {
  // used to find a rough value for the 
  // hover speed. Motor speed affected 
  // by battery voltage.
  if(!done){
    Serial.println("Go");
    timer = millis();
    while(millis()-timer<1000){
      ESC0.writeMicroseconds(hover_speed);
      ESC1.writeMicroseconds(hover_speed);
      ESC2.writeMicroseconds(hover_speed);
      ESC3.writeMicroseconds(hover_speed);
    }
    done = true;
  }else{
    Serial.println("Done");
    ESC0.writeMicroseconds(ESC_Init);
    ESC1.writeMicroseconds(ESC_Init);
    ESC2.writeMicroseconds(ESC_Init);
    ESC3.writeMicroseconds(ESC_Init);
  }
  
}

