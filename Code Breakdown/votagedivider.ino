
// voltage divider

const int analogInPin = A4;  // Analog input pin to voltage divider

int sensorValue = 0;        // value read from the pot
float outputValue = 0;      // output voltage of voltage divider
const float r1 = 2.55;      // top resistor
const float r2 = 1;         // bottom resistor
const float d1 = 0.6;       // voltage lost from diode

/*  Voltage divider diagram
 * battery                   battery
 *  (+V)--(D1)--R1--o--R2--o--(-V)
 *                  |      |
 *                 (A4)   (GND)
 *                    Arduino
 */

void setup() {
  Serial.begin(11500);
}

void loop() {
  // read voltage analoge value (0-1023)
  sensorValue = analogRead(analogInPin);
  // convert to read voltage
  outputValue = 3.3*sensorValue/1023;
  //convert read voltage to source voltage
  outputValue = (outputValue*(r1+r2)/r2)+0.6;
  
  Serial.print("sensor = ");Serial.print(sensorValue);
  Serial.print("\t output = ");Serial.println(outputValue);

  delay(2);
}


