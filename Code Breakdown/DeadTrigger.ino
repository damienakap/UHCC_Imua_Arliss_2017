
/*
 *      Arduino
 *  (6)--o o--(GND)
 *        /
 *    Dead Switch
 */

const int DeadSwitchPin = 6;

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

void setup() {
  Serial.begin(DeadSwitchPin);
  pinMode(buttonPin, INPUT_PULLUP); // use pull-up resistor input
}

void loop() {
  if (!digitalRead(DeadSwitchPin)) {
    Serial.println("Connected");
  } else {
    Serial.println("Disconnected");
  }
  delay(1000);
}
