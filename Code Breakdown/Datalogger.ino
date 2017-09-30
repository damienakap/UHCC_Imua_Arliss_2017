
#include <SPI.h>
#include <SD.h>

const int chipSelect = 4; // the sd cards SPI connection

// needed for Serial to work on Feather M0 Adalogger
#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

void setup() {
  Serial.begin(9600);

  // initialize sd card
  while (!Serial) {} // wait for serial
  
  Serial.print("Initializing SD card...");
  if(!SD.begin(chipSelect))Serial.println("Card failed, or not present");
  Serial.println("card initialized.");
}

void loop() {
  String dataString = "";
  // read voltage and convert to string
  for (int analogPin = 0; analogPin < 3; analogPin++) {
    int sensor = analogRead(analogPin);
    dataString += String(sensor);
    if (analogPin < 2) {
      dataString += ",";
    }
  }

  
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  // if the file exists, print string to file
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.flush();
    dataFile.close();
    Serial.println(dataString);
  }else{
    Serial.println("error opening datalog.txt");
  }
}









