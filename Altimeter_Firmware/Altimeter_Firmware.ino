#include <Wire.h>
#include <MS5x.h> //COMMENT OUT LINE 24 IN HEADER FILE, THIS LIBRARY CAN BE FOUND ON THE LIBRARY MANAGER

uint8_t launchState = 0; //Assume unlaunched

MS5x barometer(&Wire); //I do not understand what &Wire means but I know this has something to do with identifying the I2C pins

void launchStatus() { //Uses barometer only, accelerometer definitely useful for this application
  uint32_t time1 = millis();
  barometer.checkUpdates();
  uint16_t alt1 = barometer.getAltitude();
  delay(100); //Placeholder, could be made smaller/larger
  barometer.checkUpdates();
  uint16_t alt2 = barometer.getAltitude();
  uint16_t deltat = millis() - time1;
  avgV = (alt1 - alt2) / deltat
  if (avgV > 10) { //Placeholder avgV threshold, could be made smaller/larger
    launchState = 1;
  } else {
    launchState = 0;
  }
}
