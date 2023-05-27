#include <Wire.h>
#include <MS5x.h> //COMMENT OUT LINE 24 IN HEADER FILE, THIS LIBRARY CAN BE FOUND ON THE LIBRARY MANAGER

uint8_t launchState = 0; //Assume unlaunched

MS5x barometer(&Wire); //I do not understand what &Wire means but I know this has something to do with identifying the I2C pins
Serial.begin(9600); //Tentative

void launchStatus() { //Uses barometer only, accelerometer definitely better for this application, also bad algorithm in general
  uint16_t alttemp1 = 1; //Set higher than second measured altitude to ensure that second altitude is at least 1m higher on average
  uint16_t alttemp2 = 0; 
  uint8_t countr1 = 0;
  uint8_t countr2 = 0;
  for (uint8_t i = 0; i < 50; i++)  { //Tries to take 50 altitude value average
    barometer.checkUpdates();
    if (barometer.isReady()) {
      alttemp1 += barometer.getAltitude();
      countr1 += 1;
    }
  }
  alttemp1 /= countr1;

  delay(100) //delay to allow significant altitude increase

  for (uint8_t i = 0; i < 50; i++)  { //Tries to take 50 altitude value average
    barometer.checkUpdates();
    if (barometer.isReady()) {
      alttemp2 = barometer.getAltitude();
      countr2 += 1;
    }
  }
  alttemp2 /= countr2;

  if (alttemp2 > alttemp1) { //Compare altitude values to see if there was a positive change (in flight) or not (on pad)
    launchState = 1;
    Serial.println("Rocket in flight")
  } else {
    launchState = 0;
    Serial.println("Rocket not in flight")
  }
}

void launchStatusAccelerometer() {

}
