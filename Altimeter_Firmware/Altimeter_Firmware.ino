#include <Wire.h>
#include <MS5xxx.h>

uint1_t launchState = 0;  //0 for unlaunched, 1 for launched
double pnaught = 0;

MS5xxx barometer(&Wire);  //I do not understand what &Wire means but I know this has something to do with identifying the I2C pins

void calcAlt() {
  if (launchState == 1) {
    if (barometer.connect() == 0) {
      barometer.ReadProm();
      barometer.Readout();
      double temp = (barometer.GetTemp()) + 273;                           //Get temperature reading, convert from celcius to kelvin
      double pcurrent = (barometer.GetPres()) * 100;                       //Get pressure, convert to Pa from mbar
      return ((pow(pnaught / pcurrent, 0.19022256) - 1.) * temp) / 0.0065  //This is called the "Hypsometric Formula" and should give the delta altitude (in m) if pnaught is set properly
    }
  }
}

void zeroAlt() {
  if (launchState == 0) {
    if (barometer.connect() == 0) {
      barometer.ReadProm();
      barometer.Readout();
      pnaught = (barometer.GetPres()) * 100;  //Get pressure, convert to Pa from mbar
    }
  }
}

void launchStatus() {  //Uses barometer only, accelerometer definitely useful for this application
  uint32_t time1 = millis();
  float alt1 = calcAlt();
  delay(100);  //Placeholder, could be made smaller/larger
  float alt2 = calcAlt();
  uint16_t deltat = millis() - time1;
  avgV = (alt1 - alt2) / (deltat/1000); //Calculates average velocity in m/s
  if (avgV > 10) {  //Placeholder avgV threshold (m/s), could be made smaller/larger
    launchState = 1;
  } else {
    launchState = 0;
  }
}