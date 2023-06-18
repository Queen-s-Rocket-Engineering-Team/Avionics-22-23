#include <Wire.h>
#include <TinyGPS++.h>  // Include TinyGPSPlus library

#define GPS_ADDRESS 0x42
#define STATUS_LED_PIN PB2
#define SCL_PIN PB6
#define SDA_PIN PB7

TinyGPSPlus gps;  // Create an instance of the TinyGPSPlus class

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  Wire.setSCL(SCL_PIN);
  Wire.setSDA(SDA_PIN);
  Wire.begin();
  
  Serial.begin(115200); // Initialize serial communication safe option(9600)
  delay(500);
  Serial.println(F("GPS I2C COMMUNICATION"));  
}

void loop() {
    byte error;
  Wire.beginTransmission(GPS_ADDRESS);
  Wire.write(0xFF);
  error = Wire.endTransmission();

  Wire.requestFrom(GPS_ADDRESS, 64);

  if (error == 0) {
    digitalWrite(STATUS_LED_PIN, HIGH);  // Blink the status LED if its working
    delay(50);
    digitalWrite(STATUS_LED_PIN, LOW);
  } else {
    digitalWrite(STATUS_LED_PIN, HIGH);// Turn ON the status LED if not working
  }

  while (Wire.available()) {
    char c = Wire.read();
    gps.encode(c);  // Feed the NMEA sentence into the GPS parser
    Serial.write(c);
  }

  // Check if a new GGA sentence has been fully received
  if (gps.location.isUpdated()) {
    int satellites = gps.satellites.value();  // Get the number of satellites

    if (satellites >= 3) {
      Serial.println("Locked to 3 or more satellites!");
    } else {
      Serial.println("Locked to fewer than 3 satellites.");
    }
  }

  delay(950);
}

