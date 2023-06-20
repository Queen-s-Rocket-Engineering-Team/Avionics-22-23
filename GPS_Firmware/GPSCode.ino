#include <Wire.h>
#include <TinyGPS++.h>  // Include TinyGPSPlus library

#define GPS_ADDRESS 0x42
#define STATUS_LED_PIN PB2
#define SCL_PIN PB6
#define SDA_PIN PB7

TinyGPSPlus gps;  // Create an instance of the TinyGPSPlus class

void selfAdjustDelay(){ //From Kennan Bays
  int countdown = 5000;
  while (countdown > 0 && !Serial.available()) {
    countdown -= 10;
    delay(10);
  }//while
  Serial.println("Serial Started!");
}

void checkAck() {
  uint8_t b5 = 0, b62 = 0;  // variables to store the first two bytes of the ACK message
  Wire.requestFrom(GPS_ADDRESS, 2);  // request the first two bytes

  if (Wire.available()) {
    b5 = Wire.read();
    b62 = Wire.read();

  }

  if (b5 == 0xB5 && b62 == 0x62) {
    Serial.println("ACK received!");
  } else {
    Serial.println("Failed to receive ACK!");
  }
}

void writeUbxCfgRate(uint16_t measRate) {
  uint8_t cfg_rate_cmd[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, measRate & 0xFF, measRate >> 8, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00};

  uint8_t ck_a = 0, ck_b = 0;
  for (int i = 2; i < sizeof(cfg_rate_cmd) - 2; i++) {
    ck_a = ck_a + cfg_rate_cmd[i];
    ck_b = ck_b + ck_a;
  }
  cfg_rate_cmd[12] = ck_a;
  cfg_rate_cmd[13] = ck_b;

  Wire.beginTransmission(GPS_ADDRESS);
  Wire.write(cfg_rate_cmd, sizeof(cfg_rate_cmd));
  Wire.endTransmission();
}

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  Wire.setSCL(SCL_PIN);
  Wire.setSDA(SDA_PIN);
  Wire.begin();
  
  Serial.begin(115200); // Initialize serial communication safe option(9600)
  selfAdjustDelay();

  // Send configuration command to set update rate to 20 Hz
  writeUbxCfgRate(50);

  delay(100);  // Wait a bit for the ACK message

  checkAck();

  Serial.println(F("GPS I2C COMMUNICATION"));  
}
 
void loop() { 
  uint32_t startTime = micros();
  byte error;
  Wire.beginTransmission(GPS_ADDRESS);
  Wire.write(0xFF);
  error = Wire.endTransmission();

  Wire.requestFrom(GPS_ADDRESS, 64);

  if (error == 0) {
    digitalWrite(STATUS_LED_PIN, HIGH);  // Blink the status LED if its working
    delay(60);
    digitalWrite(STATUS_LED_PIN, LOW);
  } else {
    digitalWrite(STATUS_LED_PIN, HIGH);// Turn ON the status LED if not working
    delay(60);
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


  Serial.print("[");
  uint32_t endTime = micros();
  Serial.print(1000000/(endTime - startTime));
  Serial.println("]");
  //delay(950);
  /**/
}
