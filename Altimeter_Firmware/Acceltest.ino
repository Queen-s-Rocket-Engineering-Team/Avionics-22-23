#include <Wire.h>

#define H3LIS100DL_ADDRESS 0x19 // I2C address
#define WHO_AM_I 0x0F // "Who Am I" register address
#define H3LIS100DL_ID 0x32U // Expected "Who Am I" value
#define CTRL_REG1 0x20 // Control register 1
#define OUT_X 0x29 // X-axis acceleration data output register
#define OUT_Y 0x2B // Y-axis acceleration data output register
#define OUT_Z 0x2D // Z-axis acceleration data output register

#define SDA_PIN PB11
#define SCL_PIN PB10
#define STATUS_LED_PIN PA0

int8_t accelConst = 39;
//float XOut = 0;
//float YOut = 0;
//float ZOut = 0;

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  Wire.setSCL(SCL_PIN);
  Wire.setSDA(SDA_PIN);
  Wire.begin(); // join i2c bus
  Serial.begin(9600); // start serial for output
  selfAdjustDelay();
  
  // Check if H3LIS100DL is connected
  Wire.beginTransmission(H3LIS100DL_ADDRESS);
  Wire.write(WHO_AM_I);
  Wire.endTransmission();
  Wire.requestFrom(H3LIS100DL_ADDRESS, 1);
  if(Wire.available()) {
    int whoAmI = Wire.read();
    if(whoAmI == H3LIS100DL_ID) {
      Serial.println("H3LIS100DL detected");
    } else {
      Serial.println("H3LIS100DL not detected");
      while(1); // halt if H3LIS100DL not detected
    }
  }

  // Enable X, Y, Z axis and power on the module
  Wire.beginTransmission(H3LIS100DL_ADDRESS);
  Wire.write(CTRL_REG1);
  Wire.write(0x47);
  Wire.endTransmission();
}

void loop() {
  // Read X, Y, Z acceleration
  int16_t accelX = readAcceleration(OUT_X);
  int16_t accelY = readAcceleration(OUT_Y);
  int16_t accelZ = readAcceleration(OUT_Z);

  // Print acceleration values
  //XOut = accelX*accelConst;
  Serial.print("X: ");
  Serial.print(accelX/10000.0*accelConst, 2);
  Serial.print(" Y: ");
  Serial.print(accelY/10000.0*accelConst, 2);
  Serial.print(" Z: ");
  Serial.println(accelZ/10000.0*accelConst-2, 2);

  delay(200); // wait for a second
}

int16_t readAcceleration(byte reg) {
  Wire.beginTransmission(H3LIS100DL_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(H3LIS100DL_ADDRESS, 2);
  if(Wire.available() >= 2) {
    int16_t value = Wire.read() | (Wire.read() << 8);
    return value;
  } else {
    return 0;
  }
}

void selfAdjustDelay(){ //From Kennan Bays
  int countdown = 5000;
  while (countdown > 0 && !Serial.available()) {
    countdown -= 100;
    delay(100);
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
  }//while
  digitalWrite(STATUS_LED_PIN, LOW);
}
