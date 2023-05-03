#define BATT_SENSE_PIN PA2

// Battery sense voltage divider ratio (default 3.00x)
const float BATT_SENSE_MULTIPLIER = 3.012; //199.4k & 99.1k
// Internal Vref voltage (default 1200mV)
const long INT_REF = 1196;

/*
 * NOTES:
 * You must increase ADC sampling time in built_opt.h to
 * give it time to charge when reading battery (100k+200k
 * resistor divider limits charge speed).
 * See https://github.com/stm32duino/Arduino_Core_STM32/issues/651
 * for options.
 */


void setup() {
  // put your setup code here, to run once:
  pinMode(BATT_SENSE_PIN, INPUT_ANALOG);
  analogReadResolution(12);

  Serial.begin(9600); //Baud doesn't matter for STM32 USB
}//setup()

void loop() {
  // put your main code here, to run repeatedly:

  Serial.print("VRef=");
  Serial.print(measureVref(500));
  Serial.print("mV  VBatt=");
  Serial.print(readBatteryVoltage());
  Serial.println("mV");
  delay(1000);

}//loop()


/*
 * Reads the battery voltage and
 * returns it in millivolts.
 * 
 * TODO: Add "num samples" argument
 */
int readBatteryVoltage() {
  // Get Vref
  int vRef = measureVref(200);
  // Read ADC value
  int aVal = analogRead(BATT_SENSE_PIN);
  // Calculate voltage
  return int( vRef*aVal*BATT_SENSE_MULTIPLIER/4096 );
}//readBatteryVoltage()

/*
 * Reads internal voltage reference and back-calculates
 * the system Vdd (VDDA) using a calibrated Vref consant.
 * Returns the average across several samples (MAX ~32k
 * samples supported). Probably works with many STM32F1xx
 * chips.
 * 
 * Inspired by the STM32duino Internal_channels example
 */
int measureVref(int samples) {
  // Take samples
  int result;
  long accum = 0;
  for (int i = 0; i < samples; i++) {
    result = analogRead(AVREF);
    accum += ((INT_REF * 4096L) / result); // Back-calculate VDDA in mV
  }//for
  return accum/samples; //Average
}//measureVref(int)
