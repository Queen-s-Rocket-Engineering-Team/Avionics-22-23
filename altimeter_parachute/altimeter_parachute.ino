#define DROGUE_FIRE_PIN PB6
#define MAIN_FIRE_PIN PB7
#define DROGUE_CONTINUITY_PIN PB8
#define MAIN_CONTINUITY_PIN PB9

void setup() {
  // put your setup code here, to run once:
  pinMode(DROGUE_FIRE_PIN, OUTPUT);//drogue
  pinMode(MAIN_FIRE_PIN, OUTPUT);//main
  pinMode(DROGUE_CONTINUITY_PIN, INPUT);//drogue continuity 
  pinMode(MAIN_CONTINUITYE_PIN, INPUT);//main continunity 

}

void loop() {
  // put your main code here, to run repeatedly:


}

//fire main parachute
void fireMain(){
  digitalWrite(MAIN_FIRE_PIN,HIGH);
}

//fire drogue parachute
void fireDrogue(){
  digitalWrite(DROGUE_FIRE_PIN,HIGH);
}

//continuity check the main chute
bool continuityCheckMain() {
  return !digitalRead(MAIN_CONTINUITY_PIN);
}

//continuity check the drogue chute
bool continuityCheckDrogue() {
  return !digitalRead(DROGUE_CONTINUITY_PIN);
}
