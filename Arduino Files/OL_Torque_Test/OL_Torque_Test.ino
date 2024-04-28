#include <ME480FSM.h>
#include <Servo.h>

Servo myservo;
FSMEncoder1 enc1;

int Vyper_output = 10; //Vyper control signal
int current_input = A5; //current sensor input

float currentVal;
int serialVal = 0;
long encVal;
long encValPrev = 0;
unsigned long lastTime = millis();
unsigned long timeNow = millis();

float timeNowSec;

float A = 10;
float w = 3;
float T;
float kt = 107.34/335.0; //stall torque over stall current
float R = 22.0/335.0; //stall voltage (22V due to battery) over stall current
float Vin;

void setup() {
  myservo.attach(Vyper_output);
  serialVal = 95;
  SerialUSB.begin(115200);
  lastTime = 0;
}

void loop() {
  timeNow = millis();
  timeNowSec = timeNow/1000.0;
  currentVal = (((3.0/5.0)*analogRead(current_input))/((3.0/5.0)*1023.0))/0.02; //analogRead to amps

  encVal = enc1.getCounts();

  T = A*sin(w*timeNowSec);

  float encRadPrev = encValPrev*0.00628;
  float encRad = encVal*0.00628;
  encRadPrev = encRedPrev/8; //compensates for gearing
  encRad = encRad/8; //compensates for gearing
  float omega = (encRad-encRadPrev)/(timeNow-lastTime);

  Vin = ((T*R)/kt)+kt*omega;
  int motorCommand;

  if(Vin < 0){
    motorCommand = map(Vin,-22,-0.01,98,149);
  }
  else if(Vin > 0){
    motorCommand = map(Vin,0.01,22,92,10);
  }
  else{
    motorCommand = 95;
  }
  
  myservo.write(motorCommand);

  currentVal = (((3.0/5.0)*analogRead(current_input))/((3.0/5.0)*1023.0))/0.02; //analogRead to amps
  
  if((timeNow-lastTime)>100){
    SerialUSB.print(enc1Val);
    SerialUSB.print(", ");
    SerialUSB.print(encValPrev);
    SerialUSB.print(", ");
    SerialUSB.print(encRadPrev);
    SerialUSB.print(", ");
    SerialUSB.print(encRad);
    SerialUSB.print(", ");
    SerialUSB.print(Vin);
    SerialUSB.print(", ");
    SerialUSB.println(motorCounts);
  }
  enc1ValPrev = enc1Val;
  lastTime = timeNow;
  delay(100);
}
