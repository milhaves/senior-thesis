#include <ME480FSM.h>
#include <Servo.h>

Servo myservo;
FSMEncoder1 enc1;

int Vyper_output = 10; //Vyper control signal
int current_input = A5; //current sensor input

float currentVal;
long encVal;
long encValPrev = 0;
unsigned long lastTime = micros();
unsigned long timeNow = micros();
long startTime;
int motorCommand;
float Vin;
float omegaCounts;
float omega;
float omegaOld = 0;

void setup() {
  myservo.attach(Vyper_output);
  SerialUSB.begin(115200);
  startTime = micros();
  lastTime = 0;
}

void loop() {

  delay(1);

  timeNow = micros() - startTime;
  currentVal = (((3.0/5.0)*analogRead(current_input))/((3.0/5.0)*1023.0))/0.02;
  encVal = enc1.getCounts();

  float dt = float((timeNow-lastTime)/1000000.0);
  omega = float((((encVal*0.00628)-(encValPrev*0.00628))/(dt))/8.0);

  if(timeNow<2000000){
    Vin = 0;
  }
  else if(timeNow>2000000 && timeNow<7000000){
    Vin = 1;
  }
  else{
    Vin = 6;
  }

  if(Vin < 0){
    motorCommand = map(Vin,-22,-0.01,98,149); 
  }
  else if(Vin > 0){
    motorCommand = map(Vin,0.01,22,91,10);
  }
  else{
    motorCommand = 95;
  }

  myservo.write(motorCommand);

  if(timeNow>=2000000 && timeNow<=17000000){
//    Serial.print(fOmega);
    SerialUSB.print(timeNow);
    SerialUSB.print(", ");
    SerialUSB.print(Vin);
    SerialUSB.print(", ");
    SerialUSB.print(encVal);
    SerialUSB.print(", ");
    SerialUSB.print(encValPrev);
    SerialUSB.print(", ");
    SerialUSB.println(omega);
  }

  lastTime = timeNow;
  encValPrev = encVal;
  omegaOld = omega;
  
}
