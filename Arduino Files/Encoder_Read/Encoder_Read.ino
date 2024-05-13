#include <ME480FSM.h>
#include <Servo.h>

Servo myservo;
FSMEncoder1 enc1;

int Vyper_output = 10; //Vyper control signal
int current_input = A5; //current sensor input

//int PWMVal;
float currentVal;
int serialVal;
long encVal;
long encValPrev = 0;
unsigned long lastTime = micros();
unsigned long timeNow = micros();

float omega;
float omegaOld = 0;

void setup() {
  myservo.attach(Vyper_output);
  serialVal = 95;
  SerialUSB.begin(115200);
//  startTime = micros();
  lastTime = 0;
}

void loop() {
  while(SerialUSB.available()>0){
    if(SerialUSB.read()=='!'){ // Takes input from serial
      serialVal=SerialUSB.parseInt();
    }
  }

  myservo.write(serialVal);

  currentVal = (((3.0/5.0)*analogRead(current_input))/((3.0/5.0)*1023.0))/0.02; //analogRead to amps
  
  timeNow = micros();
  encVal = enc1.getCounts();

  float dt = float((timeNow-lastTime)/1000000.0);
  omega = float(((encVal*0.00628)-(encValPrev*0.00628))/(dt));
  
  SerialUSB.print(millis());
  SerialUSB.print(", ");
  SerialUSB.print(encVal);
  SerialUSB.print(", ");
  SerialUSB.println(omega);
//  SerialUSB.print(", ");
//  SerialUSB.print(analogRead(current_input));
//  SerialUSB.print(", ");
//  SerialUSB.print(currentVal);
//  SerialUSB.print(", ");
//  SerialUSB.println(serialVal);

  lastTime = timeNow;
  encValPrev = encVal;
  omegaOld = omega;
}
