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
unsigned long lastTime = micros();
unsigned long timeNow = micros();

float timeNowSec;

float A = 15;
//float w = 3;

float run1=0.4;
float run2=0.5720;
float run3=0.8179;
float run4=1.1696;
float run5=1.6725;
float run6=2.3916;
float run7=3.4200;
float run8=4.8904;
float run9=6.9932;
float run10=10;
float run11=15.8489;
float run12=25.1189;
float run13=39.8107;
float run14=63.0957;
float run15=100;

float T;
float kt = 107.34/335.0; //stall torque over stall current
float R = 22.0/335.0; //stall voltage (22V due to battery) over stall current
float Vin;
float omega;

void setup() {
  myservo.attach(Vyper_output);
  serialVal = 95;
  SerialUSB.begin(115200);
  lastTime = 0;
}

void loop() {
  timeNow = micros();
  timeNowSec = timeNow/1000000.0;
  currentVal = (((3.0/5.0)*analogRead(current_input))/((3.0/5.0)*1023.0))/0.02; //analogRead to amps

  encVal = enc1.getCounts();

  T = A*sin(run1*timeNowSec);

  float dt = float((timeNow-lastTime)/1000000.0);
  omega = float((((encVal*0.00628)-(encValPrev*0.00628))/(dt))/8.0);

  Vin = ((T*R)/kt)+kt*omega;
  int motorCommand;

  if(Vin < 0){
    motorCommand = map(Vin,-0.01,-22,98,149);
  }
  else if(Vin > 0){
    motorCommand = map(Vin,0.01,22,92,10);
  }
  else{
    motorCommand = 95;
  }
  
  myservo.write(motorCommand);

  currentVal = (((3.0/5.0)*analogRead(current_input))/((3.0/5.0)*1023.0))/0.02; //analogRead to amps
  
  if(timeNow>5000000 && timeNow<30000000){
    SerialUSB.print(timeNowSec,4);
    SerialUSB.print(", ");
    SerialUSB.print(T);
    SerialUSB.print(", ");
    SerialUSB.print(Vin);
    SerialUSB.print(", ");
    SerialUSB.println(omega,4);
  }
  encValPrev = encVal;
  lastTime = timeNow;
  delay(1);
}
