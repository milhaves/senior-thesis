#include <ME480FSM.h>
#include <Servo.h>

Servo myservo;
FSMEncoder1 enc1;

int Vyper_output = 10; //Vyper control signal
int current_input = A5; //current sensor input
int LSL_Input = 8; 
int LSR_Input = 9;

float goalT;

float currentVal;
int serialVal = 0;
long encVal;
long encValPrev = 0;
unsigned long lastTime = millis();
unsigned long timeNow = millis();

float dTime;
float balancerPosition;
float balancerPositionPrev = 0;
float Vin;
float kt = 107.34/372.0; //stall torque over stall current
float R = 22.0/372.0; //stall voltage (22V due to battery) over stall current

int state;
int motorCommand;
int stateL;
int stateLPrev;
int stateR;
int stateRPrev;
bool leftPressed;
bool rightPressed;
bool stickStuck;
bool fault;
bool limited;
float maxLeft = -60.0; //degrees
float maxRight = 60.0; //degrees

bool t1;
bool t2;
bool t3;
bool t4;
bool t5;
bool t6;
bool t7;
bool t8;
bool t9;
bool t10;
bool t11;
bool t12;
bool t13;
bool t14;
bool t15;
bool t16;
bool t17;

bool s1; //fault
bool s2; //idle
bool s3; //calibrating
bool s4; //ready
bool s5; //running
bool s6; //stepdown

bool brb;
bool ok;
bool go;
bool over;

FSMTimer linked(100000);
FSMTimer stuck(200000);

void setup() {
  myservo.attach(Vyper_output);
  serialVal = 95;
  pinMode(LSL_Input,INPUT_PULLUP);
  pinMode(LSR_Input,INPUT_PULLUP);
  SerialUSB.begin(115200);
}

void loop() {
  stateL=digitalRead(8);
  stateR=digitalRead(9);
  leftPressed = !stateL && stateLPrev;
  rightPressed = !stateR && stateRPrev;

  if(Serial.read()=='!'){ // Takes input from python script
//    brb=Serial.parseInt()==1;
//    ok=Serial.parseInt()==2;
//    //set=Serial.parseInt()==3;
//    go=Serial.parseInt()==4;
//    over=Serial.parseInt()==5;
//    goalT=Serial.parseFloat();
//    printData();
    
  }

//  noComm = linked.TMR;
  stickStuck = stuck.TMR;
  fault = brb or stickStuck;
  limited = leftPressed or rightPressed;

  delay(10);
  timeNow = micros();
  
}
