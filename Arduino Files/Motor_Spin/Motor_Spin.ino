//#include <ME480FSM.h>
#include <Servo.h>

Servo myservo;

int Vyper_output = 10; //Vyper control signal
int current_input = A5; //current sensor input

//FSMEncoder1 enc1;

int PWMVal;
int currentVal;
int serialVal;

void setup() {
  myservo.attach(Vyper_output);
  SerialUSB.begin(115200);
}

void loop() {
  while(SerialUSB.available()>0){
    if(SerialUSB.read()=='!'){ // Takes input from serial
      serialVal=SerialUSB.parseInt();
    }
  }

  myservo.write(serialVal);

  SerialUSB.println(serialVal);
}
