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

void setup() {
  myservo.attach(Vyper_output);
  serialVal = 95;
  SerialUSB.begin(115200);
}

void loop() {
  while(SerialUSB.available()>0){
    if(SerialUSB.read()=='!'){ // Takes input from serial
      serialVal=SerialUSB.parseInt();
    }
  }

  myservo.write(serialVal);

  currentVal = (((3.0/5.0)*analogRead(current_input))/((3.0/5.0)*1023.0))/0.02; //analogRead to amps
  
  encVal = enc1.getCounts();
  
  SerialUSB.print(millis());
  SerialUSB.print(", ");
  SerialUSB.print(encVal);
  SerialUSB.print(", ");
  SerialUSB.print(analogRead(current_input));
  SerialUSB.print(", ");
  SerialUSB.print(currentVal);
  SerialUSB.print(", ");
  SerialUSB.println(serialVal);
}
