#include <ME480FSM.h>

int Vyper_output = 1; //Vyper control signal
int current_input = A5; //current sensor input

FSMEncoder1 enc1;

int PWMVal;
int currentVal;

void setup() {
  pinMode(Vyper_output, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  //calibrated Vyper pot range to -254 to 254
  PWMVal = -254;
  analogWrite(Vyper_output,PWMVal);
//  currentVal = analogRead(current_input);
  
//  Serial.println(currentVal);
}
