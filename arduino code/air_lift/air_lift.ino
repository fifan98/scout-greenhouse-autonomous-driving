#include "Servo.h"

Servo servo;

int sensorPin = A0;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor
int procitana_vrijednost = 0;
int flag = 0;
int flag_gore = 0;
int mjeh = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  servo.attach(9);
  pinMode(3, OUTPUT); // izlaz 2 relej
  digitalWrite(3, HIGH);
  mjeh = 0;
  servo.write(15);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available() > 0) {
    // read the incoming byte:
      procitana_vrijednost = Serial.read();
      if (procitana_vrijednost == 48){      
      digitalWrite(3, HIGH);
      mjeh = 0;
      servo.write(15);
      flag = 0;
      }
    if (procitana_vrijednost == 49){
      servo.write(90);
      digitalWrite(3, LOW);
      mjeh = 1;
      flag = 1;
      }
  }
      // slanje vrijednosti senzora visine
      sensorValue = analogRead(sensorPin);
      Serial.println(sensorValue);

  if (flag == 1){
      if ( (sensorValue > 400) && (flag_gore == 0) ){
      servo.write(90);
      digitalWrite(3, HIGH);
      mjeh = 0;
      flag_gore = 1;
        } 
      if (flag_gore == 1){
         if ( sensorValue < 360){
          flag_gore = 0;
          digitalWrite(3, LOW);
          mjeh = 1;
          }
        }      
    }
    delay(100);
}
