/*
    Library RF24, website : https://nRF24.github.io/RF24/ 
    License : GNU GPL v2
*/


#include <SPI.h>      //SPI library for communication with the nRF24L01+
#include "RF24.h"  //The main library of the nRF24L01+

RF24 radio (7, 8); // CE,CSN

//Create a pipe addresses for  communication
const uint64_t pipe = 0xE8E8F0F0E1LL;

int relayPin = 3; //Relay Pin
int led = 5;
int val = 0;

void setup() {
  Serial.begin(9600);
  pinMode(led, OUTPUT);
  radio.begin();                    //Start the nRF24 communicate
  radio.openReadingPipe(1, pipe);   //Sets the address of the transmitter to which the program will receive data.
  radio.startListening();
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH);
  delay(100);
}

void loop() {
  if (radio.available()) {
    radio.read(&val, sizeof(val));
    Serial.print("Val:  ");
    Serial.println(val);
  }


  switch (val) {
    case 1 :
      analogWrite(led, 20);
      break;
    case 2 :
      analogWrite(led, 50);
      break;
    case 3 :
      analogWrite(led, 100);
      break;
    case 4 :
      analogWrite(led, 150);
      break;
    case 5 :
      analogWrite(led, 250);
      digitalWrite(relayPin, LOW);
      Serial.println("Safe unlocked");
      delay(500);
      digitalWrite(relayPin, HIGH);
      break;
    default :
      analogWrite(led, 0);
      digitalWrite(relayPin, HIGH);
      break;
  }
}
