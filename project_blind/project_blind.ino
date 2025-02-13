#include <NewPing.h>
#define TRIGGER_PIN 4
#define ECHO_PIN     3      
#define MAX_DISTANCE 800  
#define Buzzer 7             //BUZZER PIN
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 
 
void setup() {
 
  pinMode(Buzzer,OUTPUT);
  Serial.begin(9600); 
}
 
void loop() {
  //delay(50);                     
  Serial.print("Ping: ");
  Serial.print(sonar.ping_cm());
  Serial.println("cm");
  delay(500);
  if(sonar.ping_cm()<20){              //SET YOUR DISTANCE HERE
    digitalWrite(Buzzer,HIGH);
   
  }
  else{      
    digitalWrite(Buzzer,LOW);
 
  }
}
  