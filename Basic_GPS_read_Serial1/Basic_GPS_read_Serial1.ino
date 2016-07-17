
/*
Basic code to troubleshoot the Adafruit Ultimate GPS
Connections:
  - Vin - +5
  - GND - GND
  - Rx to RX (pin 0)
  - TX to TX (pin 1)
Not much else required.
 
 
*/
 
 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial.println("GPS Serial1 Test Code");
  delay(1000);
}
 
 
char c;
 
 
void loop() {
  // put your main code here, to run repeatedly:
  while(Serial1.available()) {
    c = Serial1.read();
    Serial.print(c);
  }
  delay(100);
}
