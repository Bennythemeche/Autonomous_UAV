#include <SPI.h>
#include <SD.h>

File myFile;

String sTimeA;
String sTimeB;
unsigned long timeA;
unsigned long timeB;

void setup()
{
  Serial.begin(9600);
  Serial.print("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin 
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output 
  // or the SD library functions will not work. 
   pinMode(4, OUTPUT);
 
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  myFile = SD.open("test.txt", FILE_WRITE);

}

void loop() {
  for (int loop=0; loop<100; loop++) 
  {
    timeA=millis();
    sTimeA = String(timeA);
    myFile.println(sTimeA); // How long?
    timeB=millis();
    sTimeB = String(timeB);
    myFile.println(sTimeB);
  
    Serial.println(sTimeA);
    Serial.println(sTimeB);
  }

  myFile.close();
  Serial.println("Done");
  
}
