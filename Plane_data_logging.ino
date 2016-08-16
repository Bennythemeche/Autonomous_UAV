/*CONNECTIONS
 * SD card:
 *  CS--->10
 *  DI--->51
 *  SCK-->52
 *  D0--->50
 */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "I2Cdev.h"
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "MPU6050_6Axis_MotionApps20.h"

const int throttle_pin=18;
const int roll_pin=19;
const int pitch_pin=20;

//const float Asf_nom=2*9.81/

File data_file;
//File data_file;

char c;
bool GPS_update_flag;


Adafruit_GPS GPS(&Serial1);
HardwareSerial mySerial=Serial1;
Adafruit_BMP280 bme;
MPU6050 mpu;

unsigned long GPS_millis;
float sl_press;

void SD_card_setup();
void GPS_setup();
void Barometer_setup();
void IMU_setup();
void read_IMU();

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

//stuff for ISR's

volatile unsigned long throttle_start;
volatile unsigned long throttle_end;
volatile unsigned long throttle_input;

volatile unsigned long pitch_start;
volatile unsigned long pitch_end;
volatile unsigned long pitch_input;

volatile unsigned long roll_start;
volatile unsigned long roll_end;
volatile unsigned long roll_input;






void setup() {

  Serial.begin(115200);
  while (!Serial){};

  SD_card_setup();

  

  IMU_setup();
  
  GPS_setup();
  

  Barometer_setup();

  //rc_input_setup();

  
}

  

  


void loop() {

  
  
  //If serial data is available, disable interrupts
  
  /*
  if (Serial1.available()){
    noInterrupts();
    
  }
  
  //If serial data is not available, allow interrupts
  else{
    interrupts();
  }
  */

  //We want to record GPS and barometer data at the same time.  We can get barometer pretty much any time, but only get GPS at 1Hz.
  //Whenever valid GPS data comes in, log that AND barometer data
  GPS.read();
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    GPS_millis=millis();
  
    if (!GPS.parse(GPS.lastNMEA())){   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    }
    else{
      //log data
      GPS_BMP_log();
    
    }
  }

  

  //Deal with inputs from radio
  

  
  
  
  read_IMU();
/*
  Serial.println("SD card satus");
  Serial.println(data_file);
  //data_file = SD.open("dataFile.txt", FILE_WRITE);

  Serial.println("SD card satus");
  Serial.println(data_file);
  data_file.println("in loop");
  Serial.println("writing to SD card");
  Serial.println(data_file.read());
  //data_file.close();
    
  

  delay(500);
  */
  
  
  /*
  
  //IMU stuff
  // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    if (!mpuInterrupt && fifoCount < packetSize) {
        //move on to other stuff
        return;
    }

    else{

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
    }

    mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);

    }

  

    
*/
    
    
    
    
    
  
  


  
  

  

  

  

}










/*========================================================================
 * HELPER FUNCTIONS AND ISR'S
 =========================================================================
 */

void GPS_setup(){
  Serial.println("Initializing GPS");
  GPS.begin(9600);
  
  //RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  //Use only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  
}

void GPS_BMP_log(){
  
  //Log GPS and barometer data at same time
  //data_file=SD.open("dataFile.txt",FILE_WRITE);
  data_file.println("GPS: Lat: " + String(GPS.lat)+" Lon: " + String(GPS.lon) + " Alt: " + String(GPS.altitude) + " Spd: " + String(GPS.speed) + " Ang: " + String(GPS.angle) + " Fix: " + String(GPS.fix) + " Sat: " + String(GPS.satellites) + " Fxq: " + String(GPS.fixquality) + " GPt: " + String(GPS.hour) + "_" + String(GPS.seconds) + "_" + String(GPS.milliseconds) + " Syt: " + String(GPS_millis));
  data_file.flush();

  Serial.println("GPS: Lat: " + String(GPS.lat)+" Lon: " + String(GPS.lon) + " Alt: " + String(GPS.altitude) + " Spd: " + String(GPS.speed) + " Ang: " + String(GPS.angle) + " Fix: " + String(GPS.fix) + " Sat: " + String(GPS.satellites) + " Fxq: " + String(GPS.fixquality) + " GPt: " + String(GPS.hour) + "_" + String(GPS.seconds) + "_" + String(GPS.milliseconds) + " Syt: " + String(GPS_millis));
  Serial.println("Here");
  //data_file.println("Bar: Alt: " + String(bme.readAltitude(sl_press)) + " Prs: " + String(bme.readPressure()) + " Syt: " + String(GPS_millis)); //GPS time and barometer time should be the same
  Serial.println("Bar: Alt: " + String(bme.readAltitude(sl_press)) + " Prs: " + String(bme.readPressure()) + " Syt: " + String(GPS_millis));  
}

void SD_card_setup(){
   Serial.print("Initializing SD card...");

  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  data_file = SD.open("dataFile.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (data_file) {
    Serial.print("Writing to dataFile.txt...");
    data_file.println("testing 1, 2, 3.");
    // close the file:
    data_file.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening dataFile.txt");
  }

  // re-open the file for reading:
  data_file = SD.open("dataFile.txt");
  if (data_file) {
    Serial.println("dataFile.txt:");

    // read from the file until there's nothing else in it:
    while (data_file.available()) {
      Serial.write(data_file.read());
    }
    
  }
  delay(1000); 
}

void Barometer_setup(){
  Serial.println(F("BMP280 test"));
  if (!bme.begin()) {  
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  float cumsum;
  for (int loop=0; loop<1000; loop++){
    cumsum+=bme.readPressure();
    
  }
  sl_press=cumsum/100000;
  Serial.print("Sea level pressure");
  Serial.println(sl_press);
  
}


void IMU_setup(){
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    
    bool devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }


  
}

void read_IMU(){
  if (!dmpReady) return;

  unsigned long Syt=millis();
  

  //if we aren't ready to read the IMU again, just get out of here
  if (!mpuInterrupt && fifoCount < packetSize) return;

  
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

        //Log data
        
        //data_file=SD.open("dataFile.txt", FILE_WRITE);

        data_file.print("IMU: aax: " + String(aa.x) + " aay: " + String(aa.y) + " aaz: " +String(aa.z));
        data_file.print(" aWx: " + String(aaWorld.x) + " aWy: " + String(aaWorld.x) + " aWz: " +String(aaWorld.z));
        data_file.print(" quw: " + String(q.w) + " qux: " + String(q.x) + " quy: " + String(q.y) + " quz: " + String(q.z));
        data_file.println(" Syt: "+String(millis()));

        data_file.flush();

        
        Serial.print("IMU: aax: " + String(aa.x) + " aay: " + String(aa.y) + " aaz: " +String(aa.z));
        Serial.print(" aWx: " + String(aaWorld.x) + " aWy: " + String(aaWorld.x) + " aWz: " +String(aaWorld.z));
        Serial.print(" quw: " + String(q.w) + " qux: " + String(q.x) + "quy: " + String(q.y) + " quz: " + String(q.z));
        Serial.println(" Syt: "+String(millis()));
        
        //data_file.println("testing 1, 2, 3.");
        
        
        
        
    }

        
}

void rc_input_setup(){
  attachInterrupt(digitalPinToInterrupt(throttle_pin),read_throttle,CHANGE);
  attachInterrupt(digitalPinToInterrupt(pitch_pin),read_pitch,CHANGE);
  attachInterrupt(digitalPinToInterrupt(roll_pin),read_roll,CHANGE);
  
}

void read_throttle(){
    if (digitalRead(throttle_pin)==HIGH){
        //rising edge
        throttle_start=micros();
    }

    else {
        //falling edge
        throttle_end=micros();
        throttle_input=throttle_end-throttle_start;
        //Serial.println(throttle_input_ISR);
        data_file.println("TIn: "+String(throttle_input));
    }
}

void read_pitch(){
    if (digitalRead(pitch_pin)==HIGH){
        //rising edge
        pitch_start=micros();
    }

    else {
        //falling edge
        pitch_end=micros();
        pitch_input=pitch_end-pitch_start;
        //Serial.println(yaw_input_ISR);
        data_file.println("PIn: "+String(pitch_input));
    }
}

void read_roll(){
    if (digitalRead(roll_pin)==HIGH){
        //rising edge
        roll_start=micros();
    }

    else {
        //falling edge
        roll_end=micros();
        roll_input=roll_end-roll_start;
        //Serial.println(yaw_input_ISR);
        data_file.println("RIn: "+String(roll_input));
        
    }
}




  


