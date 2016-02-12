#include "I2Cdev.h"
#include <I2C.h>
#include <Servo.h> 
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


#define delaytime 3
Servo yawServo;
Servo pitchServo;
int yawServoPin = 7;
int pitchServoPin = 6;
int val_yaw = 5;
int val_pitch = 170;
boolean stabilization = false;
float dir = 2;
int pitch_increment = 2;
int line_count = 0;
int col_count = 0;
int max_line = 80;
int max_col = 40;
MPU6050 mpu;

// Global Variables
unsigned char LIDARLite_ADDRESSES[] = {0x62,0x42}; // Array of possible address for LIDAR-Lite Sensor
char LIDARLite_ADDRESS; // Variable to save the LIDAR-Lite Address when we find it in the array


#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_TEAPOT


#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

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
float y_old = 10.0;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    //lidar stuff:
    
    Serial.begin(115200); //Opens serial connection at 9600 baud.     
    I2c.begin(); // Opens & joins the irc bus as master
    delay(100); // Waits to make sure everything is powered up before sending or receiving data  
    I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails
    llFindSensor(); // Function that uses the LIDARLite-ADDRESSES[] array to find the sensor

    yawServo.write(val_yaw);
    pitchServo.write(val_pitch);
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

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

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        Serial.println(fifoCount);
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
        //
       
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        
        if(abs(y_old * 180/M_PI-ypr[0] * 180/M_PI)<0.01 & stabilization == false){
          Serial.print('\n');
           Serial.print("Stabilized");
           Serial.print('\n');
           yawServo.attach(yawServoPin);
           pitchServo.attach(pitchServoPin);
           stabilization = true;
           yawServo.write(val_yaw);
           pitchServo.write(val_pitch);
           
        }
        
        if (stabilization==true){ 
          int d = llGetDistance();
          Serial.print(d);
          Serial.print(" ");
          Serial.print(ypr[0] * 180/M_PI);
          Serial.print(" ");
          Serial.print(ypr[1] * 180/M_PI);
          Serial.print(" ");
          Serial.println(ypr[2] * 180/M_PI);
          Serial.print(" ");
          Serial.print(line_count);
          Serial.print(" ");
          Serial.print(col_count);
          Serial.print(" ");
          if(line_count<max_line){
              val_yaw += dir;
              line_count += 1;
              yawServo.write(val_yaw);
              //Serial.print(val_yaw);
              delay(5);
          }else{
            line_count = 0;
            val_yaw = val_yaw-dir;
            dir = -dir;
            //Serial.print(val_yaw);
            
            val_pitch -= pitch_increment;
            col_count += 1;
            pitchServo.write(val_pitch);
            delay(5);
              //dir = dir*1.05;
            
          }
          if(line_count == 1 & col_count == max_col+1){
             Serial.print("FINISHED");
             exit(1); 
          }
          
        }
        
        y_old = ypr[0];
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

/* ==========================================================================================================================================
Basic read and write functions for LIDAR-Lite, waits for success message (0 or ACK) before proceeding
=============================================================================================================================================*/

// Write a register and wait until it responds with success
void llWriteAndWait(char myAddress, char myValue){
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.write(LIDARLite_ADDRESS,myAddress, myValue); // Write to LIDAR-Lite Address with Value
    delay(2); // Wait 2 ms to prevent overpolling
  }
}

// Read 1-2 bytes from a register and wait until it responds with sucess
byte llReadAndWait(char myAddress, int numOfBytes, byte arrayToSave[2]){
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.read(LIDARLite_ADDRESS,myAddress, numOfBytes, arrayToSave); // Read 1-2 Bytes from LIDAR-Lite Address and store in array
    delay(2); // Wait 2 ms to prevent overpolling
  }
  return arrayToSave[2]; // Return array for use in other functions
}



/* ==========================================================================================================================================
Get 2-byte distance from sensor and combine into single 16-bit int
=============================================================================================================================================*/

int llGetDistance(){
  llWriteAndWait(0x00,0x04); // Write 0x04 to register 0x00 to start getting distance readings
  byte myArray[2]; // array to store bytes from read function
  llReadAndWait(0x8f,2,myArray); // Read 2 bytes from 0x8f
  int distance = (myArray[0] << 8) + myArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
  return(distance);
}



/* ==========================================================================================================================================
Average readings from velocity and distance
int numberOfReadings - the number of readings you want to average (0-9 are possible, 2-9 are reccomended)
=============================================================================================================================================*/


int llGetDistanceAverage(int numberOfReadings){ 
  if(numberOfReadings < 2){
    numberOfReadings = 2; // If the number of readings to be taken is less than 2, default to 2 readings
  }
  int sum = 0; // Variable to store sum
  for(int i = 0; i < numberOfReadings; i++){ 
      sum = sum + llGetDistance(); // Add up all of the readings
  }
  sum = sum/numberOfReadings; // Divide the total by the number of readings to get the average
  return(sum);
}



/* ==========================================================================================================================================
Find the LIDAR-Lite Sensor from array of possible I2C address
=============================================================================================================================================*/

void llFindSensor(){
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  String myString; // Setup string variable to print outcome
  bool sensorFound = false; // Setup flag to indicate whether a sensor has been found or not
  int i = 0; // Setup a counter
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    LIDARLite_ADDRESS = byte(LIDARLite_ADDRESSES[i]); // Set address from addresses array
    nackack = I2c.write(LIDARLite_ADDRESS ,0x41,0x00); // Write 0x00 to read only register 0x41 (will produce an ACK if there is a deice at the device address but won't write anything)
    if(nackack != 0){
      i++; // If no ACK from write, increment to next address
    }else{
      break; // If ACK recieved stop while loop
    }
    delay(2); // Wait 2 ms to prevent overpolling
  }
  byte testValue[1]; // Create array to store bytes from read
  llReadAndWait(0x02,1,testValue); // Read from register 0x02
  if(testValue[0] == 0x80){ // If device at address is LIDAR-Lite, register 0x02 should always equal 0x80
    myString += "Device @ 0x";
    myString += String(byte(LIDARLite_ADDRESS),HEX);  
    myString += " is LIDAR-Lite.";
  }else{ // If register 0x02 does not equal 0x80, increment to next address and try again
    i++;
  }  

}


