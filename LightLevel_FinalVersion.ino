//LIGHT LEVEL PROJECT - REAL TIME SYSTEMS
//SCUOLA SUPERIORE SANT'ANNA - Feb 2019
//Author: Sergio Esteban Bautista Moreno - sebautistam@gmail.com

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
===============================================
*/

//SENSOR LIBRARIES
#include "I2Cdev.h"                                 //library for I2C communication
#include "MPU6050_6Axis_MotionApps20.h"             //RPY angles calculation
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE    
  #include "Wire.h"                                 //library for I2C communication
#endif

//LED STRIP LIBRARIES
#include <Adafruit_NeoPixel.h>                      //library for LED strip control
#ifdef __AVR__
#include <avr/power.h>                              //library for LED strip control (MU)
#endif

//Sensor structure
MPU6050 mpu;

//Interruption Pin for the sensor
#define   INTERRUPT_PIN       2  

//Strip control pin attached to the Arduino
#define   PIN                 6

// How many NeoPixels are in the strip?
#define   NUMPIXELS           144

//IMU Offsets previously calculated
#define    X_GYRO_OFFSET      149
#define    Y_GYRO_OFFSET      57
#define    Z_GYRO_OFFSET      -52
#define    Z_ACC_OFFSET       760

//FIFO maximum location
#define    FIFO_MAX_VALUE     1024

//DMP status
#define    DMP_OVERFLOW       0x10                    //Value if DMP FIFO is overflown
#define    DMP_CORRECT        0x02                    //Value if DMP is ready

//How long will the calibration be executed?
#define    INITIAL_WAIT       10000                   

//Transform values from the reading intervals from the sensor for the roll angle
#define    MAX_ROLL           0.894
#define    MIN_ROLL           0.932

//Lowest RGB, when decreasing the pitch
#define    LOW_RED            51
#define    LOW_GREEN          153
#define    LOW_BLUE           102

//Middle RGB, when pitch = 0
#define    MID_RED            155
#define    MID_GREEN          2
#define    MID_BLUE           155

//Highest RGB, when increasing the pitch
#define    HIGH_RED           54
#define    HIGH_GREEN         52
#define    HIGH_BLUE          0

//Transform values from the reading intervals from the sensor for the pitch angle
#define    MAX_PITCH_RED      1.263
#define    MAX_PITCH_GREEN    0.625
#define    MAX_PITCH_BLUE     1.938

#define    MIN_PITCH_RED      1.3
#define    MIN_PITCH_GREEN    1.888
#define    MIN_PITCH_BLUE     0.663    

//Transform value from the reading intervals from the sensor for the yaw angle
#define    MIN_YAW            0.172
#define    MAX_YAW            179

//Maximum value for the trail head
#define    MAX_HEAD           143

//Change intervals for the Red value in the calibration phase
#define    MIN_BRIGHT_CAL     30
#define    MAX_BRIGHT_CAL     255

//Initial trail to be shown during the calibration phase
#define    TAIL_CAL           63
#define    HEAD_CAL           79

//Maximum and mininum lenght of the light trail
#define    MAX_TRAIL_LENGTH   32
#define    MIN_TRAIL_LENGTH   1

//Strip middle pixel
#define    STRIP_MIDPOINT     71

//Creation of the object for strip control
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

bool      dmpReady = false;   // set true if DMP init was successful
uint8_t   mpuIntStatus;       // holds actual interrupt status byte from MPU
uint8_t   devStatus;          // return status after each device operation (0 = success, !0 = error)
uint16_t  packetSize;         // expected DMP packet size (default is 42 bytes)
uint16_t  fifoCount;          // count of all bytes currently in FIFO
uint8_t   fifoBuffer[64];     // FIFO storage buffer

Quaternion    q;              // [w, x, y, z]         quaternion container
VectorFloat   gravity;        // [x, y, z]            gravity vector
float   ypr[3];               // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float   ypr_cal[3];           // [yaw, pitch, roll]   yaw/pitch/roll for the calibration data result
float   ypr_ang[3];           // [yaw, pitch, roll]   yaw/pitch/roll container in degrees
  
volatile bool   mpuInterrupt = false;         // MPU interrupt flag

//CALIBRATION FLAGS
bool  calibrationDone, calibrationData = false;   

//CALIBRATION INITIAL TIMER
long    timeIni = 0;                          //Initial timer

//LIGHT TRAIL ROLL
int   curr_head, going_up_light = 0;           //Direction of the roll movement

int   new_head = STRIP_MIDPOINT;              //Center light of the strip

//COLOR CHANGE
int   new_red, curr_red = MID_RED;            //RGB initial color
int   new_green, curr_green = MID_GREEN;      //RGB initial color
int   new_blue, curr_blue = MID_BLUE;         //RGB initial color
int   going_up_color = 0;                     //Direction of the pitch movement

//LIGHT TRAIL LENGTH CHANGE
int   light_length = MIN_TRAIL_LENGTH;        //Initial light trail length

//INTERRUPTIONS
void dmpDataReady(void);

//CALIBRATION
void calibration(void);

//CHANGES COMPUTATION
void rollLight(void);
void pitchLight(void);
void yawLight(void);

//LIGHT DISPLAY
void updateLight(int);

void setup() {
  //Join I2C bus
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);        // 400kHz I2C clock
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  //Initialize serial communication
  Serial.begin(115200);

  //Initialize the sensor
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  //The interruption is an input
  pinMode(INTERRUPT_PIN, INPUT);

  //Verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  //Wait for data from user to continue
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  //Load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  //Gyroscope Offsets (previously calculated)
  mpu.setXGyroOffset(X_GYRO_OFFSET);
  mpu.setYGyroOffset(Y_GYRO_OFFSET);
  mpu.setZGyroOffset(Z_GYRO_OFFSET);
  mpu.setZAccelOffset(Z_ACC_OFFSET);

  //Check the sensor is ready
  if (devStatus == 0) 
  {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    //Attach interruption to the desire pin, and get the interruption status from sensor
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    //Get first data from the sensor
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    
    //Initializes the NeoPixel library.
    pixels.begin(); 

    //Start time count for initial sensor warming-up
    timeIni = millis();
  } 
  else 
  {
    //If DMP doesn't work, show the user the failed code
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

//No background tasks
void loop()
{

}

//----------------------------------------------------------------------------------
//READING DATA LOOP; executes each 5 ms.
//Data won't be available at all times, so other tasks can execute in the meantime.
//----------------------------------------------------------------------------------
void loop1(5) 
{
  //Run the calibration just one time
  if (calibrationDone == false) { 
    calibration();       
  }
  else
  {
    //If programming failed, don't proceed
  
    if (!dmpReady) 
    {
      Serial.println("DMP NOT READY");
      return;
    }
    else 
    {
      //Executes only if there are data ready in the FIFO
      if (mpuInterrupt == true) {
        //Do not allow the sensor to go to sleep mode
        mpu.setSleepEnabled(false);
  
        //Reset interrupt flag and get INT_STATUS byte
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();
  
        //Get current FIFO count
        fifoCount = mpu.getFIFOCount();
  
        //Check for overflow 
        if ((mpuIntStatus & DMP_OVERFLOW) || fifoCount == FIFO_MAX_VALUE) 
        {
          // reset so we can continue cleanly
          mpu.resetFIFO();
          Serial.println(F("FIFO overflow!"));
        }
        //executes if MPU is correct
        else if (mpuIntStatus & DMP_CORRECT)
        {
          //Wait for correct available data length
          while (fifoCount < packetSize)
            fifoCount = mpu.getFIFOCount();
  
          //Read a packet from FIFO
          mpu.getFIFOBytes(fifoBuffer, packetSize);
  
          //Reset FIFO to avoid overflow (no previous data required)
          mpu.resetFIFO();
  
          //If no calibration data has been acquired, save the first data to use as offset.
          if (calibrationData == false)
          {
            //Get the RPY angles
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr_cal, &q, &gravity);
            
            //Use the pure data for the yaw angle
            ypr_cal[0] = 0;   

            //Used for Serial printing of the RPY angles used in the calibration (optional)
            //Serial.print("ypr CAL\t");
            //Serial.print(ypr_cal[0] * 180 / M_PI);
            //Serial.print("\t");
            //Serial.print(ypr_cal[1] * 180 / M_PI);
            //Serial.print("\t");
            //Serial.println(ypr_cal[2] * 180 / M_PI);

            //set the flag to indicate that the data has been captured
            calibrationData = true;     
          }
          //Calibration process has been completed, proceed with data reading
          else
          {
            //Compute the RPY angles using the library
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  
            //Save the data in an array
            int   k;                  
            for (k = 0; k < 3; k++)
              ypr_ang[k] = (ypr[k] - ypr_cal[k]) * 180 / M_PI;

            //Used for Serial printing of the RPY angles (optional)
            //Serial.print("ypr\t");
            //Serial.print(ypr_ang[0]);
            //Serial.print("\t");
            //Serial.print(ypr_ang[1]);
            //Serial.print("\t");
            //Serial.println(ypr_ang[2]);
          }
        }
      }
    }
  }
}

//----------------------------------------------------------------------------------------------
//Computation of new light trail head, RGB colors and trail length.
//It executes each 14 ms, to have higher priority than the light update loop.
//It executes only if there is no data ready for reading and the calibration has been completed.
//----------------------------------------------------------------------------------------------
void loop2(14) 
{
  if (dmpReady)
  {
    if (calibrationDone == true && !mpuInterrupt)
    {
      rollLight();
      pitchLight();
      yawLight();
    }
  }
}

//---------------------------------------------------------
//Light update with the received values from other loops.
//It executes each 15 ms, so it has the lowest priority.
//---------------------------------------------------------
void loop3(15) 
{
  updateLight(light_length);
}

//Set the flag when a DMP data is ready
void dmpDataReady() 
{
  mpuInterrupt = true;
}

//Make a warm up of the sensor for about 10 ms and set the initial color of the light trail.
void calibration() 
{
  long  timeCurrent, timeDiff;
  timeCurrent, timeDiff = 0;              //time counters; current time and time difference
  
  int   bright, countDirection;           //light animation during calibration             
  int   j;                                //counter for pixels to be animated during calibration

  bright = MIN_BRIGHT_CAL;                
  countDirection = 1;
  
  //Return if the DMP is not ready; won't perform anymore actions.
  if (!dmpReady)
  {
    Serial.println("DMP NOT READY");
    return;
  }

  //Executes for ~10 seconds for an initial warming up. ~5 minutes to stable temperature
  while (timeDiff < INITIAL_WAIT)
  {
    //Calculate the time elapsed from initial setup until this instant
    timeCurrent = millis();
    timeDiff = timeCurrent - timeIni;
    
    //Show time enabled
    Serial.print("Calibrating\t");
    Serial.println(timeDiff);

    //Light increase or decrease
    if (bright == MIN_BRIGHT_CAL)
      countDirection = 1;
    else if (bright == MAX_BRIGHT_CAL)
      countDirection = -1;

    //actual change in the Red component
    bright = bright + countDirection;

    //write changes in the pixels array
    for (j = TAIL_CAL; j < HEAD_CAL; j++) {
      pixels.setPixelColor(j, pixels.Color(bright, 0, 0));
    }

    //updates the whole strip
    pixels.show();
  }
  
  Serial.println("Calibration Finished");

  //clear the whole strip, write the initial color in the central pixel
  for (j = 0; j < 144; j++) {
    if (j != STRIP_MIDPOINT )
      pixels.setPixelColor(j, pixels.Color(0, 0, 0));
    else
      pixels.setPixelColor(j, pixels.Color(new_red, new_green, new_blue));
  }
  pixels.show();

  //The initial delay was completed but no data has been stored for offset
  calibrationDone = true;
  calibrationData = false;
  
  return;
}

//Computes the new light trail head depending on the sensor reading.
void rollLight() 
{
  int   ypr_temp;         

  //Round to integer the roll angle in degrees
  ypr_temp = round(ypr_ang[2]);

  //Compute the new trail head pixel
  if (ypr_temp >= 0)
    new_head = round(STRIP_MIDPOINT  - (MAX_ROLL * ypr_temp));
  else if (ypr_temp < 0)
    new_head = round(STRIP_MIDPOINT  - (MIN_ROLL * ypr_temp));

  //-------------------------------------------------------------------------
  //Tail head cannot be over neither under the actual trail pixels.
  //These operations cannot be preempted to mantain consistency at all times.
  //-------------------------------------------------------------------------
  arteLock();                 
  if (new_head < 0)           
    new_head = 0;
  else if (new_head > MAX_HEAD)
    new_head = MAX_HEAD;
  arteUnlock();

  //Compute the direction of the movement required
  if (curr_head < new_head)
    going_up_light = 1;
  else if (curr_head > new_head)
    going_up_light = -1;
  else
    going_up_light = 0;
    
  return;
}

//Compute the destination color based on the sensor reading.
void pitchLight() 
{
  int   ypr_temp;

  //Round to integer the pitch angle in degrees
  ypr_temp = round(ypr_ang[1]);

  //Update the current RGB scheme
  curr_red = new_red;
  curr_green = new_green;
  curr_blue = new_blue;

  //The computation of the color change depends on the pitch direction
  if (ypr_temp >= 0)
  {
    new_red = round(MID_RED - (MAX_PITCH_RED * ypr_temp));
    new_green = round(MID_GREEN + (MAX_PITCH_GREEN * ypr_temp));
    new_blue = round(MID_BLUE - (MAX_PITCH_BLUE * ypr_temp));

    //-----------------------------------------------------------------------
    //Set limits to the color change to get to the desired colors.
    //These operations cannot be preempted to mantain the color in the range.
    //-----------------------------------------------------------------------
    arteLock();
    if (new_red < HIGH_RED)
      new_red = HIGH_RED;
    if (new_green > HIGH_GREEN)
      new_green = HIGH_GREEN;
    if (new_blue < HIGH_BLUE)
      new_blue = HIGH_BLUE; 
    arteUnlock();

    //Compute the direction of the color change required
    if (curr_blue < new_blue)
      going_up_color = 1;
    else if (curr_blue > new_blue)
      going_up_color = -1;
    else
      going_up_color = 0;
      
  }
  //repeat the operation but with different color range.
  else if (ypr_temp < 0)
  {
    new_red = round(MID_RED + (MIN_PITCH_RED * ypr_temp));
    new_green = round(MID_GREEN - (MIN_PITCH_GREEN  * ypr_temp));
    new_blue = round(MID_BLUE + (MIN_PITCH_BLUE * ypr_temp));

    //-----------------------------------------------------------------------
    //Set limits to the color change to get to the desired colors.
    //These operations cannot be preempted to mantain the color in the range.
    //-----------------------------------------------------------------------
    arteLock();
    if (new_red < LOW_RED)
      new_red = LOW_RED;
    if (new_green > LOW_GREEN)
      new_green = LOW_GREEN;
    if (new_blue < LOW_BLUE)
      new_blue = LOW_BLUE;
    arteUnlock();

    //Compute the direction of the color change required
    if (curr_green < new_green)
      going_up_color = -1;
    else if (curr_green > new_green)
      going_up_color = 1;
    else
      going_up_color = 0;
      
  }
  return;
}

//Compute the light trail length based on the sensor reading.
void yawLight()
{
  int   ypr_temp;
  
  //Round to integer the yaw angle in degrees
  ypr_temp = round(ypr_ang[0]);

  //The computation of the trial's length depending on the sign of the reading
  if (ypr_temp >= 0)
    light_length = 1 + round(MIN_YAW * (MAX_YAW - ypr_temp));
  else
    light_length = 1 + round(MIN_YAW * (MAX_YAW + ypr_temp));

  //-----------------------------------------------------------------------
  //Set limits to the length of the trail.
  //These operations cannot be preempted to mantain the color in the range.
  //-----------------------------------------------------------------------
  arteLock();
  if (light_length < MIN_TRAIL_LENGTH)
    light_length = MIN_TRAIL_LENGTH;
  else if (light_length > MAX_TRAIL_LENGTH)
    light_length = MAX_TRAIL_LENGTH;
  arteUnlock();

  return;
}

//Update the LED strip depending on previous calculations.
void updateLight(int light_length_local)
{
  int   tail;       //light tail
  int   i;          //counter for traverse the pixel array

  //move the head one pixel according to the 
  if (going_up_light == -1)
  {
    curr_head++;
    
    //do not exceed strip pixel length
    if (curr_head > MAX_HEAD)
      curr_head = MAX_HEAD;
  }
    
  else if (going_up_light == 1)
  {
    curr_head--;
    //do not exceed 0 pixel
    if (curr_head < 0)           
      curr_head = 0;
  }
    
  //Light tail depending on the head and the received length
  tail = curr_head - light_length_local;  

  if (tail < 0)           
    tail = 0;
  else if (curr_head > MAX_HEAD)
    curr_head = MAX_HEAD;

  //increment or the RGB scheme one value towards the desired
  if (going_up_color == 1)
  {
    curr_red++;
    curr_green--;
    curr_blue++;
    
    if (curr_red > HIGH_RED)
      curr_red = HIGH_RED;
    if (curr_green < HIGH_GREEN)
      curr_green = HIGH_GREEN;
    if (curr_blue > HIGH_BLUE)
      curr_blue = HIGH_BLUE; 
  }
  else if (going_up_color == -1)
  {
    curr_red--;
    curr_green++;
    curr_blue--;

    //do not exceed the minimum RGB color
    if (curr_red < LOW_RED)
      curr_red = LOW_RED;
    if (curr_green > LOW_GREEN)
      curr_green = LOW_GREEN;
    if (curr_blue < LOW_BLUE)
      curr_blue = LOW_BLUE;
  }

  arteLock();
  //traverse the pixel array, turn on with the desired color the trail and turn off the rest
  for (int i = 0; i < NUMPIXELS; i++)
  {
    if (i > tail && i <= curr_head)
      pixels.setPixelColor(i, pixels.Color(curr_red, curr_green, curr_blue));
    else
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));
  }
  pixels.show();
  arteUnlock();

  return;
}
