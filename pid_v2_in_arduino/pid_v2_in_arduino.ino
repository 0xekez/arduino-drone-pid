/*
 * This program is in large part an alteration of drone-flight-controller
 * created by lobodol to work with an adafruit feather with bluetooth controlls.
 * you can find that code here:
 * https://github.com/lobodol/drone-flight-controller
 *
 * In order for this code to run you also need the BLuefruitConfig.h
 * and PacketParser.cpp files in the same directory, as well as the Adafruit_BLE
 * library. These, as well as a walkthrough an exaple use case can be found
 * here:
 * https://learn.adafruit.com/my-mini-race-car/code-for-your-robot
 */

// Allows for communication with MPU6050 sensor
# include <Wire.h>
// Allows for communication with Bluetooth controller
#include <Arduino.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include "BluefruitConfig.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// ---------------- Constants ---------------------------------------
#define INTERRUPT_PIN 2

// Index locations for data and instructions
# define YAW         0
# define PITCH       1
# define ROLL        2
# define THROTTLE    3

# define X           0     // X axis
# define Y           1     // Y axis
# define Z           2     // Z axis
#define MPU_ADDRESS 0x68  // I2C address of the MPU-6050

// Status of Drone
# define STOPPED     0
# define STARTING    1
# define STARTED     2

// ---------------- Controll variables ---------------------------------------
/*
 * Received flight instructions in order:
 * [Yaw (cardinal direction), Pitch (x), Roll (y), Throttle]
 * you might wonder.. why does the pitch default to 4?
 * the answer: not sure, no matter how I set up the gryo offsets flat is always 
 * apparently 4 degrees. I've tried subtracting four degrees from each measure,
 * calculating the offsets differently, everything except calculating acceletometer
 * offsets which I should do but don't want to.
 */
float instruction[4] = {0,4,0,0};

// ---------------- BLE variables ---------------------------------------
String BROADCAST_NAME = "Contains Nuts";
String BROADCAST_CMD = String("AT+GAPDEVNAME=" + BROADCAST_NAME);

// Serial peripheral interface for bluetooth
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];
char buf[60];
// store information about the time of the last press
unsigned long lastPress = 0;

// ---------------- MPU Variables ---------------------------------------
/*
 * Offset values are calculated with the IMU_zero exaple sketch
 */
// Average gyro offsets on each axis in order: [x, y, z]
long gyro_offsets[3] = {130,44,35};

//acc offsets in order: [x,y,z]
long acc_offsets[3] = {-3000,-581,649};

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
/*
 * Real YPR measurements calculated from combination of accelerometer and gyro
 * data in order: [yaw, pitch, roll]
 *  - Left side up implies positive roll
 *  - Nose up implies positive pitch
 *  - Nose right implies a positive yaw
 * NOTE: above stipulations are important in assembly of drone and position of
 *       MPU sensor.
 */
float measures[3] = {0,0,0};
// Flag to signify if the system is started
bool started = false;

// ---------------- Motor Variables ---------------------------------------
// Pins MOSFET sources are attached to on Arduino for each motor
int motor_lf {11},
    motor_lb {10},
    motor_rf {9},
    motor_rb {6};

// Throttle values for each motor
int motor_lf_throttle {0},
    motor_lb_throttle {0},
    motor_rf_throttle {0},
    motor_rb_throttle {0};

// ---------------- PID Variables ---------------------------------------
// Measured errors compared to target positions in order: [yaw, pitch, roll]
float errors[3];
// Error sums used for integral component of PID in order: [yaw, pitch, roll]
float error_sum[3] = {0,0,0};
// Previous errors used for derivative component of PID in order:
// [yaw, pirch, roll]
float previous_error[3] = {0,0,0};
//float Kp[3]        = {0, 6.7, 7.2};    // P coefficients in that order : Yaw, Pitch, Roll
//float Ki[3]        = {0.00, 0.00, 0.00}; // I coefficients in that order : Yaw, Pitch, Roll
//float Kd[3]        = {0,6.8,7.3};//{1.8, 1.5, 0};        // D coefficients in that order : Yaw, Pitch, Roll

//float Kp[3]        = {0, 0, 0};
//float Kp[3]        = {0, .21, .21};    // P coefficients in that order : Yaw, Pitch, Roll
//float Ki[3]        = {0.00, 0.01, 0.01}; // I coefficients in that order : Yaw, Pitch, Roll
//float Kd[3]        = {0, 10, 10};        // D coefficients in that order : Yaw, Pitch, Roll
float Kp[3]        = {0, 0.9, 0.9};    // P coefficients in that order : Yaw, Pitch, Roll
float Ki[3]        = {0.00, 0.00, 0.00}; // I coefficients in that order : Yaw, Pitch, Roll
float Kd[3]        = {0, 13, 13};        // D coefficients in that order : Yaw, Pitch, Roll

//float Kp[3]        = {8.5, 4, 4};    // P coefficients in that order : Yaw, Pitch, Roll
//float Ki[3]        = {0.00, 0.0, 0.00}; // I coefficients in that order : Yaw, Pitch, Roll
//float Kd[3]        = {0, 0, 0};        // D coefficients in that order : Yaw, Pitch, Roll

// ---------------------------------------------------------------------------
/**
 * Status of the quadcopter:
 *   - 0 : stopped
 *   - 1 : starting
 *   - 2 : started
 * @var int
 */
int status = STOPPED;
// ---------------------------------------------------------------------------

// for calculating running frequency of code
float i {0};
float start_seconds {0};

void setup() {
  stopMotors();
  Wire.begin();
  Serial.begin(115200);

  // Turn on LED for setup
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  setupMPUv2();

  BLEsetup();

  // Turn off MPU once setup is complete
  digitalWrite(13, LOW);
  start_seconds = millis()/1000;
}

void loop() {
  // mpu setup failed
  if (!dmpReady) {
    ble.print("MPU error");
    return;
  }
  // code for calculating the running frequency of the program
  i++;
  Serial.println(i/(millis()/1000-start_seconds));

  // 3. Read input from bluetooth controller
  readController();
  
  // 2. Calculate actual angles from raw data
  calculateAnglesv2();

  // 4. Calculate Errors based on new inputs
  calculateErrors();

  if (started) {
    // 5. Calculate new motor speeds
    pidController();
    applyMotorSpeeds();
  }
  else {
    stopMotors();
  }

}

void setupMPUv2(){
    // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  mpu.initialize();
  // load and configure dmp
  devStatus = mpu.dmpInitialize();

  //calibrate gyro offsets
  mpu.setXGyroOffset(gyro_offsets[0]);
  mpu.setYGyroOffset(gyro_offsets[1]);
  mpu.setZGyroOffset(gyro_offsets[2]);
  mpu.setZAccelOffset(gyro_offsets[2]);

  // make sure it worked
  if (devStatus == 0){
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
//    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
//    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
//    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

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

void calculateAnglesv2(){
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
      if (mpuInterrupt && fifoCount < packetSize) {
        // try to get out of the infinite loop 
        fifoCount = mpu.getFIFOCount();
      }  
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      fifoCount = mpu.getFIFOCount();
      Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(measures, &q, &gravity);
      measures[YAW] = measures[YAW]*180/M_PI;
      measures[PITCH] = measures[PITCH]*180/M_PI;
      measures[ROLL] = measures[ROLL]*180/M_PI;
//      Serial.print(measures[YAW]);
//      Serial.print("\t");
//      Serial.print(measures[PITCH]);
//      Serial.print("\t");
//      Serial.println(measures[ROLL]);
}
}


/**
 * Calculate errors of Yaw, Pitch & Roll: this is simply the difference between the measure and the command.
 *
 * @return void
 */
void calculateErrors() {
    // NOTE: currently, roll measurements are very noisy
    // consider removing from PID calculations ?
    errors[YAW]   = instruction[YAW]   - measures[YAW];
    errors[PITCH] = instruction[PITCH] - measures[PITCH];
    errors[ROLL]  = instruction[ROLL]  - measures[ROLL];
//    Serial.print(errors[YAW]);
//    Serial.print("\t");
//    Serial.print(errors[PITCH]);
//    Serial.print("\t");
//    Serial.println(errors[ROLL]);
}

// A small helper for bluetooth error catching
void error(const __FlashStringHelper*err) {
    Serial.println(err);
    while (1);
}

void BLEsetup(){
    Serial.print(F("Initialising the Bluefruit LE module: "));

    if ( !ble.begin(VERBOSE_MODE) )
    {
      error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
    }
    Serial.println( F("OK!") );

    if (! ble.factoryReset() ){
         error(F("Couldn't factory reset"));
    }

    //Convert the name change command to a char array
    BROADCAST_CMD.toCharArray(buf, 60);

    //Change the broadcast device name here!
    if(ble.sendCommandCheckOK(buf)){
      Serial.println("name changed");
    }
    delay(250);

    //reset to take effect
    if(ble.sendCommandCheckOK("ATZ")){
      Serial.println("resetting");
    }
    delay(250);

    //Confirm name change
    ble.sendCommandCheckOK("AT+GAPDEVNAME");

    /* Disable command echo from Bluefruit */
    ble.echo(false);
    
    /* Print Bluefruit information */
    ble.info();

    Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
    Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
    Serial.println();

    ble.verbose(false);  // debug info is a little annoying after this point!

    /* Wait for connection */
    while (! ble.isConnected()) {
      digitalWrite(13, LOW);
        delay(250);
      digitalWrite(13, HIGH);
      delay(250);
    }

    Serial.println(F("*****************"));

    // Set Bluefruit to DATA mode
    Serial.println( F("Switching to DATA mode!") );
    ble.setMode(BLUEFRUIT_MODE_DATA);

    Serial.println(F("*****************"));
}

void readController(){
    // read new packet data
    uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
   
    if (len == 0) return;
    // otherwise, we got a packet

    if (packetbuffer[1] == 'B') {
      // if its pressed, previous git versions will have a var assignment here
      // took it out in the hope that it will get a little quicker, but I'm dubious
      if (packetbuffer[3] - '0') {
        uint8_t buttnum = packetbuffer[2] - '0';
        if(buttnum == 1){
          // Increase throttle
          ble.println(instruction[THROTTLE]);
          instruction[THROTTLE] = minMax(instruction[THROTTLE]+5, 0, 250);
        }

        if(buttnum == 2){
          // Decrease throttle
          ble.println(instruction[THROTTLE]);
          instruction[THROTTLE] = minMax(instruction[THROTTLE]-5, 0, 250);
        }

        if(buttnum == 3){
          ble.println("Start");
          // possibly make these stop and start
          started = true;
        }

        if(buttnum == 4){
          ble.println("Stop");
          started = false;
        }

        if(buttnum == 5){
//          ble.println("Forward");
//          instruction[YAW] -= 1;
//          instruction[YAW] = minMax(instruction[YAW], -180, 180);
          Kp[1] += 0.5;
          Kp[2] += 0.5;
          ble.println("Kp: "+String(Kp[1]));
         
        }

        if(buttnum == 6){
//          ble.println("Backward");
//          instruction[YAW] += 1;
//          instruction[YAW] = minMax(instruction[YAW], -180, 180);
          Kp[1] -= 0.5;
          Kp[2] -= 0.5;
          ble.println("Kp: "+String(Kp[1]));
        }

        if(buttnum == 7){
          ble.println("right");
          instruction[ROLL] += 1;
          instruction[ROLL] = minMax(instruction[ROLL], -180, 180);
//          Kp[1] -= 0.02;
//          ble.println("Kp roll: "+String(Kp[1]));
        }

        if(buttnum == 8){
          ble.println("left");
          instruction[ROLL] -= 1;
          instruction[ROLL] = minMax(instruction[ROLL], -180, 180);
//          Kp[1] += 0.02;
//          ble.println("Kp roll: "+String(Kp[1]));
        }

        lastPress = millis();
      }
    }
}

/**
 * Calculate motor speed for each motor of an X quadcopter depending on received instructions and measures from sensor
 * by applying PID control.
 *
 * (A) (B)     x
 *   \ /     z ↑
 *    X       \|
 *   / \       +----→ y
 * (C) (D)
 *
 * Motors A & D run clockwise.
 * Motors B & C run counter-clockwise.
 *
 * @return void
 */
void pidController() {
    float delta_err[3] = {0, 0, 0};          // Error deltas in that order   : Yaw, Pitch, Roll
    float yaw_pid      = 0;
    float pitch_pid    = 0;
    float roll_pid     = 0;

    // Take no action if there is no throttle to the motors
    if (instruction[THROTTLE] > 0){
      // Calculate sum of errors : Integral coefficients
      error_sum[YAW]   += errors[YAW];
      error_sum[PITCH] += errors[PITCH];
      error_sum[ROLL]  += errors[ROLL];

      // Calculate error delta : Derivative coefficients
      delta_err[YAW]   = errors[YAW]   - previous_error[YAW];
      delta_err[PITCH] = errors[PITCH] - previous_error[PITCH];
      delta_err[ROLL]  = errors[ROLL]  - previous_error[ROLL];

      // Save current error as previous_error for next time
      previous_error[YAW]   = errors[YAW];
      previous_error[PITCH] = errors[PITCH];
      previous_error[ROLL]  = errors[ROLL];

      // PID = e.Kp + ∫e.Ki + Δe.Kd
      yaw_pid   = (errors[YAW]   * Kp[YAW])   + (error_sum[YAW]   * Ki[YAW])   + (delta_err[YAW]   * Kd[YAW]);
      pitch_pid = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (delta_err[PITCH] * Kd[PITCH]);
      roll_pid  = (errors[ROLL]  * Kp[ROLL])  + (error_sum[ROLL]  * Ki[ROLL])  + (delta_err[ROLL]  * Kd[ROLL]);

      // Cauculate new target throttle for each motor
      // NOTE: These depend on setup of drone. Verify setup is propper and
      //       consider changing the plus and minuses here if issues happen.
      //       If drone is in propper setup these make sense.
      motor_lf_throttle = instruction[THROTTLE] + roll_pid + pitch_pid - yaw_pid;
      motor_rf_throttle = instruction[THROTTLE] - roll_pid + pitch_pid + yaw_pid;
      // back motors are way more powerful
      motor_lb_throttle = instruction[THROTTLE] + roll_pid - pitch_pid + yaw_pid;
      motor_rb_throttle = instruction[THROTTLE] - roll_pid - pitch_pid - yaw_pid;
    }
    // Scale values to be within acceptable range for motors
    motor_lf_throttle = minMax(motor_lf_throttle, 0, 255);
    motor_rf_throttle = minMax(motor_rf_throttle, 0, 255);
    motor_lb_throttle = minMax(motor_lb_throttle, 0, 255);
    motor_rb_throttle = minMax(motor_rb_throttle, 0, 255);
}

/**
 * Make sure that given value is not over min_value/max_value range.
 *
 * @param float value     : The value to convert
 * @param float min_value : The min value
 * @param float max_value : The max value
 * @return float
 */
float minMax(float value, float min_value, float max_value) {
    if (value > max_value) {
        value = max_value;
    } else if (value < min_value) {
        value = min_value;
    }

    return value;
}

void applyMotorSpeeds(){
//  Serial.print(motor_lf_throttle);
//  Serial.print("\t");
//  Serial.print(motor_rf_throttle);
//  Serial.print("\t");
//  Serial.print(motor_lb_throttle);
//  Serial.print("\t");
//  Serial.println(motor_rb_throttle);
  analogWrite(motor_lf, motor_lf_throttle);
  analogWrite(motor_rf, motor_rf_throttle);
  analogWrite(motor_lb, motor_lb_throttle);
  analogWrite(motor_rb, motor_rb_throttle);
}

void stopMotors(){
  analogWrite(motor_lf, 0);
  analogWrite(motor_rf, 0);
  analogWrite(motor_lb, 0);
  analogWrite(motor_rb, 0);
}
