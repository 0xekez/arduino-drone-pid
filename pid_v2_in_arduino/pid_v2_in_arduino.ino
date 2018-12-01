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

// ---------------- Constants ---------------------------------------
# define MPU_ADDRESS 0x68  // I2C address of the MPU-6050
# define FREQ        130   // Sampling frequency
# define SSF_GYRO    65.5  // Sensitivity Scale Factor of the gyro from datasheet

// Index locations for data and instructions
# define YAW         0
# define PITCH       1
# define ROLL        2
# define THROTTLE    3

# define X           0     // X axis
# define Y           1     // Y axis
# define Z           2     // Z axis
#define MPU_ADDRESS 0x68  // I2C address of the MPU-6050
#define FREQ        250   // Sampling frequency
#define SSF_GYRO    65.5  // Sensitivity Scale Factor of the gyro from datasheet

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
/*
 * Time information, stored in 'volatile' to signal to compilier that this
 * changes unpredictably and shouldn't be optimized.
 */
volatile float elapsed_time, time, previous_time;

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
// Raw values from gyro (deg/sec) in order: [x, y, z]
int gyro_raw[3] = {0,0,0};
// Average gyro offsets on each axis in order: [x, y, z]
long gyro_offset[3] = {0,0,0};
// Calculated angles from gyro in order: [x, y, z]
float gyro_angle[3] = {0,0,0};

//Raw values from accelerometer (m/sec^2) in order: [x, y, z]
int acc_raw[3] = {0,0,0};
// Calculated angles from accelerometer in order: [x, y, z]
float acc_angle[3] = {0,0,0};

// total 3D acceleration vector (m/sec^2)
long acc_total_vector;

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
// The MPU temperature
int mpu_temp;
// Flag to signify if system in initialized
bool initialized = false;
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

// possible that each motor is of slightly different powers:
// these values are added from each motors throttles 
// at each loop through the PID
int motor_lf_mod {0},//{40},
    motor_rf_mod {0},
    motor_lb_mod {0},//{17},
    motor_rb_mod {0};

// ---------------- PID Variables ---------------------------------------
// Measured errors compared to target positions in order: [yaw, pitch, roll]
float errors[3];
// Error sums used for integral component of PID in order: [yaw, pitch, roll]
float error_sum[3] = {0,0,0};
// Previous errors used for derivative component of PID in order:
// [yaw, pirch, roll]
float previous_error[3] = {0,0,0};
float Kp[3]        = {0, 6.7, 7.2};    // P coefficients in that order : Yaw, Pitch, Roll
float Ki[3]        = {0.00, 0.00, 0.00}; // I coefficients in that order : Yaw, Pitch, Roll
float Kd[3]        = {0,5.8,6.3};//{1.8, 1.5, 0};        // D coefficients in that order : Yaw, Pitch, Roll

// Violent Oscilations
//float Kp[3]        = {4.0*(250/1000), 1.8*(250/1000), 1.3*(250/1000)};    // P coefficients in that order : Yaw, Pitch, Roll
//float Ki[3]        = {0.02*(250/1000), 0.04*(250/1000), 0.04*(250/1000)}; // I coefficients in that order : Yaw, Pitch, Roll
//float Kd[3]        = {0, 18*(250/1000), 18*(250/1000)};        // D coefficients in that order : Yaw, Pitch, Roll

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
//float i {0};
//float start_seconds {0};

// Function Prototypes
void pidController();
void applyMotorSpeeds();
void stopMotors();
float minMax(float value, float min_value, float max_value);
void error(const __FlashStringHelper*err);

void setup() {
  stopMotors();
  Wire.begin();
  Serial.begin(9600);
  TWBR = 12; // 24 for 400kHz clock, Feather is 200kHz

  // Turn on LED for setup
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  setupMpu6050Registers();

  calibrateMpu6050();

  BLEsetup();

  // Turn off MPU once setup is complete
  digitalWrite(13, LOW);
  //start_seconds = millis()/1000;
}

void loop() {
  // code for calculating the running frequency of the program = 150 hz
//  i++;
//  Serial.println(i/(millis()/1000-start_seconds));
  // 1. Read raw values from MPU6050
  readSensor();

  // 2. Calculate actual angles from raw data
  calculateAngles();

  // 3. Read input from bluetooth controller
  // NOTE: right now this does nothing. make sure to make button start drone
  //       this involves the started bool variable
  readController();

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

/**
 * Request raw values from MPU6050.
 *
 * @return void
 */
void readSensor() {
    Wire.beginTransmission(MPU_ADDRESS); // Start communicating with the MPU-6050
    Wire.write(0x3B);                    // Send the requested starting register
    Wire.endTransmission();              // End the transmission
    Wire.requestFrom(MPU_ADDRESS,14);    // Request 14 bytes from the MPU-6050

    // Wait until all the bytes are received
    while(Wire.available() < 14);

    acc_raw[X]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[X] variable
    acc_raw[Y]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[Y] variable
    acc_raw[Z]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[Z] variable
    mpu_temp = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the temperature variable
    gyro_raw[X] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyro_raw[X] variable
    gyro_raw[Y] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyro_raw[Y] variable
    gyro_raw[Z] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyro_raw[Z] variable
}

/**
 * Calculate real angles from gyro and accelerometer's values
 */
void calculateAngles()
{
    
    calculateGyroAngles();
    calculateAccelerometerAngles();

    if (initialized) {
        // Correct the drift of the gyro with the accelerometer
        gyro_angle[X] = gyro_angle[X] * 0.9996 + acc_angle[X] * 0.0004;
        gyro_angle[Y] = gyro_angle[Y] * 0.9996 + acc_angle[Y] * 0.0004;
    } else {
        // At very first start, init gyro angles with accelerometer angles
        resetGyroAngles();

        initialized = true;
    }

    // To dampen the pitch and roll angles a complementary filter is used
    measures[ROLL]  = measures[ROLL]  * 0.9 + gyro_angle[X] * 0.1;
    measures[PITCH] = measures[PITCH] * 0.9 + gyro_angle[Y] * 0.1;
    measures[YAW]   = -gyro_raw[Z] / SSF_GYRO; // Store the angular motion for this axis
    Serial.print(measures[YAW]);
    Serial.print("\t");
    Serial.print(measures[PITCH]);
    Serial.print("\t");
    Serial.println(measures[ROLL]);
}

/**
 * Calculate pitch & roll angles using only the gyro.
 */
void calculateGyroAngles()
{
    // Subtract offsets
    gyro_raw[X] -= gyro_offset[X];
    gyro_raw[Y] -= gyro_offset[Y];
    gyro_raw[Z] -= gyro_offset[Z];

    // Angle calculation using integration
    gyro_angle[X] += (gyro_raw[X] / (FREQ * SSF_GYRO));
    gyro_angle[Y] += (-gyro_raw[Y] / (FREQ * SSF_GYRO)); // Change sign to match the accelerometer's one

    // Transfer roll to pitch if IMU has yawed
    gyro_angle[Y] += gyro_angle[X] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
    gyro_angle[X] -= gyro_angle[Y] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
}

/**
 * Reset gyro's angles with accelerometer's angles.
 */
void resetGyroAngles()
{
    gyro_angle[X] = acc_angle[X];
    gyro_angle[Y] = acc_angle[Y];
}

/**
 * Calculate pitch & roll angles using only the accelerometer.
 */
void calculateAccelerometerAngles()
{
    // Calculate total 3D acceleration vector : √(X² + Y² + Z²)
    acc_total_vector = sqrt(pow(acc_raw[X], 2) + pow(acc_raw[Y], 2) + pow(acc_raw[Z], 2));

    // To prevent asin to produce a NaN, make sure the input value is within [-1;+1]
    if (abs(acc_raw[X]) < acc_total_vector) {
        acc_angle[X] = asin((float)acc_raw[Y] / acc_total_vector) * (180 / PI); // asin gives angle in radian. Convert to degree multiplying by 180/pi
    }

    if (abs(acc_raw[Y]) < acc_total_vector) {
        acc_angle[Y] = asin((float)acc_raw[X] / acc_total_vector) * (180 / PI);
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

/**
 * Configure gyro and accelerometer precision as following:
 *  - accelerometer: ±8g
 *  - gyro: ±500°/s
 *
 * @see https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 * @return void
 */
void setupMpu6050Registers() {
    // Configure power management
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(0x6B);                    // Request the PWR_MGMT_1 register
    Wire.write(0x00);                    // Apply the desired configuration to the register
    Wire.endTransmission();              // End the transmission

    // Configure the gyro's sensitivity
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(0x1B);                    // Request the GYRO_CONFIG register
    Wire.write(0x08);                    // Apply the desired configuration to the register : ±500°/s
    Wire.endTransmission();              // End the transmission

    // Configure the acceleromter's sensitivity
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(0x1C);                    // Request the ACCEL_CONFIG register
    Wire.write(0x10);                    // Apply the desired configuration to the register : ±8g
    Wire.endTransmission();              // End the transmission

    // Configure low pass filter
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(0x1A);                    // Request the CONFIG register
    Wire.write(0x03);                    // Set Digital Low Pass Filter about ~43Hz
    Wire.endTransmission();              // End the transmission
}

/**
 * Calibrate MPU6050: take 2000 samples to calculate average offsets.
 * During this step, the quadcopter needs to be static and on a horizontal surface.
 *
 * This function also sends low throttle signal to each ESC to init and prevent them beeping annoyingly.
 *
 * This function might take ~2sec for 2000 samples.
 *
 * @return void
 */
void calibrateMpu6050()
{
    int max_samples = 0;
    

    for (int i = 0; i < max_samples; i++) {
        readSensor();

        gyro_offset[X] += gyro_raw[X];
        gyro_offset[Y] += gyro_raw[Y];
        gyro_offset[Z] += gyro_raw[Z];

        // Just wait a bit before next loop
        delay(3);
    }

    // Calculate average offsets
    gyro_offset[X] /= max_samples;
    gyro_offset[Y] /= max_samples;
    gyro_offset[Z] /= max_samples;
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

    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
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

    Serial.println("Requesting Bluefruit info:");
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
    // Buttons
    if (packetbuffer[1] == 'B') {
      uint8_t buttnum = packetbuffer[2] - '0';
      boolean pressed = packetbuffer[3] - '0';

      if (pressed) {
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
          //ble.println("Forward");
//          instruction[YAW] -= 10;
//          instruction[YAW] = minMax(instruction[YAW], -180, 180);
          Kp[1] += 0.1;
          ble.println("Kp roll: "+String(Kp[1]));
         
        }

        if(buttnum == 6){
          //ble.println("Backward");
//          instruction[YAW] += 10;
//          instruction[YAW] = minMax(instruction[YAW], -180, 180);
          Kp[1] -= 0.1;
          ble.println("Kp roll: "+String(Kp[1]));
        }

        if(buttnum == 7){
          Kp[1] -= 0.02;
          ble.println("Kp roll: "+String(Kp[1]));
        }

        if(buttnum == 8){
          Kp[1] += 0.02;
          ble.println("Kp roll: "+String(Kp[1]));
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
 * Each motor output is considered as a servomotor. As a result, value range is about 1000µs to 2000µs
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
      Serial.println(yaw_pid);

      // Cauculate new target throttle for each motor
      // NOTE: These depend on setup of drone. Verify setup is propper and
      //       consider changing the plus and minuses here if issues happen.
      //       If drone is in propper setup these make sense.
      motor_lf_throttle = instruction[THROTTLE] + roll_pid + pitch_pid - yaw_pid + motor_lf_mod;
      motor_rf_throttle = instruction[THROTTLE] - roll_pid + pitch_pid + yaw_pid + motor_rf_mod;
      // back motors are way more powerful
      motor_lb_throttle = instruction[THROTTLE] + roll_pid - pitch_pid + yaw_pid + motor_lb_mod;
      motor_rb_throttle = instruction[THROTTLE] - roll_pid - pitch_pid - yaw_pid + motor_rb_mod;
    }
    // Scale values to be within acceptable range for motors
    motor_lf_throttle = minMax(motor_lf_throttle, 0, 250);
    motor_rf_throttle = minMax(motor_rf_throttle, 0, 250);
    motor_lb_throttle = minMax(motor_lb_throttle, 0, 250);
    motor_rb_throttle = minMax(motor_rb_throttle, 0, 250);
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