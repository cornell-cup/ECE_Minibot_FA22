/* The script for Minibot motor control with the RFID reader.
   DC motors and Encoders
        left: J6 port, (direction) digital pin 7, (pwm) digital pin 3, (encoder) analog pin A1
        right: J5 port, (direction) digital pin 4, (pwm) digital pin 6, (encoder) analog pin A0
   RFID 
        J10 port digital pin 9 (RESET),  analog pin A3(IRQ)

   Functions: move forward/back/left/right based on RFID Tags
*/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>
#include <elapsedMillis.h>
#define PN532_IRQ (A3)
#define PN532_RESET (9)
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

elapsedMillis timeElapsed;

// Define constants for locomotion
/** Right motor drivers */
int motor0_dirPin = 7;  // direction pin, LOW for forward, HIGH for backwards
int motor0_pwmPin = 3;  // pwm pin, controls voltage signal for speed
int pwm0 = 60;          // initial pwm value (doesn't really matter)

/** Left motor drivers */
int motor1_dirPin = 4;  // direction pin, HIGH for forward, LOW for backwards
int motor1_pwmPin = 6;  // pwm pin
int pwm1 = 60;          // initial pwm value

// Encoders regulate two motors to spin at same speed
// For more information, see PID algorithm on ECE documentation
int encoder0PinA = A1;  // J6 motor on board
int encoder0Pos = 0;    // Motor's angular position read by the encoder
int encoder0PinALast = LOW;

int encoder1PinA = A0;  // J5 motor on board
int encoder1Pos = 0;
int encoder1PinALast = LOW;


int setpoint = 600;    // turn rate for comparison (degrees/sec) Range: 0-800 (upper bound varies with timeSec value. Speed at pwm=255 is upper bound)
double Integral0 = 0;  // accumulated error with motors from desired number of turns
double Integral1 = 0;  // accumulated error with motors from desired number of turns
int n = LOW;
int m = LOW;

int encoder0PrevCount = 0;
int lastSpeed0 = 0;
int encoder1PrevCount = 0;
int lastSpeed1 = 0;

double timeSec = 0.2;  // update rate of the PID algorithm. Should match the timeElapsed < X in PID()

// PID constants
// P (proportional) is how much to adjust when turn rate is not equal to set rate. Matters most.
double kP = 0.3;
double kI = 0;
// D (derivative) how quickly it deviates from set rate. Adjusts quicker for greater rates
double kD = 0;

// initialize the buffer
int bufSize = 4;
char buf[4];
volatile byte pos = 0;

int test;
// char buff [50]; Use multiple parameters
char updated;
volatile byte indx;
volatile boolean process;
int interruptPin = 10;

int IRPin = 4;  // S4 on J5
int in;
int trigPin = 9;   // J10 on board
int echoPin = A3;  // this is the ADC pin
long duration, cm;

/*
  RFID
*/
uint8_t uid[] = { 0, 0, 0, 0 };              // Buffer to store the returned UID from tag
uint8_t obj1[] = { 0xF9, 0x3E, 0x4, 0xF4 };  // Yellow Tower's id
uint8_t obj2[] = { 0xC9, 0x12, 0xD, 0xF4 };  // Blue Tower's id
uint8_t obj3[] = { 0x59, 0xE3, 0xB, 0xF4 };  // Red Tower's id
uint8_t obj4[] = { 0x59, 0xC8, 0x6, 0xF4 };  // Orange Tower's id
uint8_t obj5[] = { 0x69, 0xDB, 0x6, 0xF4 };  // Purple Tower's id
uint8_t uidLength;

boolean detector;  // The value that is going to be used to detect whether a tag is found
void setup() {
  Serial.begin(115200);
  // RFID
  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1)
      ;  // halt
  }
  // Got data, print it out!
  Serial.print("Found chip PN5");
  Serial.println((versiondata >> 24) & 0xFF, HEX);
  Serial.print("Firmware ver. ");
  Serial.print((versiondata >> 16) & 0xFF, DEC);
  Serial.print('.');
  Serial.println((versiondata >> 8) & 0xFF, DEC);

  // Set the max number of retry attempts to read from a card
  // This prevents us from waiting forever for a card, which is
  // the default behaviour of the PN532.
  nfc.setPassiveActivationRetries(0xFF);

  // configure board to read RFID tags
  nfc.SAMConfig();

  // Serial.println("Show us your card!");

  // Locomotion
  pinMode(encoder0PinA, INPUT);
  pinMode(motor0_dirPin, OUTPUT);
  pinMode(motor0_pwmPin, OUTPUT);

  pinMode(encoder1PinA, INPUT);
  pinMode(motor1_dirPin, OUTPUT);
  pinMode(motor1_pwmPin, OUTPUT);

  pinMode(MISO, OUTPUT);  // init spi
  pinMode(MOSI, INPUT);

  pinMode(20, OUTPUT);
  pinMode(22, OUTPUT);

  pinMode(IRPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  test = 0;

  process = true;
}


/** Adjust PWM for PID algorithm */
// add specification for PWM and pins
void adjustPWM() {
  Serial.println("in adjust PWM");
  int speedNow0 = calculateSpeed0();  // calculate the current speed for the right motor
  int error0 = setpoint - speedNow0;  // calculate the error between the current speed and the set speed
  double dError0 = ((double)speedNow0 - (double)lastSpeed0) / timeSec;
  Integral0 += (double)error0;  // update integral of the error

  int speedNow1 = calculateSpeed1();  // calculate the current speed for the left motor
  int error1 = setpoint - speedNow1;
  double dError1 = ((double)speedNow1 - (double)lastSpeed1) / timeSec;
  Integral1 += (double)error1;

  // cap the integral value within 0..255
  if (Integral0 > 255) Integral0 = 255;
  else if (Integral0 < 0) Integral0 = 0;

  if (Integral1 > 255) Integral1 = 255;
  else if (Integral1 < 0) Integral1 = 0;

  // calculate the value for speed adjustments
  int adjust0 = (kP * (double)error0) + kI * Integral0 + kD * dError0;
  int adjust1 = (kP * (double)error1) + kI * Integral1 + kD * dError1;

  // update pwm values according to the moving direction
  pwm0 += adjust0;
  pwm1 += adjust1;

  // cap the pwm values within 0..255
  if (pwm0 > 255) pwm0 = 255;
  else if (pwm0 < 0) pwm0 = 0;

  if (pwm1 > 255) pwm1 = 255;
  else if (pwm1 < 0) pwm1 = 0;

  // store the current speeds
  lastSpeed0 = speedNow0;
  lastSpeed1 = speedNow1;
  return;
}

/** Return the current rotational speed of right motor with encoder data. */
int calculateSpeed0() {
  int speedDetect = (encoder0Pos - encoder0PrevCount) / timeSec;
  encoder0PrevCount = encoder0Pos;
  return speedDetect;
}


/** Return the current rotational speed of left motor with encoder data. */
int calculateSpeed1() {
  int speedDetect = (encoder1Pos - encoder1PrevCount) / timeSec;
  encoder1PrevCount = encoder1Pos;
  return speedDetect;
}

/** Adjust the speed of motors with the PID algorithm. */
void PID() {

  Serial.println("in PID");
  // Adjust the rotational speeds by the calculated pwm values.
  analogWrite(motor0_pwmPin, pwm0);
  analogWrite(motor1_pwmPin, pwm1);


  // Count the degrees of rotation in 0.2 seconds for each motor.
  timeElapsed = 0;
  while (timeElapsed < 200) {
    Serial.println("in while");
    n = digitalRead(encoder0PinA);  // store the current digital signal of the encoder
    if ((encoder0PinALast == LOW) && (n == HIGH)) {
      // a switch from HIGH to LOW of the encoder signal marks rotation in 1 degree.
      encoder0Pos++;
    }
    encoder0PinALast = n;  // update the last encoder signal for future comparison

    // same process for left encoder
    m = digitalRead(encoder1PinA);
    if ((encoder1PinALast == LOW) && (m == HIGH)) {
      encoder1Pos++;
    }
    encoder1PinALast = m;
  }
  adjustPWM();
  return;
}

/************************************
  RFID sensor code to run in loop
************************************/
void readRFID() {
  // RFID
  // Serial.println("Hello");

  detector = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength);

  Serial.println(detector);

  // If an RFID tag is detected, it will recognize the command on the block and update the stored value
  if (detector) {
    Serial.println("Found a tag!");
    Serial.println("This is .....");
    if (memcmp(obj1, uid, 4) == 0) {
      Serial.println("Yellow Block");
      updated = 'F';
      delay(10);
    }

    else if (memcmp(obj2, uid, 4) == 0) {
      Serial.println("Blue Block");
      updated = 'B';
      Serial.println(updated);
      delay(10);
    }

    else if (memcmp(obj3, uid, 4) == 0) {
      Serial.println("Red Block");
      updated = 'L';
      Serial.println(updated);
      delay(10);
    }

    else if (memcmp(obj4, uid, 4) == 0) {
      Serial.println("Orange Block");
      updated = 'R';
      Serial.println(updated);
      delay(10);
    }

    else if (memcmp(obj5, uid, 4) == 0) {
      Serial.println("Purple Block");
      updated = 'S';
      Serial.println(updated);
      delay(10);
    }

    else {
      Serial.println("Not in database");
      Serial.println(updated);
      delay(10);
    }

    delay(500);

  } else {
    Serial.println("No Objects in Range");
    delay(1000);
  }
}

void loop() {
  // RFID
  // Serial.println("Hello");

  detector = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength);

  Serial.println(detector);

  // If an RFID tag is detected, it will recognize the command on the block and update the stored value
  if (detector) {
    Serial.println("Found a tag!");
    Serial.println("This is .....");
    if (memcmp(obj1, uid, 4) == 0) {
      Serial.println("Yellow Block");
      updated = 'F';
      delay(10);
    }

    else if (memcmp(obj2, uid, 4) == 0) {
      Serial.println("Blue Block");
      updated = 'B';
      Serial.println(updated);
      delay(10);
    }

    else if (memcmp(obj3, uid, 4) == 0) {
      Serial.println("Red Block");
      updated = 'L';
      Serial.println(updated);
      delay(10);
    }

    else if (memcmp(obj4, uid, 4) == 0) {
      Serial.println("Orange Block");
      updated = 'R';
      Serial.println(updated);
      delay(10);
    }

    else if (memcmp(obj5, uid, 4) == 0) {
      Serial.println("Purple Block");
      updated = 'S';
      Serial.println(updated);
      delay(10);
    }

    else {
      Serial.println("Not in database");
      Serial.println(updated);
      delay(10);
    }

    delay(500);

  } else {
    Serial.println("No Objects in Range");
    delay(1000);
  }
  // clear the buffer when a command is executed
  Serial.println("process: " + String(process));
  if (process) {
    //process = false;
    switch (updated) {  // function changes the letter value of updated to a command
      case 'F':         // Moves Foward
        Serial.println("Forward");
        digitalWrite(motor0_dirPin, HIGH);
        digitalWrite(motor1_dirPin, HIGH);
        PID();
        break;  // breaks out of the switch loop and continues the original search

      case 'B':  // Moves Backwards (back())
        Serial.println("back");
        digitalWrite(motor0_dirPin, LOW);
        digitalWrite(motor1_dirPin, LOW);
        PID();
        break;  // breaks out of the switch loop and continues the original search

      case 'L':  // Moves Left
        Serial.println("Left");
        digitalWrite(motor0_dirPin, LOW);
        digitalWrite(motor1_dirPin, HIGH);
        analogWrite(motor0_pwmPin, 0);
        analogWrite(motor1_pwmPin, 255);
        break;  // breaks out of the switch loop and continues the original search

      case 'R':  // Moves Right
        Serial.println("RIGHT");
        digitalWrite(motor0_dirPin, HIGH);
        digitalWrite(motor1_dirPin, LOW);
        analogWrite(motor0_pwmPin, 255);
        analogWrite(motor1_pwmPin, 0);
        break;  // breaks out of the switch loop and continues the original search

      case 'S':  // Stops all the motors, makes all pins low
        Serial.println("Stop");
        analogWrite(motor0_pwmPin, 0);
        analogWrite(motor1_pwmPin, 0);
        break;  // breaks out of the switch loop and continues the original search


      default:  // code run when none of the cases are met
        analogWrite(motor0_pwmPin, 0);
        analogWrite(motor1_pwmPin, 0);
        break;  // breaks out of the switch loop and continues the original search
    }
  }
}