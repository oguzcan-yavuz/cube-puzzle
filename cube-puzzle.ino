#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PololuLedStrip.h>

const int VIBRATION_MOTOR_PIN = 6;

// FACES OF THE CUBE TO => LED PINS
const int CUBE_FRONT = 7;
const int CUBE_RIGHT = 8;
const int CUBE_LEFT = 9;
const int CUBE_BACK = 10;
const int CUBE_TOP = 11;
const int CUBE_BOTTOM = 12;

#define LED_COUNT 1

MPU6050 mpu;
PololuLedStrip<CUBE_FRONT> frontLed;
PololuLedStrip<CUBE_RIGHT> rightLed;
PololuLedStrip<CUBE_LEFT> leftLed;
PololuLedStrip<CUBE_BACK> backLed;
PololuLedStrip<CUBE_TOP> topLed;
PololuLedStrip<CUBE_BOTTOM> bottomLed;

int const INTERRUPT_PIN = 2;  // Define the interruption #0 pin
bool blinkState;

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/ 
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gy;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector
float yaw = 0, pitch = 0, roll = 0;
float prevLoopYaw = 0, prevLoopPitch = 0, prevLoopRoll = 0; 
float prevStepYaw = 0, prevStepPitch = 0, prevStepRoll = 0;

bool timerEnabled = false;
unsigned long stepStartTime;
const unsigned long TIMEOUT_DURATION = 10000;  // Timeout in milliseconds (10 seconds for each step)
int resetCounter = 0;

enum Axis {
  YAW,
  PITCH,
  ROLL,
};

struct Rotation {
  Axis axis;
  float target;
};

Rotation LEFT = { YAW, -90 };
Rotation RIGHT = { YAW, 90 };

Rotation UP = { PITCH, 90 };
Rotation DOWN = { PITCH, -90 };

// WARNING: ROLL IS NOT WORKING WELL WITH THE PITCH EVEN WHEN WE CHANGE IT ONLY, SO USE IT WHEN THE PITCH IS NEUTRAL
Rotation ROLL_LEFT = { ROLL, 90 };
Rotation ROLL_RIGHT = { ROLL, -90 };

Rotation STEPS[] = { LEFT, RIGHT, DOWN, UP, RIGHT, RIGHT, UP, LEFT, LEFT, DOWN, DOWN, RIGHT, UP, LEFT };

int currentStepIndex = 0;
int totalSteps = sizeof(STEPS) / sizeof(STEPS[0]);
bool isPuzzleComplete = false;

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

// Converts a color from HSV to RGB.
// h is hue, as a number between 0 and 360.
// s is the saturation, as a number between 0 and 255.
// v is the value, as a number between 0 and 255.
rgb_color hsvToRgb(uint16_t h, uint8_t s, uint8_t v) {
    uint8_t f = (h % 60) * 255 / 60;
    uint8_t p = (255 - s) * (uint16_t)v / 255;
    uint8_t q = (255 - f * (uint16_t)s / 255) * (uint16_t)v / 255;
    uint8_t t = (255 - (255 - f) * (uint16_t)s / 255) * (uint16_t)v / 255;
    uint8_t r = 0, g = 0, b = 0;
    switch((h / 60) % 6) {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        case 5: r = v; g = p; b = q; break;
    }
    return rgb_color(r, g, b);
}

void turnOffLeds() {
  rgb_color colors[LED_COUNT];
  rgb_color off = rgb_color(0, 0, 0);
  colors[0] = off;

  frontLed.write(colors, LED_COUNT);
  rightLed.write(colors, LED_COUNT);
  leftLed.write(colors, LED_COUNT);
  backLed.write(colors, LED_COUNT);
  topLed.write(colors, LED_COUNT);
  bottomLed.write(colors, LED_COUNT);
}

void vibrate(int milliseconds) {
  digitalWrite(VIBRATION_MOTOR_PIN, true);
  delay(milliseconds);
  digitalWrite(VIBRATION_MOTOR_PIN, false);
}

void outputResetPuzzle() {
  // vibration motor is disabled
  // vibrate(2000);

  rgb_color colors[LED_COUNT];
  rgb_color red = rgb_color(255, 0, 0);
  rgb_color off = rgb_color(0, 0, 0);

  for(uint16_t i = 0; i < 5; i++) {
    colors[0] = red;

    frontLed.write(colors, LED_COUNT);
    rightLed.write(colors, LED_COUNT);
    leftLed.write(colors, LED_COUNT);
    backLed.write(colors, LED_COUNT);
    topLed.write(colors, LED_COUNT);
    bottomLed.write(colors, LED_COUNT);

    delay(200);

    turnOffLeds();

    delay(100);
  }
}

void outputCurrentStep(int currentStep) {
  const int step1Index = 3;
  const int step2Index = 6;
  const int step3Index = 8;
  const int step4Index = 10;
  const int step5Index = 12;
  const int step6Index = 14;
  rgb_color colors[LED_COUNT];
  rgb_color green = rgb_color(0, 255, 0);
  rgb_color blue = rgb_color(0, 0, 255);
  rgb_color off = rgb_color(0, 0, 0);

  if (currentStep >= step1Index) {
    colors[0] = blue;
    bottomLed.write(colors, LED_COUNT);
  }
 
  if (currentStep >= step2Index) {
    colors[0] = blue;
    backLed.write(colors, LED_COUNT);
  }

  if (currentStep >= step3Index) {
    colors[0] = blue;
    leftLed.write(colors, LED_COUNT);
  }

  if (currentStep >= step4Index) {
    colors[0] = blue;
    frontLed.write(colors, LED_COUNT);
  }

  if (currentStep >= step5Index) {
    colors[0] = blue;
    rightLed.write(colors, LED_COUNT);
  }

  if (currentStep >= step6Index) {
    colors[0] = blue;
    topLed.write(colors, LED_COUNT);
  }
}

bool isInErrorTolerance(float target, float actual) {
  float errorToleranceInDegrees = 10;
  float lowerBound = target - errorToleranceInDegrees;
  float upperBound = target + errorToleranceInDegrees;

  return lowerBound < actual && actual < upperBound;
}

void resetTimer() {
  stepStartTime = millis();
}

bool checkRotation(float targetAxisValue, float currentAxisValue, float prevStepAxisValue, float prevLoopAxisValue) {
  bool isInPosition = isInErrorTolerance(targetAxisValue, currentAxisValue);

  if (!isInPosition) {
    return false;
  }

  bool isDirectionCorrect = targetAxisValue > prevStepAxisValue ? currentAxisValue > prevLoopAxisValue : currentAxisValue < prevLoopAxisValue;

  if (!isDirectionCorrect) {
    resetPuzzle();
    return false;
  }

  return true;
}

void checkSteps() {
  if (isPuzzleComplete) {
    return;
  }

  if (timerEnabled && millis() - stepStartTime > TIMEOUT_DURATION) {
    Serial.println("--- TIMEOUT ---");
    resetPuzzle();  // Timeout - reset puzzle
    return;
  }

  Rotation currentStep = STEPS[currentStepIndex];
  float target, value, prevStepValue, prevLoopValue;

  if (currentStep.axis == YAW) {
    value = yaw;
    prevStepValue = prevStepYaw;
    prevLoopValue = prevLoopYaw;
  } else if (currentStep.axis == PITCH) {
    value = pitch;
    prevStepValue = prevStepPitch;
    prevLoopValue = prevLoopPitch;
  } else if (currentStep.axis == ROLL) {
    value = roll;
    prevStepValue = prevStepRoll;
    prevLoopValue = prevLoopRoll;
  }
  target = currentStep.target + prevStepValue;

  bool isStepComplete = checkRotation(target, value, prevStepValue, prevLoopValue);

  if (isStepComplete) {
    resetTimer();
    if (currentStep.axis == YAW) {
      prevStepYaw += currentStep.target;
    } else if (currentStep.axis == PITCH) {
      prevStepPitch += currentStep.target;
    } else if (currentStep.axis == ROLL) {
      prevStepRoll += currentStep.target;
    }
    currentStepIndex += 1;
  }
  outputCurrentStep(currentStepIndex);

  if (currentStepIndex == 1) {
    timerEnabled = true;
  }

  isPuzzleComplete = currentStepIndex == totalSteps;
}

// Reset puzzle if wrong movement is detected
void resetPuzzle() {
  prevStepYaw = 0;
  prevStepPitch = 0;
  prevStepRoll = 0;
  prevLoopYaw = 0;
  prevLoopPitch = 0;
  prevLoopRoll = 0;
  currentStepIndex = 0;
  resetCounter += 1;
  timerEnabled = false;
  Serial.println("❌ Incorrect movement or timeout! Puzzle reset.");
  outputResetPuzzle();
}

void logReadings() {
  Serial.print("yaw:\t");
  Serial.print(yaw);
  Serial.print("\tpitch:\t");
  Serial.print(pitch);
  Serial.print("\troll:\t");
  Serial.println(roll);
  
  Serial.print("prevLoopYaw:\t");
  Serial.print(prevLoopYaw);
  Serial.print("\tprevLoopPitch:\t");
  Serial.print(prevLoopPitch);
  Serial.print("\tprevLoopRoll:\t");
  Serial.println(prevLoopRoll);

  Serial.print("prevStepYaw:\t");
  Serial.print(prevStepYaw);
  Serial.print("\tprevStepPitch:\t");
  Serial.print(prevStepPitch);
  Serial.print("\tprevStepRoll:\t");
  Serial.println(prevStepRoll);
}

void logPuzzleState() {
  Serial.println("--- PUZZLE STATE ---");
  Serial.print("current step index:\t");
  Serial.println(currentStepIndex);
  Serial.print("resets:\t");
  Serial.println(resetCounter);
  Serial.print("SOLVED?: ");
  Serial.println(isPuzzleComplete ? "YES" : "NO");
  Serial.println("------");
}

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
  Serial.begin(115200); //115200 is required for Teapot Demo output
  while (!Serial);
  Serial.flush();

  /*Initialize device*/
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else {
    Serial.println("MPU6050 connection successful");
  }

  // Turn off LEDs
  turnOffLeds();

  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  } 
  else {
    Serial.print(F("DMP Initialization failed (code ")); //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }
  pinMode(LED_BUILTIN, OUTPUT);
}


void loop() {
  if (!DMPReady) {
    Serial.println("DMP failed, stopping the loop...");
    return; // Stop the program if DMP programming fails.
  }

  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet
    /* Display Euler angles in degrees */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    yaw = ypr[0] * 180/M_PI;
    pitch = ypr[2] * 180/M_PI;
    roll = ypr[1] * 180/M_PI;

    logReadings();
    logPuzzleState();

    checkSteps();

    // Store previous values for direction tracking
    prevLoopYaw = yaw;
    prevLoopPitch = pitch;
    prevLoopRoll = roll;

    /* Blink LED to indicate activity */
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);

    delay(100);
  }
}
