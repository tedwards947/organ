/*
  "WHOOSH"
    Wind Handling Organ Operation System Hub

*/

/*
  Phases
    Powerup
      Enable pin OFF for some time until 24v comes up for stepper
      Enable pin ON after


    Calibration
      Run in any direction until the first hall effect is reached
      Once reached, set direction toward the other hall effect sensor
      Which hall effect is reached will tell us which direction to go in

      *** NOTE ***
      it is possible to reach a hall effect sensor at t=0 !

    Normal Running
      Oscillate back and forth between hall effect stops
      Accelerating and decellerating evenly to avoid jerkiness

  
  Communication - I2C
    Input
      Start signal
      Stop signal
      Status inquiry? (Is this needed?)
    Output
      Status
      Pressure (cool to have but not needed)
      Errors (if any)


  Error checking / handling
    All errors are fatal. Report status to the Host.
    Error Types:
      -Check if MAX_STEPS is exceeded during Calibration or Normal Running phases
      -Can we use the presence of 2 hall effect sensors on separate belts to determine an imbalance?
      -Is it possible to detect overcurrent by the stepper/skipped steps?


  Other Considerations
    Listen for I2C at all times, I2C stop command overrides all
    Smooth acceleration / decceleration

  
  Global Constants
    MAX_STEPS //max number of steps (exceeding this is an error)
    MAX_PRESSURE //max pressure which if exceeded will cause 0 velocity
    MAX_VELOCITY //maximum velocity possible (to avoid failure)



*/

// Libraries
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <math.h>
#include <Wire.h>

// Stepper Pin / Other Definitions
#define EN_PIN 2
#define DIR_PIN 4 
#define STEP_PIN 3
#define CS_PIN 10
#define SW_MOSI 11
#define SW_MISO 12 
#define SW_SCK 13
#define R_SENSE 0.075f

// Other Pin Definitions
#define PRESSURE_PIN A0
#define HALL_0 A1
#define HALL_1 A2

#define NODE_ADDRESS 0x08 // for 
/*
  I2C default pins:
    SDA: A4
    SCL: A5
*/


// Stepper Driver setup
TMC5160Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

// Global Constants
const int MAX_DISTANCE = 600; //todo: correct to the actual measured value
const int MAX_PRESSURE = 1000;
const int DEFAULT_MAX_VELOCITY = 1500; //todo: correct to a reasonable value
const int DEFAULT_MAX_ACCELERATION = 2500;//2500;

//various velocities depending on pressure state
// const int VELOCITY_EXTREME = 0;
// const int VELOCITY_HIGH = 100;
// const int VELOCITY_NORMAL = 200;
// const int VELOCITY_LOW = 400;
const int VELOCITY_EXTREME = 0;
const int VELOCITY_HIGH = 500;
const int VELOCITY_NORMAL = 2000;
const int VELOCITY_LOW = 4000;

// Global Variables
bool allStop = false; //if this is set, stop the stepper!
bool hasError = false; //if this is true, stop the stepper! will report back via i2c too
bool direction = false; //the direction of the stepper

bool hall0Triggered = false;
bool hall1Triggered = false;

enum PressureState {
  PRESSURE_EXTREME,
  PRESSURE_HIGH,
  PRESSURE_LOW,
  PRESSURE_NORMAL
};
PressureState currentPressureState = PRESSURE_LOW;


void initI2C() {
  // Wire.begin(NODE_ADDRESS);
  // Wire.onReceive(receiveI2CEvent);
  // Wire.onRequest(requestI2CEvent);
  Serial.println("I2C Init Complete.");
}

void initHallEffectSensors() {
  pinMode(HALL_0, INPUT);
  Serial.println("Hall Effect Sensor Init Complete.");
}

void initStepperDriver() {
 
  //code goes here which sets up the driver:
  /*
    SPI setup
    serial setup

    driver start
    driver current limit
    stealthChop setup
    microstepping
    stepper enabled

    pause for 1sec (for 12-24v to come up)
    enable pin low
  */
  Serial.println("Setting up driver classes...");

  driver.begin();             
  driver.rms_current(2200);    

  driver.en_pwm_mode(1);     
  driver.pwm_autoscale(1);
  driver.pwm_freq(1);
  driver.microsteps(4);


  stepper.setMinPulseWidth(10);
  stepper.setMaxSpeed(DEFAULT_MAX_VELOCITY); 
  stepper.setAcceleration(DEFAULT_MAX_ACCELERATION); 
  stepper.setPinsInverted(false, false, true);
  stepper.enableOutputs();

  Serial.println("Waiting before setting enable pin...");
  delay(1000);
  stepper.setEnablePin(EN_PIN);

  Serial.println("Stepper Driver Init Complete.");
}

void initPressureSensor() {
   pinMode(PRESSURE_PIN, INPUT);

   //other stuff
  Serial.println("Pressure Sensor Init Complete.");
}

void receiveI2CEvent() {
  //check for stop command

}

void requestI2CEvent() {
 
}


void checkForHallTrigger() {
  int hall0 = analogRead(HALL_0);
  int hall1 = analogRead(HALL_1);

  hall0Triggered = (hall0 > 800 || hall0 < 400);
  hall1Triggered = (hall1 > 800 || hall1 < 400);
}

void checkPressureSensor() {
  const int EXTREME_THRESHOLD = 1000;
  const int HIGH_THRESHOLD = 600;
  const int LOW_THRESHOLD = 300;

  int pressure = analogRead(PRESSURE_PIN);

  if(pressure < LOW_THRESHOLD){
    currentPressureState = PRESSURE_LOW;
  }else if (pressure > EXTREME_THRESHOLD) {
    currentPressureState = PRESSURE_EXTREME;
  } else if(pressure > HIGH_THRESHOLD){
    currentPressureState = PRESSURE_HIGH;
  } else {
    currentPressureState = PRESSURE_NORMAL;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Serial Initialized.");
  Serial.println("Starting Initalization Tasks...");

  initI2C();
  initHallEffectSensors();
  initStepperDriver();
  initPressureSensor();

  // calibrate();
}

void setSpeedToMatchPressure() {
  switch (currentPressureState) {
    case PRESSURE_LOW:
      stepper.setMaxSpeed(VELOCITY_LOW);
    break;
    case PRESSURE_NORMAL:
      stepper.setMaxSpeed(VELOCITY_NORMAL);
    break;
    case PRESSURE_HIGH:
      stepper.setMaxSpeed(VELOCITY_HIGH);
    break;
    case PRESSURE_EXTREME:
      stepper.setMaxSpeed(VELOCITY_EXTREME);
    break;
  }
}

void loop() {
  driver.shaft(direction);

 
  checkForHallTrigger();
  checkPressureSensor();
  setSpeedToMatchPressure();

 

  if (hall0Triggered && direction == false) {
 
      stepper.stop();
      while (stepper.isRunning()) {
        stepper.run();
        // delayMicroseconds(1);
      }
      Serial.println("Hall effect 0 triggered. Stepper Stopped. Reversing!");
      direction = true;
      stepper.move(MAX_DISTANCE);
  } else if (hall1Triggered && direction == true) {
      stepper.stop();
      while (stepper.isRunning()) {
        stepper.run();
        // delayMicroseconds(1);
      }
      Serial.println("Hall effect 1 triggered. Stepper Stopped. Reversing!");
      direction = false;
      stepper.move(MAX_DISTANCE);
  } else if(stepper.distanceToGo() == 0) {
    direction = !direction;


    stepper.move(MAX_DISTANCE);
    // Serial.println("tryina!");
    
  }
  
  Serial.print("acceleration");
  Serial.println(stepper.acceleration());

  stepper.run();
}



// Stepper Movement Functions
void calibrate() {
  const int calibrationStepDelay = 10;
  // Run in any direction until the first hall effect is reached
  // Once reached, set direction toward the other hall effect sensor
  // Which hall effect is reached will tell us which direction to go in

  // *** NOTE ***
  // it is possible to reach a hall effect sensor at t=0 !

  const int MAX_CALIBRATION_DISTANCE = 200; //never exceed this, it's unreasonable. throw if it exceeds.


  Serial.println("Starting Stepper Calibration...");


  bool done = false;

  driver.shaft(direction);

  stepper.move(MAX_CALIBRATION_DISTANCE);


  while(!done){
    checkForHallTrigger();

      //TEMP
    stepper.setAcceleration(10000);



    if(!hall0Triggered && !hall1Triggered){
      stepper.run(); 
    } else {
      //set the direction var 
      //but don't set the direction in the driver!
      //we're doing this because what if it overruns the hall effect sensor

      if(hall0Triggered){
        direction = false;
      } else {
        direction = true;
      }

      stepper.setAcceleration(10000);
      stepper.stop();
      while (stepper.isRunning()) {
        stepper.run();
      }
      Serial.println("Hall effect triggered. Stepper Stopped.");
      done = true;
      stepper.setAcceleration(DEFAULT_MAX_ACCELERATION);
    }

  }

}



  // const int MAX_CALIB_STEPS = 200000;
  // for (uint16_t step = 0; step < MAX_CALIB_STEPS; step++) {
  //   int hallValue = checkForHallTrigger();

  //   if(hallValue == -1) {
  //     driver.shaft(direction);
  //     STEP_PORT |= (1 << STEP_PIN);
  //     // digitalWrite(STEP_PIN, HIGH);
  //     delayMicroseconds(calibrationStepDelay);
  //     STEP_PORT &= ~(1 << STEP_PIN);
  //     // digitalWrite(STEP_PIN, LOW);
  //   } else {
  //     Serial.println("Hall effect stop reached. Slowing and reversing direction bool.");
  //     //slow the stepper to a stop.
  //     //CODE

  //     //reverse the direction
  //     direction = !direction;

  //     step = MAX_CALIB_STEPS;
  //     break;

  //   }

  // }

  //don't forget to check for i2c stop event




