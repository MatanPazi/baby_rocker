#include <AccelStepper.h>
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>        // by Pololu

VL53L0X distance_sensor;

enum distanceStates {
  READ_DISTANCE_NOT_NEEDED,
  READ_DISTANCE_NEEDED,
  READ_DISTANCE_FINISHED,
};

// Pin definitions
const int ROTARY_PINS[4] = {26, 25, 33, 32};
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const int SOUND_SENSOR_PIN = 36;
const int STEPPER_STEP_PIN = 4;
const int STEPPER_DIR_PIN = 23;
const int DIGITAL_INPUT_PIN = 27;                       // ON/OFF Button
const int DRIVER_ENABLE_PIN = 19;                       // Low to enable
const int XSHUT_PIN = 15;                               // Reset pin for distance sensor


// Constants
const unsigned long DEBOUNCE_CNTR = 3;
const unsigned long ROTARY_CHECK_INTERVAL = 100;        // 100[ms] -> 10Hz
const unsigned long SOUND_CHECK_INTERVAL = 20;          // 20[ms] -> 50Hz
const unsigned long PROFILE_DURATION = 600000;          // 10 minutes in milliseconds
const unsigned long SLOWDOWN_DURATION = 4096;           // 2^12 milliseconds (~4 seconds)
const unsigned long SLOWDOWN_SHIFT = 12;                // 1 << SLOWDOWN_SHIFT = SLOWDOWN_DURATION
const unsigned long DIGITAL_INPUT_CHECK_INTERVAL = 10;  // 10[ms] -> 100Hz
const long DISTANCE_TO_STEPS = 20;                      // [steps/mm] (20 steps = 1 mm) ***Needs tuning***
const unsigned long MIDDLE_POSITION = 2900;             // [Steps]  ***Needs tuning***
const int SOUND_SAMPLES = 30;                           // # of samples in sound reading moving average
const int SOUND_THRESHOLD = 500;                        // 12 bits
const int DISTANCE_MARGIN = 100;                        // Max allowed distance from target position to be considered as "Reached target" [Steps]
const int STUCK_COUNTER_MAX = 2;                        // Max # of iterations in a row not reaching target, means motor is probably stuck.

// Structs
struct profileData {
    long topPos;
    long bottomPos;    
    int speed;
};

// Variables
bool soundLevel = 0.0;
volatile bool digitalInputState = false;
volatile bool motionActive = false;
int readState = 0;
int currentRotaryState = 0;
int sound_readings[SOUND_SAMPLES];
long distance = 0;
long posInSteps = 0;
unsigned long profileStartTime = 0;
unsigned long lastRotaryCheck = 0;
unsigned long lastDistanceCheck = 0;
unsigned long lastSoundCheck = 0;
unsigned long lastDigitalInputCheck = 0;
enum distanceStates readDistanceState = READ_DISTANCE_NEEDED;

int debug = 0;

// Stepper motor setup
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_PIN, STEPPER_DIR_PIN);

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  stepper.setPinsInverted(true, false, false);  /* Inverting direction pin logic */

  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, LOW);   // Pull XSHUT low to power down the sensor
  delay(10);                     // Wait for the sensor to shut down
  digitalWrite(XSHUT_PIN, HIGH); // Pull XSHUT high to power up the sensor
  delay(10);

  distance_sensor.setTimeout(500);
  if (!distance_sensor.init(true))
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }  
   
  pinMode(DRIVER_ENABLE_PIN, OUTPUT);

  for (int i = 0; i < 4; i++) {
    pinMode(ROTARY_PINS[i], INPUT_PULLUP);
  }

  // Initialize all sound readings to 0
  for (int i = 0; i < SOUND_SAMPLES; i++) {
    sound_readings[i] = 0;
  }    
    
  pinMode(DIGITAL_INPUT_PIN, INPUT_PULLUP);

  digitalWrite(DRIVER_ENABLE_PIN, HIGH);
  
  stepper.setMaxSpeed(500);                // Needs tuning
  stepper.setAcceleration(50);             // Needs tuning

  // Create tasks for motor control and sensor reading
  xTaskCreatePinnedToCore(motorTask, "Motor Task", 10000, NULL, 2, NULL, 0);    // Higher priority
  xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 10000, NULL, 1, NULL, 1);

  /* 
  debug = 0 - Not debugging, everything is executed
  debug = 1 - Checking readDistance
      Set debug to 1.
      Upload sketch to ESP32.
      Read SerialPrint logs.
      Point sensor at stuff and see results.
      Should give results in cm and steps assuming a 200 step per 1 cm ratio.

  debug = 2 - Checking checkRotarySwitch
      Set debug to 2.
      Upload sketch to ESP32.
      Nothing should be printed if switch state isn't changed.
      Change rotary switch between states.
      Flip between connected and also not connected states.
      Move slowly to try to potentially touch two pins simultaneously.
      Read SerialPrint logs.
      Something should be printed once only when a state has changed.
      Results should give values in bits.
      ROTARY_PINS[4] = {18, 5, 17, 16} -> 
      If pin 18 is connected, a value of 1 will be printed
      If pin 5 is connected, a value of 2 will be printed
      If pin 17 is connected, a value of 4 will be printed
      If pin 16 is connected, a value of 8 will be printed

  debug = 3 - Checking readSoundLevel
      Set debug to 3.
      Upload sketch to ESP32.      
      Read SerialPrint logs, the noise level should be printed.
      Once a noice above the threshold is detected a message will be printed.
      Once the noise disappears, another message will appear.
      Tune the threshold as desired using these messages.
      Then test.
      Make small noises, strong noises.
      Leave running for a few minutes to make sure no false positive are triggered.

  debug = 4 - Checking checkDigitalInput
      Set debug to 4.
      Upload sketch to ESP32.      
      Start reading serial log.
      Nothing should be printed if the ON/OFF button experienced no change in state.
      Press the button for a second and release.
      2 messages should be printed ('Pressed', then 'Released')
      Continue pressing and releasing the button.
      Shorten the pressing time little by little to see if there is a lower limit where a button press stops being detected.     

  debug = 5 - Checking motorTask
      * Must first move mechanism to middle position and measure distance and update MIDDLE_POSITION *

      Set debug to 5.
      Upload sketch to ESP32.
      Start reading serial log.
      Motor should start moving immediately from current position to bottom.
      Once reaching bottom it'll move to top and repeat.
      Each time it reaches the top or bottom it will print the elapsed time from previous target (Top or bottom)
  */

  if (debug == 5) {
    stepper.setCurrentPosition(MIDDLE_POSITION);
    digitalWrite(DRIVER_ENABLE_PIN, LOW);         // Enable motor in debug mode.
  }
  

}

void loop() {
    // The main loop can be left empty or used for other tasks if needed.
}

void motorTask(void *pvParameters) {
    while (true) {
      if (debug == 5 || debug == 0) {    
        // Move only if commanded
        if (motionActive || (debug == 5)) {
          updateStepperMotion();
          stepper.run();
        }
      }
      vTaskDelay(10);
    }
}

void sensorTask(void *pvParameters) {
  while (true) {
    unsigned long currentMillis = millis();
    if (debug == 1 || debug == 0 || debug == 5) {
      if ((readDistanceState == READ_DISTANCE_NEEDED) || debug == 1) {
        readDistance();
      }
    }
    if (debug == 2 || debug == 0) {    
      if (currentMillis - lastRotaryCheck >= ROTARY_CHECK_INTERVAL) {
        checkRotarySwitch();
        lastRotaryCheck = currentMillis;
      }  
    }
    
    if (debug == 3 || debug == 0) {    
      if (currentMillis - lastSoundCheck >= SOUND_CHECK_INTERVAL) {
        readSoundLevel();
        lastSoundCheck = currentMillis;
      }
    }

    if (debug == 4 || debug == 0) {    
      if (currentMillis - lastDigitalInputCheck >= DIGITAL_INPUT_CHECK_INTERVAL) {
          if (distance != 0 || debug == 4) {  // Allow for motion only after reading the distance.
            checkDigitalInput();
            lastDigitalInputCheck = currentMillis;
          }
      }
    }

    vTaskDelay(10); // Yield to other tasks
  }
}

/* Reads distance measurements and prints the distance in cm and steps */
void readDistance() {  
  distance = (long)(distance_sensor.readRangeSingleMillimeters());
  if (distance_sensor.timeoutOccurred()) {
    Serial.print(" TIMEOUT");
  }
  posInSteps = (long)(distance * DISTANCE_TO_STEPS);
  
  if (debug != 5)
  {
    stepper.setCurrentPosition(posInSteps);
  }

  readDistanceState = READ_DISTANCE_FINISHED;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" mm");
  Serial.print("Step position: ");
  Serial.print(posInSteps);
  Serial.println(" steps");  
}


/* Detects changed in the rotary switch.
Updates currentRotaryState and prints the change in state */
void checkRotarySwitch() {
  int currentState = 0;
  int connectedInputs = 0;
  static int debounceCntr = 0;
  static int prevState = 0;
  static int filtState = 0;
  
  // Read all inputs simultaneously (Assuming active low)
  currentState = (!digitalRead(ROTARY_PINS[0]))      |
                 (!digitalRead(ROTARY_PINS[1]) << 1) |
                 (!digitalRead(ROTARY_PINS[2]) << 2) |              
                 (!digitalRead(ROTARY_PINS[3]) << 3);

  // Count connected inputs using bit manipulation
  connectedInputs = __builtin_popcount(currentState);
  
  // Invalid case if connected inputs are greater than 1.
  if (connectedInputs > 1) {
    currentState = currentRotaryState; 
  }
  else if (currentState != currentRotaryState) {
    if (prevState == currentState)  {
      debounceCntr++;
      if (debounceCntr > DEBOUNCE_CNTR)   {
        filtState = currentState;
        debounceCntr = 0;
      }
    }
    else {
      debounceCntr = 0;
    }
  }
  
  prevState = currentState;
  
  // If the state is stable and different from the last stable state, update
  if (filtState != currentRotaryState) {
      currentRotaryState = filtState;      
      Serial.print("Rotary switch state: ");
      Serial.println(currentRotaryState);
  }
}

/* Detects noises above a certain threshold */
void readSoundLevel() {
  bool currentState = false;
  static int mov_avg_sum = 0;
  static int index = 0;
    
  int new_reading = analogRead(SOUND_SENSOR_PIN);    
  mov_avg_sum = mov_avg_sum - sound_readings[index] + new_reading;
  sound_readings[index] = new_reading;
  index = (index + 1) % SOUND_SAMPLES;
  if ((mov_avg_sum / SOUND_SAMPLES) > SOUND_THRESHOLD)
  {
    currentState = true;
  }  
    
  if (soundLevel != currentState) {
    soundLevel = currentState;
    Serial.print("Sound input changed to: ");
    Serial.println(soundLevel ? "High sound detected" : "No high sound detected");
  }
}

void checkDigitalInput() {
  static int debounceCntr = 0;
  static bool prevState = 0;
  static bool filt_state = 0;
  // Read the digital input (Assuming active low)
  bool currentState = !digitalRead(DIGITAL_INPUT_PIN);

  // Debouncing
  if (currentState != digitalInputState)   {
    if (prevState == currentState)  {
      debounceCntr++;
      if (debounceCntr > DEBOUNCE_CNTR)   {
        filt_state = currentState;
        debounceCntr = 0;
      }
    }
    else {
      debounceCntr = 0;
    }
  }

  prevState = currentState;
  
  // Button state has changed post debouncing
  if (filt_state != digitalInputState) {
      digitalInputState = filt_state;
      // Button was pressed
      if (digitalInputState == true) {
        // If not in motion and a state was selected
        if ((!motionActive) && (currentRotaryState != 0)) {
          digitalWrite(DRIVER_ENABLE_PIN, LOW);
          motionActive = true;
          profileStartTime = millis();                      
        }
        // In motion, or an OFF state wasn't selected (Not connected state)
        else {
          digitalWrite(DRIVER_ENABLE_PIN, HIGH);
          motionActive = false;
        }
      }      
      Serial.print("Digital input state changed to: ");
      Serial.println(digitalInputState ? "Pressed" : "Released");
  }
}


/* Setting speed and target position based on time passed since last rotary switch change and current position */
void updateStepperMotion() {
    long currentPosition = stepper.currentPosition();
    unsigned long elapsedTime = millis() - profileStartTime;
    static unsigned long prevElapsedTime = 0;
    static int stuckCounter = 0;
    
    profileData profile = calculateProfile(currentPosition, elapsedTime);

    if (stepper.distanceToGo() == 0) 
    {
        if (readDistanceState == READ_DISTANCE_NOT_NEEDED)
        {
          Serial.print("Elapsed time [ms]: ");
          Serial.println(elapsedTime - prevElapsedTime);
          if (debug == 5)
          {
            Serial.print("Estimated position: ");
            Serial.println(currentPosition);            
          }
          prevElapsedTime = elapsedTime;
          readDistanceState = READ_DISTANCE_NEEDED;          
        }
        else if (readDistanceState == READ_DISTANCE_FINISHED)
        {
          if (currentPosition > (profile.topPos - DISTANCE_MARGIN)) /* Close enough to top position */
          {
            stepper.moveTo(profile.bottomPos);
            stuckCounter = 0;
          }
          else if (currentPosition < (profile.bottomPos + DISTANCE_MARGIN)) /* Close enough to bottom position */
          {
            stepper.moveTo(profile.topPos);
            stuckCounter = 0;
          }     
          else
          {
            stepper.moveTo(profile.topPos); /* Didn't reach destination, try reaching the top position */
            stuckCounter++;
          }
          readDistanceState = READ_DISTANCE_NOT_NEEDED;
        }
        else
        {
          /* Do nothing.
             Waiting for readDistance() to run and update readDistanceState to READ_DISTANCE_FINISHED */
        }
    }    
    
    if (elapsedTime >= PROFILE_DURATION + SLOWDOWN_DURATION)
    {
        motionActive = false;
        prevElapsedTime = 0;
        digitalWrite(DRIVER_ENABLE_PIN, HIGH);
        Serial.println("Motion completed and stopped");
    }
    else if (stuckCounter >= STUCK_COUNTER_MAX)
    {
        motionActive = false;
        prevElapsedTime = 0;
        digitalWrite(DRIVER_ENABLE_PIN, HIGH);
        Serial.println("Motion stopped, system seems stuck");
        stuckCounter = 0;
    }
}

profileData calculateProfile(long currentPosition, unsigned long elapsedTime) {
    profileData profile;
    
    // Determine base speed based on current profile and direction
    switch (currentRotaryState) {
      case 1:
        // Slow speed
        profile.topPos = 3300;
        profile.bottomPos = 2600;
        profile.speed = 500;        
        break;
      case 2:
        // Different speeds for up and down        
        profile.topPos = 3300;                
        profile.bottomPos = 2600;
        profile.speed = (stepper.targetPosition() == profile.topPos) ? 3300 : 2600;        
        break;
      case 4:
        // Variable speed based on position
        profile.topPos = 3300;                
        profile.bottomPos = 2600;
        profile.speed = map(currentPosition, profile.bottomPos, profile.topPos, 2600, 3300);        
        break;
      case 8:
        // Variable speed based on position        
        profile.topPos = 3300;                
        profile.bottomPos = 2600;
        profile.speed = 3500;
        break;        
      default:        
        profile.topPos = 3300;                
        profile.bottomPos = 2600;
        profile.speed = 2500;
    }
    
    // Apply slowdown factor
    if (elapsedTime > PROFILE_DURATION) {
        unsigned long slowdownTime = elapsedTime - PROFILE_DURATION;

        if (slowdownTime >= SLOWDOWN_DURATION) {
          profile.speed = 0;
        }
        else {
          uint32_t slowdownFactor = 1024 - ((slowdownTime << 10) >> SLOWDOWN_SHIFT);
          profile.speed = (profile.speed * slowdownFactor) >> 10;          
        }
    }    
    return profile;
}


/* TODO:
Add harsher constraints to read ON/OFF button on initialization (Harsher then if (distance != 0.0))
Allow for readings only in certain range.
Filter distance readings (Average over 5 samples?)
*/
