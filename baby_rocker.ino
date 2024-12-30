#include <AccelStepper.h>
#include <Arduino.h>

// Pin definitions
const int ROTARY_PINS[4] = {26, 25, 33, 32};
const int ULTRASONIC_TRIG_PIN = 17;
const int ULTRASONIC_ECHO_PIN = 16;
const int SOUND_SENSOR_PIN = 21;
const int STEPPER_STEP_PIN = 22;
const int STEPPER_DIR_PIN = 23;
const int DIGITAL_INPUT_PIN = 27;                       // ON/OFF Button
const int DRIVER_ENABLE_PIN = 19;                       // Low to enable

// Constants
const unsigned long DEBOUNCE_CNTR = 3;
const unsigned long ROTARY_CHECK_INTERVAL = 100;        // 100[ms] -> 10Hz
const unsigned long SOUND_CHECK_INTERVAL = 20;          // 20[ms] -> 50Hz
const unsigned long PROFILE_DURATION = 600000;          // 10 minutes in milliseconds
const unsigned long SLOWDOWN_DURATION = 4096;           // 2^12 milliseconds (~4 seconds)
const unsigned long SLOWDOWN_SHIFT = 12;                // 1 << SLOWDOWN_SHIFT = SLOWDOWN_DURATION
const float SOUND_THRESHOLD = 500;                      // ***Needs tuning***
const unsigned long DIGITAL_INPUT_CHECK_INTERVAL = 50;  // 50[ms] -> 20Hz
const float DISTANCE_TO_STEPS = 2000;                   // [steps/cm] (200 steps = 1 mm) ***Needs tuning***
const unsigned long MIDDLE_POSITION = 20000;              // [Steps]  ***Needs tuning***
const unsigned long PULSE_IN_TIMEOUT = 2000;            // [uS]  ***Needs tuning***

// Structs
struct profileData {
    long topPos;
    long bottomPos;    
    int speed;
};

// Variables
unsigned long lastRotaryCheck = 0;
unsigned long lastDistanceCheck = 0;
unsigned long lastSoundCheck = 0;
int readState = 0;
int currentRotaryState = 0;
float distance = 0.0;
long posInSteps = 0;
bool soundLevel = 0.0;
volatile bool digitalInputState = false;
unsigned long lastDigitalInputCheck = 0;
volatile bool initializationFlag = true;
volatile bool readDistanceFlag = true;
volatile bool motionActive = false;
unsigned long profileStartTime = 0;
int debug = 0;

// Stepper motor setup
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_PIN, STEPPER_DIR_PIN);

void setup() {
  Serial.begin(115200);
  
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);  
  pinMode(DRIVER_ENABLE_PIN, OUTPUT);

  for (int i = 0; i < 4; i++) {
    pinMode(ROTARY_PINS[i], INPUT_PULLUP);
  }
    
  pinMode(DIGITAL_INPUT_PIN, INPUT_PULLUP);

  digitalWrite(DRIVER_ENABLE_PIN, HIGH);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  
  stepper.setMaxSpeed(5000);               // Needs tuning
  stepper.setAcceleration(500);             // Needs tuning

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
      Should give results in cm and steps assuming a 2000 step per 1 cm ratio.

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
      A value between 0 (Quiet) and 1023 (Loud)      
      Make noise.
      Verify that if the value exceeds SOUND_THRESHOLD (Default = 500), an additional message is printed:
      'Loud sound detected!'

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

  debug = 1;
  if (debug == 5) {
    stepper.setCurrentPosition(MIDDLE_POSITION);
  }
  

}

void loop() {
    // The main loop can be left empty or used for other tasks if needed.
}

void motorTask(void *pvParameters) {
    while (true) {
      if (debug == 5 || debug == 0) {    
        // Move only if commanded to and if distance finished reading
        if ((motionActive && !readDistanceFlag) || debug == 5) {
          updateStepperMotion();
          stepper.run();
        }
      }
      vTaskDelay(1);
    }
}

void sensorTask(void *pvParameters) {
  while (true) {
    unsigned long currentMillis = millis();
    if (debug == 1 || debug == 0) {
      if (initializationFlag || readDistanceFlag || debug == 1) {
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
          if (distance != 0.0 || debug == 4) {  // Allow for motion only after reading the distance.
            checkDigitalInput();
            lastDigitalInputCheck = currentMillis;
          }
      }
    }

    vTaskDelay(1); // Yield to other tasks
  }
}

/* Reads distance measurements and prints the distance in cm and steps */
void readDistance() {  
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  
  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, PULSE_IN_TIMEOUT);
  distance = duration * 0.034 / 2;
  posInSteps = (long)(distance * DISTANCE_TO_STEPS);
  
  stepper.setCurrentPosition(posInSteps);
  readDistanceFlag = false;
  if (initializationFlag) {
    initializationFlag = false;
  }

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
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
  static bool prevState = 0;
  bool filtState = 0;
  
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

// Need to filter sound?
void readSoundLevel() {
  static int debounceCntr = 0;
  static bool prevState = 0;
  bool filt_state = 0;
  bool currentState = digitalRead(SOUND_SENSOR_PIN);

  // Debouncing
  if (currentState != soundLevel)   {
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
  
  if (soundLevel != filt_state) {
    soundLevel = filt_state
    Serial.print("Sound input changed to: ");
    Serial.println(soundLevel ? "High sound detected" : "No high sound detected");
  }
}

void checkDigitalInput() {
  static int debounceCntr = 0;
  static bool prevState = 0;
  bool filt_state = 0;
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
    
    profileData profile = calculateProfile(currentPosition, elapsedTime);

    if (stepper.distanceToGo() == 0) {
        stepper.moveTo(stepper.currentPosition() == profile.topPos ? profile.bottomPos : profile.topPos);        
        Serial.print("Elapsed time [ms]: ");
        Serial.println(elapsedTime - prevElapsedTime);
        prevElapsedTime = elapsedTime;
        readDistanceFlag = true;
    }

    stepper.setSpeed(profile.speed);
    
    if (elapsedTime >= PROFILE_DURATION + SLOWDOWN_DURATION) {
        motionActive = false;
        prevElapsedTime = 0;
        digitalWrite(DRIVER_ENABLE_PIN, HIGH);
        Serial.println("Motion completed and stopped");
    }
}

profileData calculateProfile(long currentPosition, unsigned long elapsedTime) {
    profileData profile;
    
    // Determine base speed based on current profile and direction
    switch (currentRotaryState) {
      case 1:
        // Slow speed
        profile.topPos = 30000;
        profile.bottomPos = 20000;
        profile.speed = 100;        
        break;
      case 2:
        // Different speeds for up and down        
        profile.topPos = 30000;                
        profile.bottomPos = 20000;
        profile.speed = (stepper.targetPosition() == profile.topPos) ? 700 : 300;        
        break;
      case 4:
        // Variable speed based on position
        profile.topPos = 30000;                
        profile.bottomPos = 20000;
        profile.speed = map(currentPosition, profile.bottomPos, profile.topPos, 300, 700);        
        break;
      case 8:
        // Variable speed based on position        
        profile.topPos = 30000;                
        profile.bottomPos = 20000;
        profile.speed = 800;
        break;        
      default:        
        profile.topPos = 30000;                
        profile.bottomPos = 20000;
        profile.speed = 500;
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
*/
