#include <AccelStepper.h>
#include <Arduino.h>

// Pin definitions
const int ROTARY_PINS[4] = {18, 5, 17, 16};
const int ULTRASONIC_TRIG_PIN = 14;
const int ULTRASONIC_ECHO_PIN = 12;
const int SOUND_SENSOR_PIN = 34;
const int STEPPER_STEP_PIN = 39;
const int STEPPER_DIR_PIN = 36;
const int DIGITAL_INPUT_PIN = 19;
const int DRIVER_ENABLE_PIN = 35;       // Low to enable

// Constants
const unsigned long DEBOUNCE_CNTR = 3;
unsigned long lastDebounceTime = 0;
const unsigned long ROTARY_CHECK_INTERVAL = 100;
const unsigned long DISTANCE_CHECK_INTERVAL = 20;
const unsigned long SOUND_CHECK_INTERVAL = 20;
const unsigned long PROFILE_DURATION = 600000;      // 10 minutes in milliseconds
const unsigned long SLOWDOWN_DURATION = 4096;       // 2^12 milliseconds (~4 seconds)
const unsigned long SLOWDOWN_SHIFT = 12;            // [bits]
const float SOUND_THRESHOLD = 500;
const unsigned long DIGITAL_INPUT_CHECK_INTERVAL = 50;
const float DISTANCE_TO_STEPS = 2000;               // 200 steps result in 1 mm
const unsigned long BOTTOM_POSITION = 20000;        // [Steps]
const unsigned long TOP_POSITION = 30000;           // [Steps]


// Variables
unsigned long lastRotaryCheck = 0;
unsigned long lastDistanceCheck = 0;
unsigned long lastSoundCheck = 0;
int readState = 0;
int currentRotaryState = 0;
float distance = 0.0;
long pos_in_steps = 0;
float soundLevel = 0.0;
volatile bool digitalInputState = false;
unsigned long lastDigitalInputCheck = 0;
volatile bool initializationFlag = true;
volatile bool readDistanceFlag = true;
volatile bool motionActive = false;
unsigned long profileStartTime = 0;

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

  digitalWrite(DRIVER_ENABLE_PIN, HIGH);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);

  // Create tasks for motor control and sensor reading
  xTaskCreatePinnedToCore(motorTask, "Motor Task", 10000, NULL, 2, NULL, 0);    // Higher priority
  xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 10000, NULL, 1, NULL, 1);  
}

void loop() {
    // The main loop can be left empty or used for other tasks if needed.
}

void motorTask(void *pvParameters) {
    while (true) {
      // Move only if commanded to and if distance finished reading
      if (motionActive && !readDistanceFlag) {
        updateStepperMotion();
        stepper.run();
      }
      vTaskDelay(1);
    }
}

void sensorTask(void *pvParameters) {
  while (true) {
    unsigned long currentMillis = millis();
    
    if (initializationFlag || readDistanceFlag) {
      readDistance();
    }    
    
    if (currentMillis - lastRotaryCheck >= ROTARY_CHECK_INTERVAL) {
      checkRotarySwitch();
      lastRotaryCheck = currentMillis;
    }  
    
    if (currentMillis - lastSoundCheck >= SOUND_CHECK_INTERVAL) {
      readSoundLevel();
      lastSoundCheck = currentMillis;
    }

    if (currentMillis - lastDigitalInputCheck >= DIGITAL_INPUT_CHECK_INTERVAL) {
        if (distance != 0.0) {  // Allow for motion only after reading the distance.
          checkDigitalInput();
          lastDigitalInputCheck = currentMillis;
        }
    }
    
    vTaskDelay(1); // Yield to other tasks
  }
}

void checkRotarySwitch() {
  int currentState = 0;
  int connectedInputs = 0;
  static int debounceCntr = 0;
  static bool prevState = 0;
  bool filtState = 0;
  
  // Read all inputs simultaneously
  currentState = (!digitalRead(ROTARY_PINS[0]))      |
                 (!digitalRead(ROTARY_PINS[1]) << 1) |
                 (!digitalRead(ROTARY_PINS[2]) << 2) |              
                 (!digitalRead(ROTARY_PINS[3]) << 3);

  // Count connected inputs using bit manipulation
  connectedInputs = __builtin_popcount(currentState);
  
  // Check if the read state is valid (only one input connected)
  if (connectedInputs > 1) {
    currentState = currentRotaryState; // Invalid state, use last stable state
  }
  else if (currentState != currentRotaryState) {
    if (prevState == currentState)  {
      debounceCntr++;
      if (debounceCntr > DEBOUNCE_CNTR)   {
        filtState = currentState;
        debounceCntr = 0;
      }
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

void readDistance() {  
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  
  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;
  pos_in_steps = (long)(distance * DISTANCE_TO_STEPS);
  
  readDistanceFlag = false;
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  Serial.print("Step position: ");
  Serial.print(pos_in_steps);
  Serial.println(" steps");  
}

void readSoundLevel() {
  int rawSound = analogRead(SOUND_SENSOR_PIN);
  soundLevel = map(rawSound, 0, 4095, 0, 1023);
  
  if (soundLevel > SOUND_THRESHOLD) {
    Serial.println("Loud sound detected!");
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
  }

  prevState = currentState;
  
  // Button was pressed post debouncing
  if (filt_state != digitalInputState) {
      digitalInputState = filt_state;
      // If not in motion and a state was selected
      if ((!motionActive) && (digitalInputState =! 0)) {
        digitalWrite(DRIVER_ENABLE_PIN, LOW);
        motionActive = true;
        profileStartTime = millis();                      
      }
      // In motion, or an OFF state wasn't selected (Not connected state)
      else {
        digitalWrite(DRIVER_ENABLE_PIN, HIGH);
        motionActive = false;
      }      
      Serial.print("Digital input state changed to: ");
      Serial.println(digitalInputState ? "Pressed" : "Released");
  }
}



void updateStepperMotion() {
    long currentPosition = stepper.currentPosition();
    unsigned long elapsedTime = millis() - profileStartTime;
    static unsigned long prevElapsedTime = 0;
    
    int speed = calculateSpeed(currentPosition, elapsedTime);    
    
    if (initializationFlag) {
      stepper.setCurrentPosition(pos_in_steps)
      stepper.moveTo(BOTTOM_POSITION)
    }

    if (stepper.distanceToGo() == 0) {
        stepper.moveTo(stepper.currentPosition() == TOP_POSITION ? BOTTOM_POSITION : TOP_POSITION);
        prevElapsedTime = elapsedTime;
        Serial.print("Elapsed time [ms]: ");
        Serial.println(elapsedTime - prevElapsedTime);
        readDistanceFlag = true;
    }

    stepper.setSpeed(speed);
    
    if (elapsedTime >= PROFILE_DURATION + SLOWDOWN_DURATION) {
        motionActive = false;
        prevElapsedTime = 0;
        digitalWrite(DRIVER_ENABLE_PIN, HIGH);
        Serial.println("Motion completed and stopped");
    }
}

int calculateSpeed(long currentPosition, unsigned long elapsedTime) {
    int baseSpeed;
    
    // Determine base speed based on current profile and direction
    switch (currentRotaryState) {
      case 0:
        baseSpeed = 500; // Constant speed
        break;
      case 1:
        // Different speeds for up and down
        baseSpeed = (stepper.targetPosition() == TOP_POSITION) ? 700 : 300;
        break;
      case 2:
        // Variable speed based on position
        baseSpeed = map(currentPosition, BOTTOM_POSITION, TOP_POSITION, 300, 700);
        break;
      default:
        baseSpeed = 500;
    }
    
    // Apply slowdown factor
    if (elapsedTime > PROFILE_DURATION) {
        unsigned long slowdownTime = elapsedTime - PROFILE_DURATION;
        if (slowdownTime >= SLOWDOWN_DURATION) {
            return 0;
        }

        if (slowdownTime > SLOWDOWN_DURATION) {
          baseSpeed = 0;
        }
        else {
          uint32_t slowdownFactor = 1024 - ((slowdownTime << 10) >> SLOWDOWN_SHIFT);
          baseSpeed = (baseSpeed * slowdownFactor) >> 10;          
        }
    }    
    return baseSpeed;
}


/* TODO:
Add QA to each function to test it's performance.
Add step by step guide to run the relevant QA procedures.
*/
