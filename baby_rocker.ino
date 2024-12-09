#include <Arduino.h>
#include <AccelStepper.h>

// Pin definitions
const int ROTARY_PINS[4] = {18, 5, 17, 16};
const int ULTRASONIC_TRIG_PIN = 14;
const int ULTRASONIC_ECHO_PIN = 12;
const int SOUND_SENSOR_PIN = 34;
const int STEPPER_STEP_PIN = 39;
const int STEPPER_DIR_PIN = 36;
const int ON_OFF_PIN = 19;

// Constants
const unsigned long DEBOUNCE_DELAY = 200; // 200ms debounce time
unsigned long lastDebounceTime = 0;
const unsigned long ROTARY_CHECK_INTERVAL = 100;
const unsigned long DISTANCE_CHECK_INTERVAL = 10;
const unsigned long SOUND_CHECK_INTERVAL = 20;
const unsigned long PROFILE_DURATION = 600000; // 10 minutes in milliseconds
const unsigned long SLOWDOWN_DURATION = 32768; // 2^15 milliseconds (~32.8 seconds)
const unsigned long SLOWDOWN_SHIFT = 15; // log2(SLOWDOWN_DURATION)
const float SOUND_THRESHOLD = 500;


// Variables
unsigned long lastRotaryCheck = 0;
unsigned long lastDistanceCheck = 0;
unsigned long lastSoundCheck = 0;
int lastStableState = 0;
int readState = 0;
int currentRotaryState = 0;
float distance = 0;
float soundLevel = 0;

// Stepper motor setup
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_PIN, STEPPER_DIR_PIN);

// Motion profile function type
typedef void (*MotionProfile)(unsigned long, uint16_t);

// Helper function for gradual slowdown
uint16_t calculateSlowdownFactor(unsigned long elapsedTime) {
  if (elapsedTime <= PROFILE_DURATION) {
    return 1024; // 1.0 in fixed-point (10 bit precision)
  } else if (elapsedTime >= PROFILE_DURATION + SLOWDOWN_DURATION) {
    return 0;
  } else {
    return 1024 - (((elapsedTime - PROFILE_DURATION) << 10) >> SLOWDOWN_SHIFT);
  }
}

// Motion profile functions
void standstill(unsigned long elapsedTime, uint16_t slowdownFactor) {
  stepper.setSpeed(0);
  stepper.run(); // This will hold the motor's position
}

void constantSpeed(unsigned long elapsedTime, uint16_t slowdownFactor) {
  int32_t speed = (1000LL * slowdownFactor) >> 10;
  stepper.setSpeed(speed);
}

void oscillating(unsigned long elapsedTime, uint16_t slowdownFactor) {
  int32_t speed = static_cast<int32_t>((1000LL * sin(2 * PI * elapsedTime / 5000.0) * slowdownFactor)) >> 10;
  stepper.setSpeed(speed);
}

void fastSlowFast(unsigned long elapsedTime, uint16_t slowdownFactor) {
  unsigned long cycleTime = 10000;  // 10-second cycle
  unsigned long cyclePosition = elapsedTime % cycleTime;
  int32_t speed;
  
  if (cyclePosition < 3000) {
    speed = 2000;  // Fast for 3 seconds
  } else if (cyclePosition < 7000) {
    speed = 500;   // Slow for 4 seconds
  } else {
    speed = 2000;  // Fast for 3 seconds
  }
  
  speed = (speed * slowdownFactor) >> 10;
  stepper.setSpeed(speed);
}

// Array of motion profiles
const int NUM_PROFILES = 4;
MotionProfile profiles[NUM_PROFILES] = {
  standstill,
  constantSpeed,
  oscillating,
  fastSlowFast
};

// Variables
int currentProfile = 0;
unsigned long profileStartTime = 0;


void setup() {
  Serial.begin(115200);
  
  for (int i = 0; i < 4; i++) {
    pinMode(ROTARY_PINS[i], INPUT_PULLUP);
  }
  
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastRotaryCheck >= ROTARY_CHECK_INTERVAL) {
    checkRotarySwitch();
    lastRotaryCheck = currentMillis;
  }
  
  if (currentMillis - lastDistanceCheck >= DISTANCE_CHECK_INTERVAL) {
    readDistance();
    lastDistanceCheck = currentMillis;
  }
  
  if (currentMillis - lastSoundCheck >= SOUND_CHECK_INTERVAL) {
    readSoundLevel();
    lastSoundCheck = currentMillis;
  }
  
  // Update and run stepper motor
  updateStepperMotion(currentMillis);
  stepper.runSpeed();
}

void checkRotarySwitch() {
  int currentRead = 0;
  int connectedInputs = 0;
  
  // Read all inputs simultaneously
  currentRead = (!digitalRead(ROTARY_PINS[0]))     |
                (!digitalRead(ROTARY_PINS[1]) << 1) |
                (!digitalRead(ROTARY_PINS[2]) << 2) |              
                (!digitalRead(ROTARY_PINS[3]) << 3);

  // Count connected inputs using bit manipulation
  connectedInputs = __builtin_popcount(currentRead);
  
  // Check if the read state is valid (only one input connected)
  if (connectedInputs > 1) {
    currentRead = lastStableState; // Invalid state, use last stable state
  }
  
  // If the state has changed, reset the debounce timer
  if (currentRead != readState) {
    lastDebounceTime = millis();
  }
  
  readState = currentRead;
  
  // Check if enough time has passed for debounce
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    // If the state is stable and different from the last stable state, update
    if (readState != lastStableState) {
      lastStableState = readState;
      currentRotaryState = lastStableState;
      
      Serial.print("Rotary switch state: ");
      Serial.println(currentRotaryState);
      
      // Update stepper profile when rotary switch changes
      updateStepperProfile(currentRotaryState % NUM_PROFILES);
    }
  }
}

void readDistance() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  
  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}

void readSoundLevel() {
  int rawSound = analogRead(SOUND_SENSOR_PIN);
  soundLevel = map(rawSound, 0, 4095, 0, 1023);
  
  if (soundLevel > SOUND_THRESHOLD) {
    Serial.println("Loud sound detected!");
  }
}

void updateStepperProfile(int profileIndex) {
  currentProfile = profileIndex;
  profileStartTime = millis();
  Serial.print("Updated to profile: ");
  Serial.println(currentProfile);
}

// In the updateStepperMotion function
void updateStepperMotion(unsigned long currentMillis) {
  unsigned long elapsedTime = currentMillis - profileStartTime;
  uint16_t slowdownFactor = calculateSlowdownFactor(elapsedTime);
  profiles[currentProfile](elapsedTime, slowdownFactor);
  
  if (slowdownFactor == 0 && currentProfile != 0) { // Don't stop if already in standstill mode
    stepper.stop();
    Serial.println("Motion profile completed and stopped");
  }
}


/* TODO:
1. Add ON/OFF button:
    If in motion, stops immediatley (Set ENBL pin LOW or HIGH)
    Otherwise, starts the motion based on selected state.
2. Require back and forth motion of the stepper motor, not driven by simply changing speed/acceleration.
3. When first turned on, go to initial position as determined by position sensor (Closed loop) when first commanded to move, move slowly in this phase, add timeout.
4. Add off state, if rotary switch inputs are 0 (None are connected) disable stepper (ENBL = LOW/HIGH)
*/
