#include <Servo.h>

// application states
const int INITIAL_STAGE = 0;
const int CLOSING_STAGE = 1;
const int CARRYING_STAGE = 2;
const int ARRIVAL_STAGE = 3;
const int ERROR_STAGE = 4;

// servo
const int SERVO_OPEN = 0;
const int SERVO_CLOSE = 90;

// pins
const int SERVO_PIN = 8;
const int SPEAKER_PIN = 9;

// state variables
const int APP_STATE = 0;
const int SERVO = 1;

// run time states
int states[] = {INITIAL_STAGE};
int readings[] = {};

Servo servo;

void servoAction() {
  if (states[APP_STATE] == INITIAL_STAGE || states[APP_STATE] == ARRIVAL_STAGE) {
    servo.write(SERVO_OPEN);
  }
  if (states[APP_STATE] == CLOSING_STAGE || states[APP_STATE] == CARRYING_STAGE) {
    servo.write(SERVO_CLOSE);
  }
}

void errorAction() {
  if (states[APP_STAGE] != ERROR_STATE) {
    return;
  }

  tone(SPEAKER_PIN, 1000);
  // turn on LED here
}

void calculateNewState() {
  switch (states[APP_STATE]) {
    case INITIAL_STAGE:
      return CLOSING_STAGE;
      break;
    case CLOSING_STAGE:
      // useless?
      return CARRYING_STAGE;
      break;
    case CARRYING_STAGE:
      if (abs(state[DISTANCE_SENSOR] - reading[DISTANCE_SENSOR]) > 10 || accelerometer.shaking || reading[OPENED]) {
        // error
        return ERROR_STATE;
      } else if (reading[GPS] == states[DESTINATION_GPS]) {
        return ARRIVAL_STAGE;
      }
      break;
    case ARRIVAL_STAGE:
      if ((reading[OPENED] && reading[GPS] != states[DESTINATION_GPS]) || accelerometer.shaking) {
        return ERROR_STATE;
      } else if (reading[GPS] != states[DESTINATION_GPS]) {
        return CARRYING_STAGE;
      }
      break;
    case ERROR_STATE:
      return ERROR_STATE;
    default:
      return states[APP_STATE];
  }
}

void setup() {
  // initialize servo
  servo.attach(SERVO_PIN);

  // initialize spaker
  pinMode(SPEAKER_PIN, OUTPUT);
}

void loop() {
  // detect new state
  readings = {....};

  // changes states (based on readings)
  states[APP_STATE] = calculateNewState();

  // call the hardwares after state change
  servoAction();
  errorAction();

  delay(500);
}
