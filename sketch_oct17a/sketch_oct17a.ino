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
const int LED_PIN_RED = 5;
const int LED_PIN_GREEN = 6;
const int DISTANCE_SENSOR_PIN = A0;

// state variables
const int APP_STATE = 0;
const int INIT_DISTANCE = 1;
const int BOX_OPENED = 2;

// run time states
int states[] = {INITIAL_STAGE, 0, true};
int readings[] = {};

Servo servo;

void servoAction() {
  if (states[APP_STATE] == INITIAL_STAGE || states[APP_STATE] == ARRIVAL_STAGE) {
    servo.write(SERVO_OPEN);
  }
  if (states[APP_STATE] == CLOSING_STAGE || states[APP_STATE] == CARRYING_STAGE || states[APP_STATE] == ERROR_STAGE) {
    servo.write(SERVO_CLOSE);
  }
}

bool isOpen() {
  int val = analogRead(DISTANCE_SENSOR_PIN);
  return (val - states[INIT_DISTANCE] < 100);
}

void errorAction() {
  if (states[APP_STATE] != ERROR_STAGE) {
    return;
  }

  //tone(SPEAKER_PIN, 1000);

  digitalWrite(LED_PIN_GREEN, LOW);
  digitalWrite(LED_PIN_RED, HIGH);
}

int calculateNewState() {
  switch (states[APP_STATE]) {
    case INITIAL_STAGE:
      if (!states[BOX_OPENED]) { // closed
        delay(4000); // give it enough time to rest
        return CLOSING_STAGE;
      } else {
        return INITIAL_STAGE;
      }
      break;
    case CLOSING_STAGE:
      return CARRYING_STAGE;
      break;
    case CARRYING_STAGE:
      /*
      if (states[BOX_OPENED] || abs(states[DISTANCE_SENSOR] - reading[DISTANCE_SENSOR]) > 10 || accelerometer.shaking) {
        // error
        return ERROR_STAGE;
      } else if (reading[GPS] == states[DESTINATION_GPS]) {
        return ARRIVAL_STAGE;
      }
      */
      break;
    case ARRIVAL_STAGE:
      /*
      if ((states[BOX_OPENED] && readings[GPS] != states[DESTINATION_GPS]) || accelerometer.shaking) {
        return ERROR_STAGE;
      } else if (reading[GPS] != states[DESTINATION_GPS]) {
        return CARRYING_STAGE;
      }
      */
      break;
    case ERROR_STAGE:
      return ERROR_STAGE; // always stays in error state
    default:
      return states[APP_STATE];
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("hello world!");

  // initialize servo
  servo.attach(SERVO_PIN);
  servo.write(SERVO_OPEN);

  // initialize spaker
  pinMode(SPEAKER_PIN, OUTPUT);
  noTone(SPEAKER_PIN);

  // initialize distance sensor (must be open at this point)
  states[INIT_DISTANCE] = analogRead(DISTANCE_SENSOR_PIN);

  pinMode(LED_PIN_RED, OUTPUT);
  pinMode(LED_PIN_GREEN, OUTPUT);
  digitalWrite(LED_PIN_RED, LOW);
  digitalWrite(LED_PIN_GREEN, HIGH);
}

void loop() {
  // detect new state
  /*readings = {....};*/

  // changes states (based on readings)
  states[BOX_OPENED] = isOpen();
  states[APP_STATE] = calculateNewState();

  // call the hardwares after state change
  servoAction();
  errorAction();

  delay(500);
}
