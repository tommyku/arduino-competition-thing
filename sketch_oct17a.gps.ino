#define GPSECHO  true

#include <Servo.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// GPS
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

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

// state keys
const int APP_STATE = 0;
const int INIT_DISTANCE = 1;
const int BOX_OPENED = 2;
const int ARRIVED = 3;
const int SHAKED = 4;

// reading keys
const int READ_DISTANCE = 0;
const int READ_LAT = 1;
const int READ_LON = 2;

// Set destination
float desLat = 22.337255;
float desLong = 114.263155;
bool fakeDestination = true;

// run time states
int states[] = {INITIAL_STAGE, 0, true, false, false};
int readings[] = {0, 0, 0};

Servo servo;

boolean usingInterrupt = false;

void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void servoAction(bool stateChanged) {
  if (!stateChanged) {
    Serial.println('return');
    return;
  }
  if (states[APP_STATE] == INITIAL_STAGE || states[APP_STATE] == ARRIVAL_STAGE) {
    Serial.println('open');
    servo.write(SERVO_OPEN);
  }
  if (states[APP_STATE] == CLOSING_STAGE || states[APP_STATE] == CARRYING_STAGE || states[APP_STATE] == ERROR_STAGE) {
    Serial.println('close');
    servo.write(SERVO_CLOSE);
  }
}

bool isOpen() {
  int val = analogRead(DISTANCE_SENSOR_PIN);
  return (val - states[INIT_DISTANCE] < 100);
}

bool isAtDestination() {
  float sq_distance = (readings[READ_LAT]- desLat) * (readings[READ_LAT]- desLat)
    + (readings[READ_LON] - desLong) * (readings[READ_LON] - desLong);
  return sq_distance < 10000;
}

bool isShaked() {
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
        delay(500); // give it enough time to rest
        return CLOSING_STAGE;
      } else {
        return INITIAL_STAGE;
      }
      break;
    case CLOSING_STAGE:
      return CARRYING_STAGE;
      break;
    case CARRYING_STAGE:
      if (states[BOX_OPENED]) {
        return ERROR_STAGE;
      }
      if (states[ARRIVED]) {
        return ARRIVAL_STAGE;
      }
      break;
    case ARRIVAL_STAGE:
      if (states[BOX_OPENED] && !states[ARRIVED]) {
        return ERROR_STAGE;
      }
      if (states[SHAKED]) {
        return ERROR_STAGE;
      }
      if (!states[ARRIVED]) {
        return CARRYING_STAGE;
      }
      break;
    case ERROR_STAGE:
      return ERROR_STAGE; // always stays in error state
    default:
      return states[APP_STATE];
  }
}

void ledBlink() {
  digitalWrite(LED_PIN_GREEN, HIGH);
  delay(500);
  digitalWrite(LED_PIN_GREEN, LOW);
  delay(500);
  digitalWrite(LED_PIN_GREEN, HIGH);
}

void setup() {
  Serial.begin(115200);
  Serial.println("hello world!");

  // initialize GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);

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

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void loop() {
  // read the values from sensors
  // if a sentence is received, we can check the checksum, parse it...
  /*
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  */

/*
  Serial.print("\nTime: ");
  Serial.print(GPS.hour, DEC); Serial.print(':');
  Serial.print(GPS.minute, DEC); Serial.print(':');
  Serial.print(GPS.seconds, DEC); Serial.print('.');
  Serial.println(GPS.milliseconds);
  Serial.print("Date: ");
  Serial.print(GPS.day, DEC); Serial.print('/');
  Serial.print(GPS.month, DEC); Serial.print("/20");
  Serial.println(GPS.year, DEC);
  Serial.print("Fix: "); Serial.print((int)GPS.fix);
  Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
*/

/*
  if (GPS.fix) {
    readings[READ_LAT] = GPS.latitudeDegrees;
    readings[READ_LON] = GPS.longitudeDegrees;
  }
  */

  // changes states (based on readings)
  states[BOX_OPENED] = isOpen();
  states[ARRIVED] = isAtDestination();
  int originalState = states[APP_STATE];
  states[APP_STATE] = calculateNewState();
  bool stateChanged = originalState != states[APP_STATE];

  if (stateChanged) {
    ledBlink();
  }

  // call the hardwares after state change
  servoAction(stateChanged);
  errorAction();

  Serial.println(states[APP_STATE]);

  delay(2000);
}
