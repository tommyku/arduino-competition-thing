#include <Servo.h>
#include <Adafruit_GPS.h>
#include <SparkFun_ADXL345.h>

#define GPSSerial Serial

Adafruit_GPS GPS(&GPSSerial);

#define GPSECHO  false

ADXL345 adxl = ADXL345();             // USE FOR I2C COMMUNICATION

boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

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

// run time states
int states[] = {INITIAL_STAGE, 0, true, false, false};
int readings[] = {0, 0, 0};

Servo servo;

int acceleration[10] = {};

double shaken_threshold = 2.0;

int cycleBeforeFaking = 40;

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

void errorAction() {
  if (states[APP_STATE] != ERROR_STAGE) {
    return;
  }

  tone(SPEAKER_PIN, 1000);

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
      if (states[ARRIVED]) {
        return ARRIVAL_STAGE;
      }
      if (states[BOX_OPENED] || states[SHAKED]) {
        return ERROR_STAGE;
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

    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);

  useInterrupt(true);

  delay(1000);


  adxl.powerOn();                     // Power on the ADXL345
  adxl.setRangeSetting(16);           // Give the range settings
  adxl.setSpiBit(0);                  // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to 1
  adxl.setActivityXYZ(1, 0, 0);       // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setActivityThreshold(75);      // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)
  adxl.setInactivityXYZ(1, 0, 0);     // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setInactivityThreshold(75);    // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  adxl.setTimeInactivity(10);         // How many seconds of no activity is inactive?
  adxl.setTapDetectionOnXYZ(0, 0, 1); // Detect taps in the directions turned ON "adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setTapThreshold(50);           // 62.5 mg per increment
  adxl.setTapDuration(15);            // 625 μs per increment
  adxl.setDoubleTapLatency(80);       // 1.25 ms per increment
  adxl.setDoubleTapWindow(200);       // 1.25 ms per increment
  adxl.setFreeFallThreshold(7);       // (5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(30);       // (20 - 70) recommended - 5ms per increment
  adxl.InactivityINT(1);
  adxl.ActivityINT(1);
  adxl.FreeFallINT(1);
  adxl.doubleTapINT(1);
  adxl.singleTapINT(1);
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

int activity_record() {
  byte interrupts = adxl.getInterruptSource();
  // Free Fall Detection
  if(adxl.triggered(interrupts, ADXL345_FREE_FALL)){
    Serial.println("*** FREE FALL ***");
    return 1;
  }
  // Double Tap Detection
  if(adxl.triggered(interrupts, ADXL345_DOUBLE_TAP)){
    Serial.println("*** DOUBLE TAP ***");
    return 1;
  }
  // Tap Detection
  if(adxl.triggered(interrupts, ADXL345_SINGLE_TAP)){
    Serial.println("*** TAP ***");
    return 1;
  }
  return 0;
}

bool is_being_shaken(){ // Determine whether the box is rudely treated by calculation
  double record = 0;
  for (int i = 0; i < 10; i++) record += (double)acceleration[i] / (10 - i);
  Serial.println(record);
  if (record >= shaken_threshold) return true;
  return false;
}

void loop() {
  // read the values from sensors
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  if (GPS.fix) {
    readings[READ_LAT] = GPS.latitudeDegrees;
    readings[READ_LON] = GPS.longitudeDegrees;
  }

  Serial.print("The state is ");
  Serial.println(states[APP_STATE]);
  Serial.print("Cycle before faking");
  Serial.println(cycleBeforeFaking);

  if (cycleBeforeFaking <= 0 && states[APP_STATE] == CARRYING_STAGE) {
    readings[READ_LAT] = 0;
    readings[READ_LON] = 0;
    desLat = 0;
    desLong = 0;
  } else {
    cycleBeforeFaking = cycleBeforeFaking - 1;
  }

  for (int i = 0; i < 9; i++) {
    acceleration[i] = acceleration[i+1];
  }
  acceleration[9] = activity_record();

  // changes states (based on readings)
  states[BOX_OPENED] = isOpen();
  states[ARRIVED] = isAtDestination();
  states[SHAKED] = is_being_shaken();
  int originalState = states[APP_STATE];
  states[APP_STATE] = calculateNewState();
  bool stateChanged = originalState != states[APP_STATE];

  if (stateChanged) {
    ledBlink();
  }

  // call the hardwares after state change
  servoAction(stateChanged);
  errorAction();

  delay(500);
}
