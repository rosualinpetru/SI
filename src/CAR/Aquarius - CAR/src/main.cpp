#include <AFMotor.h>
#include <Aquarius.h>
#include <EEPROMex.h>
#include <HCSR04.h>
#include <QTRSensors.h>
#include <RF24.h>
#include <RF24Network.h>
#include <SPI.h>

#define IR_SENSOR_THRESHOLD 10

#define MIN_SPEED 100
#define MAX_SPEED 220

#define EEPROM_ADDR_MIN_ON 0
#define EEPROM_ADDR_MAX_ON 100

#define IR_QTR_COUNT 5

#define STP_STEPS 512

#define VOLTAGE_OFFSET 20
#define VOLTAGE_TRESHOLD 11.1

#define ADDR_CALIBRATED_MINIMUM_ON 0
#define ADDR_CALIBRATED_MAXIMUM_ON 100

#define WATERING_TIME 4000

#define MIN_EMPTY_DIST 4

// Line follower QTR
byte ir1 = 26;
byte ir2 = 27;
byte ir3 = 28;
byte ir4 = 29;
byte ir5 = 30;
QTRSensors qtr;
unsigned int raw_ir_data[IR_QTR_COUNT];
bool ir_data[IR_QTR_COUNT];

// Stepper
int stp0 = 36, stp1 = 37, stp2 = 38, stp3 = 39;

// Pump
byte pump = 34;

// Voltage sensor
int voltage = A15;

// NRF24L01
RF24 radio(2, 53);
RF24Network network(radio);
AquariusNetworkCommunicator anc(network);

RF24NetworkHeader car_header(NODE_CAR);
RF24NetworkHeader ct_header(NODE_CT);
RF24NetworkHeader h1_header(NODE_H1);
RF24NetworkHeader h2_header(NODE_H2);
int signal;

// Water level (trig = 23, echo = 22)
HCSR04 water_level(23, 22);

// Motors
AF_DCMotor fl_motor(3);
AF_DCMotor fr_motor(2);
AF_DCMotor bl_motor(4);
AF_DCMotor br_motor(1);

// Directions
enum direction { forward, left, right, stop };

// Data
bool needs_water[POTS];
bool is_patrolling;
int stop_counter, column_counter;

/**
 *  This function is responsible mapping raw qtr data to bools
 */
void read_line() {
  qtr.readLineBlack(raw_ir_data);
  for (int i = 0; i < IR_QTR_COUNT; i++) {
    ir_data[i] = raw_ir_data[i] < IR_SENSOR_THRESHOLD;
  }
}

/**
 * This function is responsible for reading the current voltage of the battery
 */
double read_voltage() {
  int voltage_raw;
  voltage_raw = analogRead(voltage);
  return (double)((double)map(voltage_raw, 0, 1023, 0, 2500) + VOLTAGE_OFFSET) /
         100.0;
}

/**
 * These functions are responsible for controlling the stepper
 */

void step_right(int step) {
  switch (step % 4) {
  case 0:
    digitalWrite(stp0, HIGH);
    digitalWrite(stp1, LOW);
    digitalWrite(stp2, LOW);
    digitalWrite(stp3, LOW);
    break;
  case 1:
    digitalWrite(stp0, LOW);
    digitalWrite(stp1, HIGH);
    digitalWrite(stp2, LOW);
    digitalWrite(stp3, LOW);
    break;
  case 2:
    digitalWrite(stp0, LOW);
    digitalWrite(stp1, LOW);
    digitalWrite(stp2, HIGH);
    digitalWrite(stp3, LOW);
    break;
  case 3:
    digitalWrite(stp0, LOW);
    digitalWrite(stp1, LOW);
    digitalWrite(stp2, LOW);
    digitalWrite(stp3, HIGH);
    break;
  }
}

void step_left(int step) {
  switch (step % 4) {
  case 0:
    digitalWrite(stp0, LOW);
    digitalWrite(stp1, LOW);
    digitalWrite(stp2, LOW);
    digitalWrite(stp3, HIGH);
    break;
  case 1:
    digitalWrite(stp0, LOW);
    digitalWrite(stp1, LOW);
    digitalWrite(stp2, HIGH);
    digitalWrite(stp3, LOW);
    break;
  case 2:
    digitalWrite(stp0, LOW);
    digitalWrite(stp1, HIGH);
    digitalWrite(stp2, LOW);
    digitalWrite(stp3, LOW);
    break;
  case 3:
    digitalWrite(stp0, HIGH);
    digitalWrite(stp1, LOW);
    digitalWrite(stp2, LOW);
    digitalWrite(stp3, LOW);
  }
}

/**
 * This function is responsible for setting the speed of one DC motor
 */
void set_speed(AF_DCMotor motor, int speed) { motor.setSpeed(speed); }

/**
 * This function is responsible for setting the speed of all DC motors
 */
void set_speed_all(int speed) {
  fl_motor.setSpeed(speed);
  bl_motor.setSpeed(speed);
  fr_motor.setSpeed(speed);
  br_motor.setSpeed(speed);
}

/**
 * This function is responsible for setting the direction of the car
 */
void set_direction(direction d) {
  switch (d) {
  case forward:
    fl_motor.run(FORWARD);
    fr_motor.run(FORWARD);
    bl_motor.run(FORWARD);
    br_motor.run(FORWARD);
    break;
  case left:
    fl_motor.run(BACKWARD);
    fr_motor.run(FORWARD);
    bl_motor.run(BACKWARD);
    br_motor.run(FORWARD);
    break;
  case right:
    fl_motor.run(FORWARD);
    fr_motor.run(BACKWARD);
    bl_motor.run(FORWARD);
    br_motor.run(BACKWARD);
    break;
  case stop:
    fl_motor.run(RELEASE);
    fr_motor.run(RELEASE);
    bl_motor.run(RELEASE);
    br_motor.run(RELEASE);
    break;
  }
}

/**
 * These functions are responsible for watering a pot
 */

void water_pot_left() {
  for (int a = 0; a < STP_STEPS; a++) {
    step_left(a);
    delay(2);
  }
  // digitalWrite(pump, LOW);
  delay(WATERING_TIME);
  // digitalWrite(pump, HIGH);
  for (int a = 0; a < STP_STEPS; a++) {
    step_right(a);
    delay(2);
  }
}

void water_pot_right() {
  for (int a = 0; a < STP_STEPS; a++) {
    step_right(a);
    delay(2);
  }
  // digitalWrite(pump, LOW);
  delay(WATERING_TIME);
  // digitalWrite(pump, HIGH);
  for (int a = 0; a < STP_STEPS; a++) {
    step_left(a);
    delay(2);
  }
}

double read_water_level() {
  int read_times = 0;
  double s = 0, level;
  for (int i = 0; i < 1024; i++) {
    level = water_level.dist();
    if (level == 0.0 || level > 20) {
      read_times--;
    } else {
      s += level;
    }
  }
  return s / read_times;
}

/**
 * This function is responsible for handling the automatic refill
 */
void refill() {
  RF24NetworkHeader aux;

  // We ACK only if the water level is not close to the MIN_EMPTY_DIST;
  // In case a read is not 100% precise the loop might desynchronize the CT and
  // the CAR
  if (read_water_level() - MIN_EMPTY_DIST < 0.3) {
    Serial.println("No water needed!");
    Serial.println("The water is to close to the maximum value");
    return;
  }

  signal = SIG_REFILL_ACK;

  if (!anc.writeTimeout(ct_header, &signal, sizeof(signal))) {
    Serial.println("ERROR: Response SIG_NEED_WATER_* not sent!");
    return;
  }

  delay(1000);

  Serial.println("Pouring water!");
  unsigned long currentMillis = millis();
  int level;
  while (millis() - currentMillis <= MAX_REFILL_MILLIS + 6000) {

    level = read_water_level();
    if (level != 0.0 && level - MIN_EMPTY_DIST < 0) {
      // IF THIS HAPPENS THIS IS REALLY BAD BE CAREFULL
      signal = SIG_REFILL_STOP;
      if (!anc.writeTimeout(ct_header, &signal, sizeof(signal))) {
        Serial.println("ERROR: STOP REFILL NOT SENT!");
        set_speed_all(MAX_SPEED);
        set_direction(forward);
        delay(300);
        set_direction(stop);
        set_speed_all(0);
      }
      return;
    }

    network.update();

    if (network.available()) {
      network.read(aux, &signal, sizeof(signal));
    }

    if (signal == SIG_REFILL_STOP) {
      Serial.println("Refill succeeded!");
      return;
    }
  }
  Serial.println("WARNING: Refill TIMEOUT!");
}

/**
 * These functions are responsible for running the circuit and water plants
 * along the way.
 */

bool confirm_start() {
  signal = SIG_PATROL_START;
  Serial.println("Confirming patrol!");
  if (!anc.writeTimeout(ct_header, &signal, sizeof(signal))) {
    Serial.println("ERROR: Confirming patrol failed!");
    return false;
  }
  return true;
}

bool read_watering_data() {
  Serial.println("Reading watering data!");
  if (!anc.readTimeout(needs_water, sizeof(needs_water))) {
    Serial.println("ERROR: Reading watering data failed!");
    return false;
  }
  return true;
}

bool finish_patrol() {
  signal = SIG_PATROL_STOP;
  Serial.println("Finish patrol!");
  if (!anc.writeTimeout(ct_header, &signal, sizeof(signal))) {
    Serial.println("ERROR: Tell CT patrol ended failed!");
    return false;
  }
  return true;
}

void patrol() {

  set_speed_all(MIN_SPEED);
  set_direction(forward);

  delay(500);

  while (true) {
    read_line();
    if (ir_data[0] && ir_data[1] && ir_data[2] && ir_data[3] && ir_data[4]) {
      set_direction(stop);

      column_counter = stop_counter / 4;

      switch (column_counter) {
      case 0:
        if (needs_water[stop_counter]) {
          water_pot_left();
          delay(500);
        }

        break;
      case 1:
        if (needs_water[stop_counter]) {
          water_pot_left();
          delay(500);
        }
        if (needs_water[stop_counter + 4]) {
          water_pot_right();
          delay(500);
        }
        break;
      case 2:
        if (needs_water[stop_counter + 4]) {
          water_pot_right();
          delay(500);
        }
        break;
      case 3:
        column_counter = 0;
        stop_counter = 0;
        Serial.println("Finish patrol!");
        return;
      }

      stop_counter++;

      set_speed_all(MIN_SPEED);
      set_direction(forward);

      while (ir_data[0] && ir_data[1] && ir_data[2] && ir_data[3] &&
             ir_data[4]) {
        read_line();
      }
      continue;
    }

    if (ir_data[0] && !ir_data[2] && !ir_data[4]) {
      set_speed_all(MAX_SPEED);
      set_direction(left);
      while (!ir_data[3]) {
        read_line();
      }
      set_speed_all(MIN_SPEED);
      set_direction(forward);
      continue;
    }
    if (!ir_data[0] && !ir_data[2] && ir_data[4]) {
      set_speed_all(MAX_SPEED);
      set_direction(right);
      while (!ir_data[1]) {
        read_line();
      }
      set_speed_all(MIN_SPEED);
      set_direction(forward);
      continue;
    }

    if (ir_data[1] && !ir_data[3]) {
      set_speed_all(MAX_SPEED);
      set_direction(left);
      continue;
    }
    if (!ir_data[1] && ir_data[3]) {
      set_speed_all(MAX_SPEED);
      set_direction(right);
      continue;
    }
    if (ir_data[2]) {
      set_speed_all(MIN_SPEED);
      set_direction(forward);
      continue;
    }
  }
}

void setup() {
  Serial.begin(9600);

  // Line follower QTR
  qtr.setTypeRC();
  byte ir_sensors[] = {ir1, ir2, ir3, ir4, ir5};
  qtr.setSensorPins(ir_sensors, IR_QTR_COUNT);
  qtr.calibrate();
  EEPROM.readBlock<unsigned int>(EEPROM_ADDR_MIN_ON, qtr.calibrationOn.minimum,
                                 IR_QTR_COUNT);
  EEPROM.readBlock<unsigned int>(EEPROM_ADDR_MAX_ON, qtr.calibrationOn.maximum,
                                 IR_QTR_COUNT);

  // NRF24L01
  SPI.begin();
  radio.begin();
  network.begin(90, NODE_CAR);

  // Pump
  pinMode(pump, OUTPUT);
  digitalWrite(pump, HIGH);

  // Motors
  set_speed_all(MIN_SPEED);
}

void loop() {
  double voltage = read_voltage();
  if (voltage < VOLTAGE_TRESHOLD) {
    Serial.println("WARNING: Low battery level! Cannot operate!");
  }

  if (anc.readTimeout(&signal, sizeof(signal))) {
    if (signal == SIG_REFILL_START) {
      Serial.println("Refilling!");
      refill();
    }

    if (signal == SIG_PATROL_START) {
      Serial.println("Patrolling!");
      if (read_watering_data()) {
        if (confirm_start()) {
          patrol();
          while (!finish_patrol())
            ;
        }
      }
    }
  }
}
