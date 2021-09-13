#include <Aquarius.h>
#include <Ethernet.h>
#include <RF24.h>
#include <RF24Network.h>
#include <SPI.h>

// Control
#define TIME_BETWEEN_PATROLS 5000
#define NUMBER_OF_PHASES 5
typedef enum {
  phase_one,
  phase_two,
  phase_three,
  phase_four,
  phase_five
} program_phase;
program_phase current_phase = phase_one;

program_phase next_phase() {
  return (program_phase)((current_phase + 1) % NUMBER_OF_PHASES);
}

// Pump
byte pump = 47;

// NRF24L01
RF24 radio(49, 53);         // CE, CSN
RF24Network network(radio); // Network
AquariusNetworkCommunicator anc(network);

RF24NetworkHeader car_header(NODE_CAR);
RF24NetworkHeader ct_header(NODE_CT);
RF24NetworkHeader h1_header(NODE_H1);
RF24NetworkHeader h2_header(NODE_H2);
int signal;

// Ethernet
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
EthernetClient client;
bool have_written_db = false;

// Harvesting
byte pot_data[POTS];
bool is_patrolling = false;

// Monitoring
int red_light_pin = 7;
int green_light_pin = 6;
int blue_light_pin = 4;

void RGB_color(int red, int green, int blue) {
  analogWrite(red_light_pin, red);
  analogWrite(green_light_pin, green);
  analogWrite(blue_light_pin, blue);
}

void incolor() { RGB_color(0, 0, 0); }

void white() { RGB_color(255, 255, 255); }

void red() { RGB_color(255, 0, 0); }

void blue() { RGB_color(0, 0, 255); }

void green() { RGB_color(0, 255, 0); }

void yellow() { RGB_color(255, 255, 0); }

void magenta() { RGB_color(255, 0, 255); }

void cyan() { RGB_color(0, 255, 255); }

void led_signal(void (*color)(), int duration) {
  color();
  delay(duration);
  incolor();
}

void led_phase_change() {
  for (int i = 255; i >= 0; i--) {
    RGB_color(i, i, i);
    delay(5);
  }
}

void led_phase_start(int phase_number) {
  for (int i = 0; i < phase_number + 1; i++) {
    led_signal(white, 1000);
    delay(1000);
  }
}

void led_phase_error(int error_code) {
  for (int i = 0; i < error_code; i++) {
    led_signal(red, 500);
    delay(500);
  }
}

void led_phase_success() {
  led_signal(green, 3000);
  delay(500);
}

/*******************************************************************************
*********************************** Main Code **********************************
********************************************************************************/

/**
 *  This function is responsible for requesting data from harvesters and storing
 * it in memory. These functions define phase_one.
 */
bool harvest() {
  Serial.println("Phase 1!");

  byte harvest[8];
  byte null[8];
  memset(null, 0, sizeof(null));
  memset(pot_data, 0x00, POTS);

  signal = SIG_HARVEST_START;

  yellow();
  for (int i = 0; i < HARVESTERS; i++) {

    Serial.print("Harvester: ");
    Serial.println(i + 1);

    RF24NetworkHeader header(NODE_H1 + i);

    if (!anc.writeTimeout(header, &signal, sizeof(signal))) {
      Serial.print("TIMEOUT: Cannot start harvest for harvester: ");
      Serial.println(i + 1);
      led_phase_error(1);
      return false;
    }

    if (!anc.readTimeout(harvest, sizeof(harvest))) {
      Serial.print("TIMEOUT: Could not read data from harvester: ");
      Serial.println(i + 1);
      led_phase_error(2);
      return false;
    }

    if (memcmp(harvest, null, sizeof(harvest)) == 0) {
      Serial.print("ERROR: Data received is wrong for harvester: ");
      Serial.println(i + 1);
      led_phase_error(3);
      return false;
    }

    memcpy(pot_data + i * sizeof(harvest), harvest, sizeof(harvest));
  }
  incolor();
  led_phase_success();
  return true;
}

void print_data() {
  for (int i = 0; i < POTS; i++) {
    Serial.print(pot_data[i]);
    Serial.print(" ");
  }
  Serial.print("\n");
}

/**
 * This function is responsible for saving the requested data in db. This
 * function defines phase_two.
 */
bool persist_data() {

  Serial.println("Phase 2!");

  if (client.connect("si-aquarius.go.ro", 80)) {

    String data;

    magenta();
    for (int i = 0; i < POTS; i++) {
      int h = i / 8 + 1;
      int p = i % 8 + 1;
      data = "";
      data.concat("harvester=");
      data.concat(h);
      data.concat("&pot=");
      data.concat(p);
      data.concat("&humidity=");
      data.concat(pot_data[i]);

      client.println("POST /php/data.php? HTTP/1.1");
      client.println("Host: si-aquarius.go.ro");
      client.println("Content-Type: application/x-www-form-urlencoded");
      client.print("Content-Length: ");
      client.println(data.length());
      client.println();
      client.print(data);
      client.println();
      Serial.print("Wrote record in database. Harvester: ");
      Serial.print(h);
      Serial.print(", Pot: ");
      Serial.println(p);
    }
    incolor();
    client.stop();

    led_phase_success();
    return true;
  } else {

    Serial.println("Could not connect to the database!");
    led_phase_error(1);
    return false;
  }
}

/**
 * This function is responsible for refilling the tank until the car says stop.
 * This function defines phase_three.
 */
bool refill_tank() {
  Serial.println("Phase 3!");

  signal = SIG_REFILL_START;
  if (!anc.writeTimeout(car_header, &signal, sizeof(signal))) {
    Serial.println("TIMEOUT: Seinding SIG_REFILL_START failed!");
    led_phase_error(1);
    return false;
  }

  if (!anc.readTimeout(&signal, sizeof(signal))) {
    Serial.println("TIMEOUT: Receiving acknowledgement failed!");
    led_phase_error(2);
    return false;
  }

  if (signal != SIG_REFILL_ACK) {
    RF24NetworkHeader aux = anc.getReadHeader();
    Serial.print("ERROR: Incorrect response: ");
    Serial.println(signal);
    Serial.print("Message received from node: ");
    Serial.println(aux.from_node);
    led_phase_error(3);
    return false;
  }

  blue();
  digitalWrite(pump, HIGH);

  // Here synchronization is critical + MAX_REFILL_MILLIS and READ_TIMEOUT are
  // different so we work without the library
  RF24NetworkHeader aux;
  unsigned long currentMillis = millis();

  while (millis() - currentMillis <= MAX_REFILL_MILLIS) {
    network.update();
    if (network.available()) {

      network.read(aux, &signal, sizeof(signal));

      if (signal == SIG_REFILL_STOP) {
        digitalWrite(pump, LOW);
        incolor();
        Serial.print("Received stop from car, as it is full!");
        led_phase_success();
        return true;
      }
    }
  }

  digitalWrite(pump, LOW);
  incolor();

  signal = SIG_REFILL_STOP;

  if (!anc.writeTimeout(car_header, &signal, sizeof(signal))) {
    Serial.println("Could not tell the car that the refill is over!");
    Serial.println("Skipping phase as the car will timeout in 10 seconds!");
    led_signal(cyan, 1000);
    delay(6000);
  }

  led_phase_success();
  return true;
}

/**
 * This function is responsible for sending the car in a patrol. This function
 * defines phase_four.
 */
bool send_car_patrol() {
  Serial.println("Phase 4!");

  int signal = SIG_PATROL_START;

  cyan();
  if (!anc.writeTimeout(car_header, &signal, sizeof(signal))) {
    Serial.println("Could not send car to patrol!");
    led_phase_error(1);
    return false;
  }

  bool needs_water[POTS];
  for (int i = 0; i < POTS; i++) {
    needs_water[i] = pot_data[i] < 30;
  }

  if (!anc.writeTimeout(car_header, needs_water, sizeof(needs_water))) {
    Serial.println("Could not send watering data!");
    led_phase_error(2);
    return false;
  }

  // Confirmation
  if (!anc.readTimeout(&signal, sizeof(signal)) && signal != SIG_PATROL_START) {
    Serial.println("Car did not confirm that it started!");
    Serial.println("Check car status!");
    led_phase_error(3);
    return false;
  }
  incolor();

  led_phase_success();
  return true;
}

/**
 * This function is responsible for awaing for the car to finish a patrol
 * This should receive a meesage when a car finishes a patrol.
 */
void await_next_patrol() {
  Serial.println("Phase 5!");

  magenta();
  while (true) {
    Serial.println("Waiting for the car to finish the patrol!");
    if (anc.readTimeout(&signal, sizeof(signal))) {
      if (signal == SIG_PATROL_STOP) {
        incolor();
        led_phase_success();
        return;
      } else {
        RF24NetworkHeader aux = anc.getReadHeader();
        Serial.print("ERROR: Incorrect response: ");
        Serial.println(signal);
        Serial.print("Message received from node: ");
        Serial.println(aux.from_node);
      }
    }
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(pump, OUTPUT);

  pinMode(red_light_pin, OUTPUT);
  pinMode(green_light_pin, OUTPUT);
  pinMode(blue_light_pin, OUTPUT);

  Serial.println("Init NRF24L01");
  SPI.begin();
  radio.begin();
  network.begin(90, NODE_CT);

  Serial.println("Init Ethernet");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet");
  }
}

void loop() {
  led_phase_start(current_phase);
  switch (current_phase) {
  case phase_one:
    if (!harvest()) {
      Serial.println("PHASE-ERROR: Harvest failed!");
      return;
    }
    print_data();
    break;
  case phase_two:
    if (!persist_data()) {
      Serial.println("PHASE-ERROR: Persisting data failed!");
      return;
    }
    break;
  case phase_three:
    if (!refill_tank()) {
      Serial.println("PHASE-ERROR: Refilling failed!");
      return;
    }
    break;
  case phase_four:
    if (!send_car_patrol()) {
      Serial.println("PHASE-ERROR: Patrol failed!");
      return;
    }
    break;
  case phase_five:
    await_next_patrol();
    break;
  }
  current_phase = next_phase();
  led_phase_change();
  delay(3000);
}
