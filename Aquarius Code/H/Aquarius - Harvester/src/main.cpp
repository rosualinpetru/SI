#include <RF24.h>
#include <RF24Network.h>
#include <SPI.h>

#include <Aquarius.h>

#define CURRENT NODE_H2

byte pots[] = {40, 10, 40, 65, 95, 20, 100, 35};

RF24 radio(7, 8);
RF24Network network(radio);
AquariusNetworkCommunicator anc(network);

RF24NetworkHeader car_header(NODE_CAR);
RF24NetworkHeader ct_header(NODE_CT);
RF24NetworkHeader h1_header(NODE_H1);
RF24NetworkHeader h2_header(NODE_H2);

void read_data() {}

void setup() {
  Serial.begin(9600);
  SPI.begin();
  radio.begin();
  network.begin(90, CURRENT);
}

void loop() {
  int signal;
  if (anc.readTimeout(&signal, sizeof(signal))) {
    if (signal == SIG_HARVEST_START) {
      Serial.println("Reading sensors!");
      read_data();
      if (anc.writeTimeout(ct_header, pots, sizeof(pots))) {
        Serial.println("Data sent to control tower!");
      } else {
        Serial.println("Could not send data to control tower!");
      }
    }
  }
}
