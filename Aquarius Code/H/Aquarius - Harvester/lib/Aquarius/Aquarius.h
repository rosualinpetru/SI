#ifndef __AQUARIUS__
#define __AQUARIUS__

// Signals
#define SIG_HARVEST_START 1
#define SIG_REFILL_START 2
#define SIG_REFILL_STOP 3
#define SIG_PATROL_START 4
#define SIG_PATROL_STOP 5
#define SIG_NEED_WATER_Y 6
#define SIG_NEED_WATER_N 7

// Refilling
#define MAX_REFILL_MILLIS 5000

// Network
#define NODE_CT 0
#define NODE_CAR 1
#define NODE_H1 2
#define NODE_H2 3

// Pot Mapping
#define HARVESTERS 2
#define POTS 8 * HARVESTERS

// Network
#include <RF24Network.h>

class AquariusNetworkCommunicator {
private:
  RF24Network &network;
  RF24NetworkHeader read_header;

public:
  AquariusNetworkCommunicator(RF24Network &_network);
  bool readTimeout(void *data, int data_size);

  bool writeTimeout(RF24NetworkHeader &header, void *data, int data_size);

  RF24NetworkHeader getReadHeader();
};

#endif
