#include <RF24Network.h>

#include "Aquarius.h"
#include "Aquarius_config.h"

AquariusNetworkCommunicator::AquariusNetworkCommunicator(RF24Network &_network)
    : network(_network) {}

bool AquariusNetworkCommunicator::readTimeout(void *data, int data_size) {
  int s = 0;
  unsigned long current = millis();
  while (millis() - current < READ_TIMEOUT) {
    network.update();
    if (network.available()) {
      s += network.read(read_header, data, data_size);
      if (s == data_size)
        return true;
    }
  }
  return false;
}

bool AquariusNetworkCommunicator::writeTimeout(RF24NetworkHeader &header,
                                               void *data, int data_size) {
  unsigned long current = millis();
  while (millis() - current < WRITE_TIMEOUT) {
    network.update();
    if (network.write(header, data, data_size)) {
      return true;
    }
  }
  return false;
}

RF24NetworkHeader AquariusNetworkCommunicator::getReadHeader() {
  return read_header;
}
