#pragma once

#include <serial/serial.h>

namespace reuv {

class Communicator {
public:
    Communicator();
    ~Communicator();

private:
    void send();
    void receive();
};
}