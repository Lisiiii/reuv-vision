#pragma once

#include <serial/serial.h>

class Communicator {
public:
    Communicator();
    ~Communicator();

private:
    void send();
    void receive();
};