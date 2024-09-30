#pragma once

#include <serial/serial.h>

class Communication {
public:
    Communication();
    ~Communication();

private:
    void send();
    void receive();
};