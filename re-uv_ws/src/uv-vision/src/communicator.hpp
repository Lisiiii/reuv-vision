#pragma once

#include "visual_tracker.hpp"
#include <cstdint>
#include <memory>
#include <opencv2/core/hal/interface.h>
#include <serial/serial.h>
#include <unistd.h>

namespace reuv {

class Communicator {
public:
    Communicator(VisualTracker visual_tracker)
        : visual_tracker_(std::make_unique<VisualTracker>(visual_tracker))
        , serial_("/dev/cp210_serial", 115200, serial::Timeout::simpleTimeout(1000), serial::eightbits, serial::parity_none, serial::stopbits_one) {};
    ~Communicator()
    {
        serial_.close();
    };

    void start()
    {
        while (true) {
            send();
            sleep(1);
        }
    }

private:
    std::unique_ptr<VisualTracker> visual_tracker_;
    serial::Serial serial_;

    void send()
    {
        uint8_t data[1] = { static_cast<uint8_t>(1.14514) };
        serial_.write(data, sizeof(data));
    };
    void receive();
};
}