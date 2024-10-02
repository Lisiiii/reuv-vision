#pragma once

#include "protocol.hpp"
#include "visual_tracker.hpp"

#include <cstdint>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

namespace reuv {

class Communicator : public rclcpp::Node {
public:
    Communicator(std::string name, VisualTracker visual_tracker, std::string serial_port, int control_rate_hz = 100)
        : Node(name)
        , visual_tracker_(std::make_unique<VisualTracker>(visual_tracker))
        , serial_(serial_port, 115200, serial::Timeout::simpleTimeout(1000), serial::eightbits, serial::parity_none, serial::stopbits_one)
        , control_rate_hz_(control_rate_hz)
    {
        start();
        visual_tracker_->track();
    };

    ~Communicator()
    {
        main_process_timer_.reset();
        serial_.close();
    };

    void start()
    {
        main_process_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / control_rate_hz_)),
            [this]() { send_callback(); });
    }

private:
    std::unique_ptr<VisualTracker> visual_tracker_;
    std::shared_ptr<rclcpp::TimerBase> main_process_timer_;
    serial::Serial serial_;
    int control_rate_hz_;

    void send_callback()
    {
        RCLCPP_INFO(get_logger(), "%f", visual_tracker_->offset_);
        protocol::Command command {
            .x = 0,
            .y = 0,
            .w = visual_tracker_->offset_,
            .event = reuv::protocol::Event::Run
        };
        serial_.write(reinterpret_cast<uint8_t*>(&command), sizeof(command));
    };
    void on_receive();
};
}