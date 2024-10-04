#include "communicator.hpp"
#include "visual_tracker.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto communicator = std::make_shared<reuv::Communicator>(
        "communicator",
        reuv::VisualTracker("/workspaces/reuv/re-uv_ws/src/uv-vision/src/test.mp4"),
        "/dev/ttyUSB0",
        100);
    rclcpp::spin(communicator);
    rclcpp::shutdown();

    return 0;
}