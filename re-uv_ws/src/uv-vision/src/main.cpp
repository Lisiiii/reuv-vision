#include "communicator.hpp"
#include "visual_tracker.hpp"

int main(int argc, char** argv)
{
    reuv::VisualTracker visual_tracker("/workspaces/reuv/re-uv_ws/src/uv-vision/src/test.mp4");

    rclcpp::init(argc, argv);
    auto communicator = std::make_shared<reuv::Communicator>("communicator", visual_tracker, "/dev/ttyUSB0", 100);
    rclcpp::spin(communicator);
    rclcpp::shutdown();

    return 0;
}