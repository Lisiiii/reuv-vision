#include "visual_tracker.hpp"
#include <opencv2/highgui.hpp>

int main()
{
    reuv::VisualTracker visual_tracker("/workspaces/reuv/re-uv_ws/src/uv-vision/src/test.mp4");
    visual_tracker.track();

    return 0;
}