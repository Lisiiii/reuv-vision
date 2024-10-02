#include "communicator.hpp"
#include "visual_tracker.hpp"
#include <opencv2/highgui.hpp>

int main()
{
    reuv::VisualTracker visual_tracker("/workspaces/reuv/re-uv_ws/src/uv-vision/src/test.mp4");
    reuv::Communicator communicator(visual_tracker);
    visual_tracker.track();
    communicator.start();

    return 0;
}