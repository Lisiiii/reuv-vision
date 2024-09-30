#include "visual_tracker.hpp"

int main()
{
    VisualTracker visual_tracker("/workspaces/reuv/re-uv_ws/src/uv-vision/src/test.mp4");
    visual_tracker.tracking();

    return 0;
}