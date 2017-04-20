#include <string>
#include <opencv2/highgui/highgui.hpp>

#include "inside_out_tracker.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "human_localization");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    inside_out_tracker::InsideOutTracker human_tracker(nh, pnh);

    ros::spin();
}
