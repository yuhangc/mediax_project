#include <string>
#include <opencv2/highgui/highgui.hpp>

#include "inside_out_tracker.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker_calibration");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    inside_out_tracker::InsideOutTracker tracker(nh, pnh);
    tracker.run_calibration_aruco();

    ros::spin();
}