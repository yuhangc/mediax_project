#include <string>
#include <opencv2/highgui/highgui.hpp>

#include "inside_out_tracker.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "human_localization");
    InsideOutTracker human_tracker;

    ros::Rate loop_rate(10);
    for (int i = 0; i < 50; i++) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    while (!ros::isShuttingDown())
    {
        ros::spinOnce();
        human_tracker.update();
        loop_rate.sleep();
    }
}
