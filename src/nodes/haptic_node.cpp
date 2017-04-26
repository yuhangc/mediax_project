#include "haptic_usb.h"

int main(int argc, char** argv)
{
    // initialize node
    ros::init(argc, argv, "haptic_test_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // a haptic control object
    HapticController controller = HapticController(nh, pnh);
    controller.init();

    ros::Rate rate(1000);
    while (ros::ok())
    {
        ros::spinOnce();
        controller.update();
        rate.sleep();
    }
}