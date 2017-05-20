#include "serial_manager.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "opt_flow_bridge");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    serial_interface::OptFlowSerial opt_flow(nh, pnh);
    opt_flow.init();

    ros::Rate loop_rate(1000);
    while (!ros::isShuttingDown())
    {
        opt_flow.update();

        ros::spinOnce();
        loop_rate.sleep();
    }
}
