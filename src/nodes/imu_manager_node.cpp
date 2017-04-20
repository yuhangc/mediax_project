#include "imu_manager.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_manager");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    IMUManager imu_manager(nh, pnh);

    ros::Rate loop_rate(500);
    while (!ros::isShuttingDown())
    {
        imu_manager.update();

        ros::spinOnce();
        loop_rate.sleep();
    }
}
