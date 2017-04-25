#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

#include "CArduinoDevice.h"

class IMUManager
{
public:
    // constructor
    IMUManager(ros::NodeHandle &nh, ros::NodeHandle &pnh);

    // destructor
    ~IMUManager();

    // main update function
    void update();

private:
    // node handler
    ros::NodeHandle nh_;

    // subscriber and publisher
    ros::Publisher imu_data_pub;

    // arduino device
    CArduinoDevice* arduino;

    // raw data
    std_msgs::Float32MultiArray imu_data_raw;
};
