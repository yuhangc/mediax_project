#include "imu_manager.h"
#include <string>
#include <iostream>
#include <sstream>

// ============================================================================
// constructor
IMUManager::IMUManager(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh)
{
    // initialize publisher
    this->imu_data_pub = this->nh_.advertise<std_msgs::Float32MultiArray>("imu/data_raw", 1);

    // initialize the arduino device
    this->arduino = new CArduinoDevice("/dev/ttyACM0",CArduinoDevice::BAUD_115200);
    if (!this->arduino->connect()) {
        ROS_ERROR("Cannot connect to input device!");
    } else {
        ROS_INFO("Input device connected");
    }
}

// ============================================================================
// distructor
IMUManager::~IMUManager()
{
    delete this->arduino;
}

// ============================================================================
// main update
void IMUManager::update()
{
    if (!this->arduino->isConnected()) {
        ROS_WARN("Input device not connected! Try to connect again");
        if (this->arduino->connect()) {
            ROS_INFO("Input device connected");
        } else {
            ROS_ERROR("Cannot connect to input device!");
            return;
        }
    }

    std::string message;
    int read_n = this->arduino->read(message);

    std::stringstream ss(message);

    if (read_n > 10) {
        float value;
        this->imu_data_raw.data.clear();
        for (int i = 0; i < 9; i++) {
            ss >> value;
            this->imu_data_raw.data.push_back(value);
            if (i < 8)
                ss.ignore(2);
        }

        this->imu_data_pub.publish(this->imu_data_raw);
    }
}
