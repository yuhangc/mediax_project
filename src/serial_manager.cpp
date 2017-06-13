#include "serial_manager.h"
#include <string>
#include <iostream>
#include <sstream>

namespace serial_interface {

    // ============================================================================
    // constructor
    SerialManager::SerialManager(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh)
    {
        std::string device_port;
        pnh.param<std::string>("device_port", device_port, "/dev/ttyACM0");

        // initialize the arduino device
        this->arduino = new CArduinoDevice(device_port, CArduinoDevice::BAUD_115200);
        if (!this->arduino->connect()) {
            ROS_ERROR("Cannot connect to input device!");
        } else {
            ROS_INFO("Input device connected");
        }
    }

    // ============================================================================
    // distructor
    SerialManager::~SerialManager()
    {
        delete this->arduino;
    }

    // ============================================================================
    void SerialManager::init() {
        // initialize publisher
        this->setup_publisher();
    }

    // ============================================================================
    // main update
    void SerialManager::update()
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

        if (read_n > 10) {
            parse_and_publish(message);
        }
    }

    // ============================================================================
    void ImuSerial::setup_publisher() {
        this->serial_data_pub = this->nh_.advertise<std_msgs::Float32MultiArray>("imu_data_raw", 1);
    }

    // ============================================================================
    void ImuSerial::parse_and_publish(const std::string &message) {
        std::stringstream ss(message);

        float value;
        this->imu_data_raw.data.clear();
        for (int i = 0; i < 9; i++) {
            ss >> value;
            this->imu_data_raw.data.push_back(value);
            if (i < 8)
                ss.ignore(2);
        }

        this->serial_data_pub.publish(this->imu_data_raw);
    }

    // ============================================================================
    void OptFlowSerial::setup_publisher() {
        this->serial_data_pub = this->nh_.advertise<std_msgs::Float32MultiArray>("opt_flow", 1);
        time_start = ros::Time().now().toSec();
    }

    // ============================================================================
    void OptFlowSerial::parse_and_publish(const std::string &message) {
        std::stringstream ss(message);

        float value;
        this->opt_flow_data.data.clear();

//        // read in time and ignores it
//        ss >> value; ss.ignore(3);

        // read in x, y velocities
        ss >> value; ss.ignore(2);
        this->opt_flow_data.data.push_back(value / 1000.0);

        ss >> value; ss.ignore(2);
        this->opt_flow_data.data.push_back(value / 1000.0);

        // read in angular velocity
        ss >> value; ss.ignore(2);
        this->opt_flow_data.data.push_back(value / 32768.0 * 300.0 / 180.0 * 3.14159265359);

        // read in optical flow quality
        ss >> value; ss.ignore(2);
        this->opt_flow_data.data.push_back(value);

        // read in altitude
        ss >> value;
        this->opt_flow_data.data.push_back(value / 1000.0);

        this->serial_data_pub.publish(this->opt_flow_data);
    }

}
