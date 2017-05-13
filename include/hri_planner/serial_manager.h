#ifndef SERIAL_MANAGER_H
#define SERIAL_MANAGER_H

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

#include "CArduinoDevice.h"

namespace serial_interface {

    // A general serial manager virtual class
    // Methods setup_publisher() and parse_and_publish() need to be implemented
    // by child classes
    class SerialManager
    {
    public:
        // constructor
        SerialManager(ros::NodeHandle &nh, ros::NodeHandle &pnh);

        // initialization
        void init();

        // destructor
        virtual ~SerialManager();

        // main update function
        void update();

    protected:
        // node handler
        ros::NodeHandle nh_;
        // subscriber and publisher
        ros::Publisher serial_data_pub;

        // arduino device
        CArduinoDevice* arduino;

        virtual void setup_publisher() = 0;
        virtual void parse_and_publish(const std::string &message) = 0;
    };

    // ============================================================================
    // reads and parse serial data from IMU
    class ImuSerial: public SerialManager
    {
    public:
        ImuSerial(ros::NodeHandle &nh, ros::NodeHandle &pnh) : SerialManager(nh, pnh) {};
        ~ImuSerial() {};

    protected:
        void setup_publisher();
        void parse_and_publish(const std::string &message);

    private:
        // raw data
        std_msgs::Float32MultiArray imu_data_raw;
    };

    // ============================================================================
    // reads and parse serial data from px4flow sensor
    class OptFlowSerial: public SerialManager
    {
    public:
        OptFlowSerial(ros::NodeHandle &nh, ros::NodeHandle &pnh) : SerialManager(nh, pnh) {};
        ~OptFlowSerial() {};

    protected:
        void setup_publisher();
        void parse_and_publish(const std::string &message);

    private:
        std_msgs::Float32MultiArray opt_flow_data;
    };

}

#endif