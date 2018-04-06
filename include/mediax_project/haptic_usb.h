#ifndef HAPTIC_USB_H
#define HAPTIC_USB_H

#include <string>
#include <unordered_map>

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Char.h"
#include "std_msgs/String.h"

#include "libusb/pmd.h"
#include "libusb/usb-3100.h"

typedef enum
{
    State_Haptic_Idle,
    State_Haptic_Render,
    State_Haptic_Pause
} haptic_control_states;


typedef struct {
    int rep;
    double amp;
    double freq;            // in hz
    double period_render;   // in secs
    double period_pause;
} haptic_param;

class HapticController
{
public:
    // constructor
    HapticController(ros::NodeHandle &nh, ros::NodeHandle &pnh);

    // destructor
    ~HapticController();

    // main update function
    void update();

    // initialization
    void init();

private:
    // node handler
    ros::NodeHandle nh_;

    // subscriber and publisher
    ros::Subscriber haptic_control_sub;

    // output channels and board number
    int a_out0;

    hid_device *hid_3101;

    // control variables
    double t_state;
    double t_state_start;
    double t_vib_start;

    // haptic variables
    std::string haptic_effect;
    haptic_param render_param;

    // rendering parameters
    std::unordered_map<std::string, haptic_param> render_params_;

    // state variables
    haptic_control_states state;
    int set_state;

    // callback functions
    void hapticCallback(const std_msgs::String::ConstPtr &haptic_msg);

    // other functions
    void render(bool flag_render);
};

#endif