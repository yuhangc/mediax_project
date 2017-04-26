#include <string>

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
    int init();

private:
    // subscriber and publisher
    ros::Subscriber haptic_control_sub;

    // output channels and board number
    int a_out0;

    hid_device *hid_3101;

    // control variables
    double t_step;
    double t_ramp;
    double t_state;
    double t_render;
    double t_state_start;
    double t_vib_start;

    // haptic variables
    std::string haptic_effect;
    int repetition;
    double period_render;  // in secs
    double period_pause;
    double amp_max;

    // state variables
    haptic_control_states state;
    int set_state;
    int vib_state;

    // callback functions
    void hapticCallback(const std_msgs::String::ConstPtr &haptic_msg);

    // other functions
    void render(double amp_max);
};
