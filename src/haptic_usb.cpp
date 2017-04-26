#include <iostream>
#include <math.h>

#include "haptic_usb.h"

#define pi 3.14159

// ============================================================================
HapticController::HapticController(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    ros::param::param<int>("~analog_out_channel0", this->a_out0, 1);
    ros::param::param<double>("~t_step", this->t_step, 0.005);
    ros::param::param<double>("~t_ramp", this->t_ramp, 0.015);

    // handler and subscribers
    this->haptic_control_sub = nh.subscribe<std_msgs::String>("haptic_control", 1,
                                                              &HapticController::hapticCallback, this);
}

// ============================================================================
HapticController::~HapticController()
{
    // Program all analog outputs to zero volts.
    for (int ch = 0; ch < 4; ch ++) {
        u_int16_t dacval = volts_USB31XX(BP_10_00V, 0);
        usbAOut_USB31XX(this->hid_3101, ch, dacval, 0);
    }
}

// ============================================================================
int HapticController::init()
{
    // initialize the usb3101 board
    int ret = hid_init();
    if (ret < 0) {
        ROS_ERROR("hid_init failed with return code %d", ret);
        return -1;
    }

    if ((this->hid_3101 = hid_open(MCC_VID, USB3101_PID, NULL)) > 0) {
        ROS_INFO("USB 3101 device is found!");
    } else {
        ROS_ERROR("USB 3010 device not found!");
    }

    // configure digital IOs

    // configure analog output channels
    for (int ch = 0; ch < 4; ch ++) {
        usbAOutConfig_USB31XX(this->hid_3101, ch, BP_10_00V);   // +/- 10V
    }

    // set initial state
    this->state = State_Haptic_Idle;
    this->vib_state = 0;

    // get an initial time
    this->t_state_start = ros::Time::now().toSec();
    this->t_vib_start = ros::Time::now().toSec();
}

// ============================================================================
void HapticController::hapticCallback(const std_msgs::String::ConstPtr &haptic_msg)
{
    // get message data

    // set state to render
    this->set_state = 1;
}

// ============================================================================
void HapticController::update()
{
    this->t_state = ros::Time::now().toSec();

    // state machine
    switch (this->state)
    {
    case State_Haptic_Idle:

//        this->render(0, 0);
        // check if set to render
        if (this->set_state == 1) {
            // reset the set state variable
            this->set_state = 0;

            this->state = State_Haptic_Render;
            this->t_state_start = ros::Time::now().toSec();
            ROS_INFO("Move to state Render!");
        }
        break;
    case State_Haptic_Render:
//        this->render(this->amp_max);

        // check if time out
        if (this->t_state - this->t_state_start >= this->period_render) {
            this->repetition -= 1;
            if (this->repetition <= 0) {
                // set back to idle
                this->state = State_Haptic_Idle;
                ROS_INFO("Move to state Idle!");
            } else {
                // set to pause
                this->state = State_Haptic_Pause;
                this->t_state_start = ros::Time::now().toSec();
                ROS_INFO("Move to state Pause!");
            }
        }
        break;
    case State_Haptic_Pause:
//        this->render(0, 0);

        // check if time out
        if (this->t_state - this->t_state_start >= this->period_pause) {
            this->state = State_Haptic_Render;
            this->t_state_start = ros::Time::now().toSec();
            ROS_INFO("Move to state Render!");
        }
    }
}

// ============================================================================
void HapticController::render(double amp_max)
{
    // calculate desired amplitude
    double amp;
    double t_now = ros::Time::now().toSec() - this->t_vib_start;

    // send signal to sensoray
    int errcode;
    uint16_t dacval;

    dacval = volts_USB31XX(BP_10_00V, amp);
    usbAOut_USB31XX(this->hid_3101, this->a_out0, dacval, 0);
}

// ============================================================================
int main(int argc, char** argv)
{
    // initialize node
    ros::init(argc, argv, "haptic_test_node");

    // a haptic control object
//    HapticController controller = HapticController();
//    controller.init();
//
//    ros::Rate rate(1000);
//    while (ros::ok())
//    {
//        ros::spinOnce();
//        controller.update();
//        rate.sleep();
//    }
}
