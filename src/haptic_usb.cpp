#include <iostream>
#include <math.h>

#include "haptic_usb.h"

#define pi 3.14159

// ============================================================================
HapticController::HapticController(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh)
{
    ros::param::param<int>("~analog_out_channel0", this->a_out0, 1);

    // get haptic rendering parameters
    haptic_param t_param;

    pnh.param<int>("repetition_repel", t_param.rep, 2);
    pnh.param<double>("amplitude_repel", t_param.amp, 2.0);
    pnh.param<double>("frequency_repel", t_param.freq, 50);
    pnh.param<double>("period_render_repel", t_param.period_render, 1.0);
    pnh.param<double>("period_pause_repel", t_param.period_pause, 0.5);
    this->render_params_.insert({"repel", t_param});

    pnh.param<int>("repetition_attract", t_param.rep, 3);
    pnh.param<double>("amplitude_attract", t_param.amp, 1.0);
    pnh.param<double>("frequency_attract", t_param.freq, 50);
    pnh.param<double>("period_render_attract", t_param.period_render, 0.3);
    pnh.param<double>("period_pause_attract", t_param.period_pause, 0.3);
    this->render_params_.insert({"attract", t_param});

    // handler and subscribers
    this->haptic_control_sub = nh_.subscribe<std_msgs::String>("haptic_control", 1,
                                                              &HapticController::hapticCallback, this);
}

// ============================================================================
HapticController::~HapticController()
{
    // Program all analog outputs to zero volts.
    for (uint8_t ch = 0; ch < 4; ch ++) {
        uint16_t dacval = volts_USB31XX(BP_10_00V, 0);
        usbAOut_USB31XX(this->hid_3101, ch, dacval, 0);
    }
}

// ============================================================================
void HapticController::init()
{
    // initialize the usb3101 board
    int ret = hid_init();
    if (ret < 0) {
        ROS_ERROR("hid_init failed with return code %d", ret);
        return;
    }

    if ((this->hid_3101 = hid_open(MCC_VID, USB3101_PID, NULL)) > 0) {
        ROS_INFO("USB 3101 device is found!");
    } else {
        ROS_ERROR("USB 3101 device not found!");
        return;
    }

    // configure analog output channels
    for (uint8_t ch = 0; ch < 4; ch ++) {
        usbAOutConfig_USB31XX(this->hid_3101, ch, BP_10_00V);   // +/- 10V
    }

    // set initial state
    this->state = State_Haptic_Idle;

    // get an initial time
    this->t_state_start = ros::Time::now().toSec();
    this->t_vib_start = ros::Time::now().toSec();
}

// ============================================================================
void HapticController::hapticCallback(const std_msgs::String::ConstPtr &haptic_msg)
{
    // get message data
    if (haptic_msg->data == "repel") {
        this->render_param = this->render_params_["repel"];
    }
    else if (haptic_msg->data == "attract") {
        this->render_param = this->render_params_["attract"];
    }

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

        this->render(false);
        // check if set to render
        if (this->set_state == 1) {
            // reset the set state variable
            this->set_state = 0;

            this->state = State_Haptic_Render;
            this->t_state_start = ros::Time::now().toSec();
            ROS_DEBUG("Move to state Render!");
        }
        break;
    case State_Haptic_Render:
        this->render(true);

        // check if time out
        if (this->t_state - this->t_state_start >= render_param.period_render) {
            render_param.rep -= 1;
            if (render_param.rep <= 0) {
                // set back to idle
                this->state = State_Haptic_Idle;
                ROS_DEBUG("Move to state Idle!");
            } else {
                // set to pause
                this->state = State_Haptic_Pause;
                this->t_state_start = ros::Time::now().toSec();
                ROS_DEBUG("Move to state Pause!");
            }
        }
        break;
    case State_Haptic_Pause:
        this->render(false);

        // check if time out
        if (this->t_state - this->t_state_start >= render_param.period_pause) {
            this->state = State_Haptic_Render;
            this->t_state_start = ros::Time::now().toSec();
            ROS_DEBUG("Move to state Render!");
        }
    }
}

// ============================================================================
void HapticController::render(bool flag_render)
{
    // calculate desired amplitude
    double amp;
    double t_now = ros::Time::now().toSec() - this->t_vib_start;

    if (flag_render) {
        amp = render_param.amp * std::sin(2 * pi * render_param.freq * t_now);
    }
    else {
        amp = 0;
    }

    // send signal to mcc usb device
    uint16_t dacval;

    dacval = volts_USB31XX(BP_10_00V, (float)amp);
    usbAOut_USB31XX(this->hid_3101, (uint8_t)this->a_out0, dacval, 0);
}
