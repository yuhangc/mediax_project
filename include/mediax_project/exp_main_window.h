#ifndef EXP_MAIN_WINDOW_H
#define EXP_MAIN_WINDOW_H

#include <QMainWindow>
#include <QTimer>

#include <fstream>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Char.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/Odometry.h"

#include "json.hpp"
using json = nlohmann::json;

typedef enum {
    exp_state_idle,
    exp_state_training,
    exp_state_pre_experiment,
    exp_state_experimenting
} exp_state_type;

namespace Ui {
class ExpMainWindow;
}

class ExpMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ExpMainWindow(QWidget *parent = 0);
    ~ExpMainWindow();

    // Initialization
    void Init();

    // a function that gets yaw angle from quarternion
    double get_yaw_quat(geometry_msgs::Quaternion quat) {
        tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        double yaw, pitch, roll;

        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return yaw;
    }

private:
    Ui::ExpMainWindow *ui;

    // Timer for GUI update
    QTimer update_timer_;

    // node handler
    ros::NodeHandle nh_;

    // subscribers
    ros::Subscriber robot_state_sub_;
    ros::Subscriber robot_odom_sub_;
    ros::Subscriber human_pose2d_sub_;
    ros::Subscriber robot_pose2d_sub_;
    ros::Subscriber opt_flow_sub_;
    ros::Subscriber sys_msg_sub_;

    // publishers
    ros::Publisher set_robot_state_pub_;
    ros::Publisher haptic_control_pub_;
    ros::Publisher reset_human_tracker_pub_;
    ros::Publisher reset_robot_tracker_pub_;

    // variables to record
    int robot_state_;
    geometry_msgs::Pose2D human_pose_;
    geometry_msgs::Pose2D robot_pose_;
    geometry_msgs::Twist robot_vel_curr_;
    geometry_msgs::Twist robot_vel_cmd_;

    // experiment control variables
    int exp_num_;
    int config_num_;

    int trial_num_;
    int cond_num_;

    int num_cond_total_;
    std::vector<int> num_trial_total_;
    std::vector<int> num_training_total_;
    std::vector< std::vector<int> > robot_action_list_;

    exp_state_type exp_state_;

    // target poses
    std::vector<double> y_targets_;

    // for publishing
    std_msgs::String set_robot_state_;
    std_msgs::String haptic_msg_;

    // for data saving
    std::string dir_saving_;
    std::ofstream data_stream_;

    bool flag_start_data_saving_;

    // pre-set file loading and saving locations
    std::string dir_loading_pre_set_;
    std::string dir_saving_pre_set_;

    // timer for sending robot action
    double t_exp_start_;
    double t_robot_action_delay_;

    // flags
    bool flag_protocol_loaded_;
    bool flag_dir_saving_set_;

    bool flag_use_odom_as_pose_;

    bool flag_start_exp_requested_;
    bool flag_stop_exp_requested_;

    bool flag_exp_training_;

    bool flag_robot_action_sent_;

    // load experiment protocols
    void load_protocol(std::string file_name);
    void load_protocol_exp1(json& proto_parser);
    void load_protocol_exp2(json& proto_parser);

    // state machine functions
    void state_machine_exp1_config0();
    void state_machine_exp1_config1();
    void state_machine_exp2();

    // functions for data saving
    void save_exp_data();
    void start_data_saving();
    void stop_data_saving();

    // send desired robot action and haptic cue
    void set_robot_action_delay();
    void send_robot_action();
    void send_haptic_cue();

    // update UI components
    void set_cond_trial_text();

    // callback functions
    void human_pose_callback(const geometry_msgs::Pose2D::ConstPtr& pose2d_msg);
    void robot_pose_callback(const geometry_msgs::Pose2D::ConstPtr& pose2d_msg);

    void robot_state_callback(const std_msgs::String::ConstPtr& state_msg);
    void robot_odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);

    void opt_flow_callback(const std_msgs::Float32MultiArrayConstPtr &opt_flow_msg);

    void sys_msg_callback(const std_msgs::String::ConstPtr& sys_msg);

signals:
    void human_pose_received(double x, double y, double th);
    void robot_pose_received(double x, double y, double th);

private slots:
    void update_gui_info();
    void exp_state_update();
    void on_combo_set_state_currentTextChanged(const QString &arg1);
    void on_combo_haptic_type_currentTextChanged(const QString &arg1);
    void on_button_set_state_clicked();
    void on_button_send_haptic_clicked();
    void on_button_load_protocol_clicked();
    void on_button_set_saving_dir_clicked();
    void on_button_start_exp_clicked();
    void on_button_stop_exp_clicked();
    void on_button_reset_human_tracker_clicked();
    void on_button_reset_robot_tracker_clicked();
    void on_button_set_start_trial_clicked();
    void on_button_set_cond_clicked();
};

#endif // EXP_MAIN_WINDOW_H
