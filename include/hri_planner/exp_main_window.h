#ifndef EXP_MAIN_WINDOW_H
#define EXP_MAIN_WINDOW_H

#include <QMainWindow>
#include <QTimer>

#include <fstream>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Char.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"

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
    ros::Subscriber sys_msg_sub_;

    // publishers
    ros::Publisher set_robot_state_pub_;
    ros::Publisher haptic_control_pub_;

    // variables to record
    int robot_state_;
    geometry_msgs::Pose2D human_pose_;
    geometry_msgs::Pose2D robot_pose_;
    geometry_msgs::Twist robot_vel_curr_;
    geometry_msgs::Twist robot_vel_cmd_;

    // experiment control variables
    int trial_num_;
    int cond_num_;

    // for publishing
    std_msgs::String set_robot_state_;
    std_msgs::String haptic_msg_;

    // for data saving
    std::string data_saving_path_;
    std::ofstream data_file_;

    bool flag_start_data_saving_;
    double t_start_data_saving_;

    // callback functions
    void human_pose_callback(const geometry_msgs::Pose2D::ConstPtr& pose2d_msg);
    void robot_pose_callback(const geometry_msgs::Pose2D::ConstPtr& pose2d_msg);

    void robot_state_callback(const std_msgs::String::ConstPtr& state_msg);
    void robot_odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);

    void sys_msg_callback(const std_msgs::String::ConstPtr& sys_msg);

signals:
    void human_pose_received(double x, double y, double th);
    void robot_pose_received(double x, double y, double th);

private slots:
    void update_gui_info();
    void on_combo_set_state_currentTextChanged(const QString &arg1);
    void on_combo_haptic_type_currentTextChanged(const QString &arg1);
    void on_button_set_state_clicked();
    void on_button_send_haptic_clicked();
};

#endif // EXP_MAIN_WINDOW_H
