#ifndef EXP_MAIN_WINDOW_H
#define EXP_MAIN_WINDOW_H

#include <QMainWindow>
#include <QTimer>

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
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber human_pose2d_sub_;

    // variables
    geometry_msgs::Pose2D human_pose_;
    geometry_msgs::Pose2D robot_pose_;
    geometry_msgs::Twist robot_vel_curr_;
    geometry_msgs::Twist robot_vel_cmd_;

    // callback functions
    void human_pose_callback(const geometry_msgs::Pose2D::ConstPtr& pose2d_msg);

signals:
    void human_pose_received(double x, double y, double th);
    void robot_pose_received(double x, double y, double th);

private slots:
    void update_gui_info();
};

#endif // EXP_MAIN_WINDOW_H
