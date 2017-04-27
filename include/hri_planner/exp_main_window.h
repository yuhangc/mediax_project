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
    QTimer m_update_timer;

    // node handler
    ros::NodeHandle nh;

    // subscribers
    ros::Subscriber robot_state_sub;
    ros::Subscriber robot_odom_sub;
    ros::Subscriber cmd_vel_sub;
};

#endif // EXP_MAIN_WINDOW_H
