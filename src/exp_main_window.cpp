#include "exp_main_window.h"
#include "ui_exp_main_window.h"

//===========================================================================
ExpMainWindow::ExpMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ExpMainWindow)
{
    ui->setupUi(this);
}

//===========================================================================
ExpMainWindow::~ExpMainWindow()
{
    delete ui;
}

//===========================================================================
void ExpMainWindow::Init()
{
    // initialize subscribers
    human_pose2d_sub_ = nh_.subscribe<geometry_msgs::Pose2D>("/tracking/human_pose2d", 1,
                                                             &ExpMainWindow::human_pose_callback, this);
    robot_pose2d_sub_ = nh_.subscribe<geometry_msgs::Pose2D>("/tracking/robot_pose2d", 1,
                                                             &ExpMainWindow::robot_pose_callback, this);

    robot_state_sub_ = nh_.subscribe<std_msgs::String>("/robot/status", 1,
                                                       &ExpMainWindow::robot_state_callback, this);
    robot_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/robot/odom", 1,
                                                        &ExpMainWindow::robot_odom_callback, this);

    sys_msg_sub_ = nh_.subscribe<std_msgs::String>("/sys_message", 1,
                                                   &ExpMainWindow::sys_msg_callback, this);

    // initialize publishers

    // get parameters

    // initialize variables
    human_pose_ = geometry_msgs::Pose2D();
    robot_pose_ = geometry_msgs::Pose2D();

    // connect signals and slots
    connect(this, SIGNAL(human_pose_received(double,double,double)),
            ui->gl_tracking_display, SLOT(set_human_pose(double,double,double)));
    connect(this, SIGNAL(robot_pose_received(double,double,double)),
            ui->gl_tracking_display, SLOT(set_robot_pose(double,double,double)));

    connect(&update_timer_, SIGNAL(timeout()), this, SLOT(update_gui_info()));
    connect(&update_timer_, SIGNAL(timeout()), ui->gl_tracking_display, SLOT(update()));
    update_timer_.start(30);
}

//===========================================================================
void ExpMainWindow::update_gui_info()
{
    // spin ros
    ros::spinOnce();

    // check for shutdown
    if (ros::isShuttingDown()) {
        this->close();
    }
}

//===========================================================================
void ExpMainWindow::human_pose_callback(const geometry_msgs::Pose2D::ConstPtr &pose2d_msg)
{
    // record new pose
    human_pose_.x = pose2d_msg->x;
    human_pose_.y = pose2d_msg->y;
    human_pose_.theta = pose2d_msg->theta;

    // display new pose
    ui->lcd_human_pose_x->display(human_pose_.x);
    ui->lcd_human_pose_y->display(human_pose_.y);
    ui->lcd_human_pose_theta->display(human_pose_.theta);

    // send out the signal
    emit human_pose_received(human_pose_.x, human_pose_.y, human_pose_.theta);
}

//===========================================================================
void ExpMainWindow::robot_pose_callback(const geometry_msgs::Pose2D::ConstPtr &pose2d_msg)
{
    // record new pose
    robot_pose_.x = pose2d_msg->x;
    robot_pose_.y = pose2d_msg->y;
    robot_pose_.theta = pose2d_msg->theta;

    // display new pose
    ui->lcd_robot_pose_x->display(robot_pose_.x);
    ui->lcd_robot_pose_y->display(robot_pose_.y);
    ui->lcd_robot_pose_theta->display(robot_pose_.theta);

    // send out the signal
    emit robot_pose_received(robot_pose_.x, robot_pose_.y, robot_pose_.theta);
}

//===========================================================================
void ExpMainWindow::robot_state_callback(const std_msgs::String::ConstPtr &state_msg)
{
    if (state_msg->data == "Idle") {
        robot_state_ = 0;
        ui->label_robot_status->setText("Idle");
        ui->label_robot_status->setStyleSheet("QLabel { background-color : yellow; color : black; }");
    }
    else if (state_msg->data == "RandMove") {
        robot_state_ = 5;
        ui->label_robot_status->setText("Rand Move");
        ui->label_robot_status->setStyleSheet("QLabel { background-color : blue; color : black; }");
    }
    else if (state_msg->data == "Teleop") {
        robot_state_ = 6;
        ui->label_robot_status->setText("Teleoperation");
        ui->label_robot_status->setStyleSheet("QLabel { background-color : white; color : black; }");
    }
}

//===========================================================================
void ExpMainWindow::robot_odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    // record robot velocity
    robot_vel_curr_.linear.x = odom_msg->twist.twist.linear.x;
    robot_vel_curr_.angular.z = odom_msg->twist.twist.angular.y;

    // display the velocity
    ui->lcd_vel_current_lin->display(robot_vel_curr_.linear.x);
    ui->lcd_vel_current_ang->display(robot_vel_curr_.angular.z);
}

//===========================================================================
void ExpMainWindow::sys_msg_callback(const std_msgs::String::ConstPtr &sys_msg)
{
    ui->browser_sys_message->append(QString::fromStdString(sys_msg->data));
}
