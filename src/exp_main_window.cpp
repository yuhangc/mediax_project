#include <QFileDialog>

#include <random>

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

    robot_state_sub_ = nh_.subscribe<std_msgs::String>("/robot_state", 1,
                                                       &ExpMainWindow::robot_state_callback, this);
    robot_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/robot_odom", 1,
                                                        &ExpMainWindow::robot_odom_callback, this);

    sys_msg_sub_ = nh_.subscribe<std_msgs::String>("/sys_message", 1,
                                                   &ExpMainWindow::sys_msg_callback, this);

    // initialize publishers
    set_robot_state_pub_ = nh_.advertise<std_msgs::String>("/set_robot_state", 1);
    haptic_control_pub_ = nh_.advertise<std_msgs::String>("/haptic_control", 1);
    reset_human_tracker_pub_ = nh_.advertise<std_msgs::Bool>("/tracking/reset_human_tracker", 1);
    reset_robot_tracker_pub_ = nh_.advertise<std_msgs::Bool>("/tracking/reset_robot_tracker", 1);

    // get parameters
    ros::param::param<std::string>("~dir_loading_pre_set", dir_loading_pre_set_, "/home");
    ros::param::param<std::string>("~dir_saving_pre_set", dir_saving_pre_set_, "/home");

    ros::param::param<bool>("~use_odom_as_pose", flag_use_odom_as_pose_, true);

    // initialize variables
    human_pose_ = geometry_msgs::Pose2D();
    robot_pose_ = geometry_msgs::Pose2D();

    exp_state_ = exp_state_idle;

    trial_num_ = 0;
    cond_num_ = 0;
    num_cond_total_ = 0;

    exp_num_ = 1;
    config_num_ = 0;

    // initialize flags
    flag_start_data_saving_ = false;

    flag_dir_saving_set_ = false;
    flag_protocol_loaded_ = false;

    flag_start_exp_requested_ = false;
    flag_stop_exp_requested_ = false;

    flag_exp_training_ = true;

    // connect signals and slots
    connect(this, SIGNAL(human_pose_received(double,double,double)),
            ui->gl_tracking_display, SLOT(set_human_pose(double,double,double)));
    connect(this, SIGNAL(robot_pose_received(double,double,double)),
            ui->gl_tracking_display, SLOT(set_robot_pose(double,double,double)));

    connect(&update_timer_, SIGNAL(timeout()), this, SLOT(exp_state_update()));
    connect(&update_timer_, SIGNAL(timeout()), ui->gl_tracking_display, SLOT(update()));
    update_timer_.start(30);
}

//===========================================================================
void ExpMainWindow::load_protocol(std::string file_name)
{
    std::ifstream protocol(file_name);
    if (!protocol.is_open()) {
        ui->browser_sys_message->append("Cannot open protocol file! Please try again.");
        return;
    }

    json proto_parser;
    protocol >> proto_parser;

    if (proto_parser["experiment_name"] == "exp1") {
        exp_num_ = 1;
        load_protocol_exp1(proto_parser);
    }
    else if (proto_parser["experiment_name"] == "exp2") {
        exp_num_ = 2;
        load_protocol_exp2(proto_parser);
    }

    flag_protocol_loaded_ = true;
    flag_exp_training_ = true;

    cond_num_ = 0;
    trial_num_ = 0;
    set_cond_trial_text();
}

//===========================================================================
void ExpMainWindow::load_protocol_exp1(json &proto_parser)
{
    config_num_ = proto_parser["configuration"];

    // get the target positions
    y_targets_.clear();
    y_targets_.push_back(proto_parser["human_target0"]["y"]);
    y_targets_.push_back(proto_parser["human_target1"]["y"]);

    // load the trial conditions
    num_cond_total_ = proto_parser["num_conditions"];

    num_trial_total_.clear();
    num_training_total_.clear();
    for (int i = 0; i < num_cond_total_; i++) {
        num_trial_total_.push_back(proto_parser["num_trials"][i]);
        num_training_total_.push_back(proto_parser["num_trainings"][i]);
    }

    robot_action_list_.clear();
    for (int i = 0; i < num_cond_total_; i++) {
        std::vector<int> action_list;
        int trials_total = num_training_total_[i] + num_trial_total_[i];
        for (int j = 0; j < trials_total; j++) {
            std::stringstream field_name;
            field_name << "condition_" << i;

            int act = proto_parser["robot_actions"][field_name.str()][j];
            action_list.push_back(proto_parser["robot_actions"][field_name.str()][j]);
        }
        robot_action_list_.push_back(action_list);
        ui->browser_sys_message->append("end");
    }

}

//===========================================================================
void ExpMainWindow::load_protocol_exp2(json &proto_parser)
{
    // TODO: to be implemented
}

//===========================================================================
void ExpMainWindow::start_data_saving()
{
    std::stringstream file_name;
    if (flag_exp_training_) {
        file_name << dir_saving_ << "/training_cond" << cond_num_
                  << "_trial" << trial_num_ << ".txt";
    }
    else {
        file_name << dir_saving_ << "/cond" << cond_num_
                  << "_trial" << trial_num_ << ".txt";
    }

    data_stream_.open(file_name.str());
    ui->browser_sys_message->append("Created file " + QString::fromStdString(file_name.str()));
}

//===========================================================================
void ExpMainWindow::stop_data_saving()
{
    data_stream_.close();
}

//===========================================================================
void ExpMainWindow::save_exp_data()
{
    // may save different information based on which experiment is running
    switch (exp_num_) {
    case 1:
        data_stream_ << ros::Time().now() << ", "
                     << human_pose_.x << ", " << human_pose_.y << ", " << human_pose_.theta << ", "
                     << robot_pose_.x << ", " << robot_pose_.y << ", " << robot_pose_.theta << ", "
                     << robot_vel_curr_.linear.x << ", " << robot_vel_curr_.angular.z
                     << std::endl;
        break;
    case 2:
        break;
    default:
        break;
    }
}

//===========================================================================
void ExpMainWindow::exp_state_update()
{
    // spin ros
    ros::spinOnce();

    // main exp state machine
    switch (exp_num_) {
    case 1:
        if (config_num_ == 0) {
            state_machine_exp1_config0();
        }
        else {
            state_machine_exp1_config1();
        }
        break;
    case 2:
        state_machine_exp2();
        break;
    default:
        ui->browser_sys_message->append("Unknown experiment configuration!");
    }

    // check for shutdown
    if (ros::isShuttingDown()) {
        this->close();
    }
}

//===========================================================================
void ExpMainWindow::state_machine_exp1_config0()
{
    // obtain time
    double t_exp = ros::Time::now().toSec() - t_exp_start_;

    switch (exp_state_) {
    case exp_state_idle:
        // check for condition and trial number
        if (cond_num_ >= num_cond_total_ || trial_num_ >= num_trial_total_[cond_num_]) {
            return;
        }

        // check for start experiment flag
        if (flag_start_exp_requested_) {
            set_robot_action_delay();
            send_haptic_cue();
            start_data_saving();

            exp_state_ = exp_state_experimenting;
            ui->browser_sys_message->append("Started experiment!");
        }
        break;
    case exp_state_experimenting:
        save_exp_data();

        // check for send robot action
        if (!flag_robot_action_sent_ && t_exp >= t_robot_action_delay_) {
            send_robot_action();
            flag_robot_action_sent_ = true;
        }

        // check for stop experiment flag
        if (flag_stop_exp_requested_) {
            stop_data_saving();

            // update trial number and condition number
            trial_num_ ++;
            if (flag_exp_training_) {
                if (trial_num_ >= num_training_total_[cond_num_]) {
                    trial_num_ = 0;
                    flag_exp_training_ = false;
                    ui->browser_sys_message->append("Training ended!");
                }
            }
            else {
                if (trial_num_ >= num_trial_total_[cond_num_]) {
                    cond_num_ ++;
                    trial_num_ = 0;
                    flag_exp_training_ = true;
                    ui->browser_sys_message->append("Condition ended!");
                }
            }

            set_cond_trial_text();

            exp_state_ = exp_state_idle;
            ui->browser_sys_message->append("Stopped experiment!");
        }
        break;
    default:
        ui->browser_sys_message->append("Unknown experiment state!");
        break;
    }

    // reset some of the flags to prevent unwanted state switching
    flag_start_exp_requested_ = false;
    flag_stop_exp_requested_ = false;
}

//===========================================================================
void ExpMainWindow::state_machine_exp1_config1()
{
    int target = trial_num_ & 0x01;
    bool target_reached = ((target == 0) && (human_pose_.y > y_targets_[1])) ||
            ((target == 1) && (human_pose_.y < y_targets_[0]));

    switch (exp_state_) {
    case exp_state_idle:
        // check for condition and trial number
        if (cond_num_ >= num_cond_total_ || trial_num_ >= num_trial_total_[cond_num_]) {
            return;
        }

        // check for start experiment flag
        if (flag_start_exp_requested_) {
            // set robot to teleoperation
            set_robot_state_.data = "Teleop";
            set_robot_state_pub_.publish(set_robot_state_);

            exp_state_ = exp_state_pre_experiment;
            ui->browser_sys_message->append("Started experiment!");
        }
        break;
    case exp_state_pre_experiment:
        // check the position of the human
        if (((target == 0) && (human_pose_.y > y_targets_[0])) ||
                ((target == 1) && (human_pose_.y < y_targets_[1]))) {
            // start data saving
            start_data_saving();

            // send haptic feedback
            send_haptic_cue();

            // start the trial
            exp_state_ = exp_state_experimenting;
            ui->browser_sys_message->append("Started trial!");
        }
        break;
    case exp_state_experimenting:
        save_exp_data();

        // check for goal reached or stop experiment
        if (target_reached || flag_stop_exp_requested_) {
            stop_data_saving();

            // update trial number and condition number
            trial_num_ ++;
            if (flag_exp_training_) {
                if (trial_num_ >= num_training_total_[cond_num_]) {
                    trial_num_ = 0;
                    flag_exp_training_ = false;
                    ui->browser_sys_message->append("Training ended!");
                }
            }
            else {
                if (trial_num_ >= num_trial_total_[cond_num_]) {
                    // reset trial number
                    cond_num_ ++;
                    trial_num_ = 0;
                    flag_exp_training_ = true;

                    exp_state_ = exp_state_idle;
                    ui->browser_sys_message->append("Condition ended!");
                    break;
                }
            }

            set_cond_trial_text();

            exp_state_ = exp_state_pre_experiment;
            ui->browser_sys_message->append("Trial ended!");
        }
        break;
    default:
        ui->browser_sys_message->append("Unknown experiment state!");
        break;
    }

    // reset some of the flags to prevent unwanted state switching
    flag_start_exp_requested_ = false;
    flag_stop_exp_requested_ = false;
}

//===========================================================================
void ExpMainWindow::state_machine_exp2()
{
    // to be implemented
}

//===========================================================================
void ExpMainWindow::set_robot_action_delay()
{
    int action_id = trial_num_;
    if (!flag_exp_training_) {
        action_id += num_training_total_[cond_num_];
    }

    if (robot_action_list_[cond_num_][action_id] == 1) {
        // delay randomly between 1 ~ 2 seconds
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(1, 2);

        t_robot_action_delay_ = dis(gen);
        t_exp_start_ = ros::Time::now().toSec();
        flag_robot_action_sent_ = false;
    }
    else if (robot_action_list_[cond_num_][action_id] == 0) {
        // set to idle
        set_robot_state_.data = "Idle";
        set_robot_state_pub_.publish(set_robot_state_);
    }
}

//===========================================================================
void ExpMainWindow::send_robot_action()
{
    int action_id = trial_num_;
    if (!flag_exp_training_) {
        action_id += num_training_total_[cond_num_];
    }

    if (robot_action_list_[cond_num_][action_id] == 1) {
        // set to random move
        set_robot_state_.data = "RandMove";
        set_robot_state_pub_.publish(set_robot_state_);
    }
    else if (robot_action_list_[cond_num_][action_id] == 0) {
        // set to idle
        set_robot_state_.data = "Idle";
        set_robot_state_pub_.publish(set_robot_state_);
    }
}

//===========================================================================
void ExpMainWindow::send_haptic_cue()
{
    int action_id = trial_num_;
    if (!flag_exp_training_) {
        action_id += num_training_total_[cond_num_];
    }

    if (cond_num_ == 2) {
        if (robot_action_list_[cond_num_][action_id] < 2) {
            haptic_msg_.data = "Attract";
            haptic_control_pub_.publish(haptic_msg_);
        }
        else if (robot_action_list_[cond_num_][action_id] < 4) {
            haptic_msg_.data = "Repel";
            haptic_control_pub_.publish(haptic_msg_);
        }
    }

}

//===========================================================================
void ExpMainWindow::set_cond_trial_text()
{
    QString text = QString::number(trial_num_);
    if (flag_exp_training_) {
        text = "Training " + text;
    }

    ui->label_trial_num->setText(text);
    ui->label_trial_cond->setText(ui->combo_cond_set->itemText(cond_num_));

    // set instructions
    int action_id = trial_num_;
    if (!flag_exp_training_) {
        action_id += num_training_total_[cond_num_];
    }

    switch (robot_action_list_[cond_num_][action_id]) {
    case 0:
        ui->label_instruction->setText("Avoid human: wait-move-stop");
        break;
    case 1:
        ui->label_instruction->setText("Avoid human: move-stop");
        break;
    case 2:
        ui->label_instruction->setText("Don't avoid: wait-move");
        break;
    case 3:
        ui->label_instruction->setText("Don't avoid: move-stop-move");
        break;
    }
}

//===========================================================================
void ExpMainWindow::update_gui_info()
{
    // update gui info
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
    robot_vel_curr_.angular.z = odom_msg->twist.twist.angular.z;

    // optional: record odom pose as the robot pose
    if (flag_use_odom_as_pose_) {
        robot_pose_.x = odom_msg->pose.pose.position.x;
        robot_pose_.y = odom_msg->pose.pose.position.y;
        robot_pose_.theta = get_yaw_quat(odom_msg->pose.pose.orientation);

        // send out the signal
        emit robot_pose_received(robot_pose_.x, robot_pose_.y, robot_pose_.theta);

        // display new pose
        ui->lcd_robot_pose_x->display(robot_pose_.x);
        ui->lcd_robot_pose_y->display(robot_pose_.y);
        ui->lcd_robot_pose_theta->display(robot_pose_.theta);
    }

    // display the velocity
    ui->lcd_vel_current_lin->display(robot_vel_curr_.linear.x);
    ui->lcd_vel_current_ang->display(robot_vel_curr_.angular.z);
}

//===========================================================================
void ExpMainWindow::sys_msg_callback(const std_msgs::String::ConstPtr &sys_msg)
{
    ui->browser_sys_message->append(QString::fromStdString(sys_msg->data));
}

//===========================================================================
void ExpMainWindow::on_combo_set_state_currentTextChanged(const QString &arg1)
{
    set_robot_state_.data = arg1.toStdString();
}

//===========================================================================
void ExpMainWindow::on_combo_haptic_type_currentTextChanged(const QString &arg1)
{
    haptic_msg_.data = arg1.toStdString();
}

//===========================================================================
void ExpMainWindow::on_button_set_state_clicked()
{
    set_robot_state_pub_.publish(set_robot_state_);
}

//===========================================================================
void ExpMainWindow::on_button_send_haptic_clicked()
{
    haptic_control_pub_.publish(haptic_msg_);
}

//===========================================================================
void ExpMainWindow::on_button_load_protocol_clicked()
{
    QString file_name = QFileDialog::getOpenFileName(this, tr("Open Protocal File"),
                                                     dir_loading_pre_set_.c_str(), tr("JSON files (*.json);;Text files (*.txt)"));
    load_protocol(file_name.toStdString());
    ui->browser_sys_message->append("Loaded protocal: " + file_name);
}

//===========================================================================
void ExpMainWindow::on_button_set_saving_dir_clicked()
{
    QString dir = QFileDialog::getExistingDirectory(this, tr("Set Data Saving Directory"), dir_saving_pre_set_.c_str());
    dir_saving_ = dir.toStdString();
    ui->browser_sys_message->append("Data Saving Directory: " + dir);

    flag_dir_saving_set_ = true;
}

//===========================================================================
void ExpMainWindow::on_button_start_exp_clicked()
{
    if ((!flag_dir_saving_set_) || (!flag_protocol_loaded_)) {
        ui->browser_sys_message->append("Protocol directory or saving directory not set!");
        return;
    }

    flag_start_exp_requested_ = true;
}

//===========================================================================
void ExpMainWindow::on_button_stop_exp_clicked()
{
    flag_stop_exp_requested_ = true;
}

//===========================================================================
void ExpMainWindow::on_button_reset_human_tracker_clicked()
{
    std_msgs::Bool reset_msg;
    reset_msg.data = true;
    reset_human_tracker_pub_.publish(reset_msg);
}

//===========================================================================
void ExpMainWindow::on_button_reset_robot_tracker_clicked()
{
    std_msgs::Bool reset_msg;
    reset_msg.data = true;
    reset_human_tracker_pub_.publish(reset_msg);
}

//===========================================================================
void ExpMainWindow::on_button_set_start_trial_clicked()
{
    trial_num_ = ui->spinBox_trial_num_set->value();
    if (trial_num_ >= num_training_total_[cond_num_]) {
        trial_num_ -= num_training_total_[cond_num_];
        flag_exp_training_ = false;
        ui->browser_sys_message->append("Set to trial number " + QString::number(trial_num_));
    } else {
        flag_exp_training_ = true;
        ui->browser_sys_message->append("Set to training number " + QString::number(trial_num_));
    }

    set_cond_trial_text();
    exp_state_ = exp_state_idle;
}

//===========================================================================
void ExpMainWindow::on_button_set_cond_clicked()
{
    cond_num_ = ui->combo_cond_set->currentIndex();
    ui->browser_sys_message->append("Set to condition: " + ui->combo_cond_set->currentText());
    ui->browser_sys_message->append("Please also set the trial number!");

    exp_state_ = exp_state_idle;
}
