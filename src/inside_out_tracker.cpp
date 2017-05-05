#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include "tf/transform_datatypes.h"

#include "inside_out_tracker.h"

using std::sin;
using std::cos;

#define INCH2METER 0.0254
#define DEG2RAD M_PI / 180.0
//#define DEBUG_DRAW_MARKER

namespace inside_out_tracker {

    // ============================================================================
    // constructor
    InsideOutTracker::InsideOutTracker(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh) {
        std::string dict;
        std::string camera_info_file;
        std::string map_file, filter_param_file;

        // get parameters
        pnh.param<std::string>("dictionary", dict, "ARUCO");
        pnh.param<std::string>("camera_info_file", camera_info_file, "camera_calibration.yml");
        pnh.param<std::string>("map_file", map_file, "map.json");
        pnh.param<std::string>("filter_param_file", filter_param_file, "filter_param.json");

        pnh.param<std::string>("filter_mode", this->filter_mode, "low_pass");
        pnh.param<std::string>("odom_source", this->odom_source, "imu");
        pnh.param<bool>("use_accelerometer", this->m_flag_use_acc, false);
        pnh.param<bool>("draw_markers", this->m_flag_draw_markers, false);

        pnh.param<double>("marker_size", this->m_marker_size, 0.204);
        pnh.param<int>("num_sample_reset", this->m_num_sample_reset_max, 30);
        pnh.param<int>("num_frames_skip", this->m_num_frames_skip, 0);
        pnh.param<double>("velocity_filter_alpha", this->m_vel_filter_alpha, 0.3);
        pnh.param<double>("pose_change_threshold", this->m_pose_change_thresh, 0.3);

        // initialize aruco trackers
        this->m_detector.setThresholdParams(7, 7);
        this->m_detector.setThresholdParamRange(2, 0);
        this->m_detector.setDictionary(dict);

        // get camera info
        this->m_cam_param.readFromXMLFile(camera_info_file);

        // set camera to world rotation transformation
        this->m_rot_cam_to_world << 1.0, 0.0, 0.0,
                                    0.0, 0.0, 1.0,
                                    0.0, -1., 0.0;

        // load map from data file
        this->load_map(map_file);

        // load filter data
        this->load_filter_param(filter_param_file);

        // initialize publishers and subscribers
        this->m_pose_pub = this->nh_.advertise<geometry_msgs::Pose2D>("inside_out_tracker/pose2d", 1);
        this->m_camera_sub = this->nh_.subscribe<sensor_msgs::Image>("inside_out_tracker/image", 1,
                                                                     &InsideOutTracker::camera_rgb_callback, this);
        this->m_imu_sub = this->nh_.subscribe<std_msgs::Float32MultiArray>("inside_out_tracker/imu_data_raw", 1,
                                                                    &InsideOutTracker::imu_callback, this);
        this->m_odom_sub = this->nh_.subscribe<nav_msgs::Odometry>("inside_out_tracker/odom", 1,
                                                                   &InsideOutTracker::odom_callback, this);
        this->m_reset_sub = this->nh_.subscribe<std_msgs::Bool>("inside_out_tracker/reset", 1,
                                                                &InsideOutTracker::reset_callback, this);

        // time interval for discretization
        this->m_dt_process = 0.05;

        // initialize velocity and pose history
        this->m_body_pose_last = geometry_msgs::Pose2D();
        this->m_body_vel_last = geometry_msgs::Vector3();

        this->m_num_frames_skipped = 0;

        this->m_flag_reset_filter = true;
        this->m_mu.setZero(); this->m_mu[1] = 3;
        this->m_cov.setZero();
    }

    // ============================================================================
    // distructor
    InsideOutTracker::~InsideOutTracker() {
    }

    // ============================================================================
    void InsideOutTracker::load_map(const std::string file_name) {
        std::ifstream map_stream(file_name);
        json j;

        if (!map_stream.is_open()) {
            ROS_ERROR("cannot open file!!!!!");
        }

        map_stream >> j;

        if (j["type"] == "marker_map") {
            load_marker_map(j);
        }
        else {
            load_board_map(j);
        }
    }

    // ============================================================================
    void InsideOutTracker::load_marker_map(json &j) {
        int num_markers = j["num_markers"];

        this->map_markers.clear();
        for (int i = 0; i < num_markers; i++) {
            geometry_msgs::Pose2D pose;
            std::stringstream field_name;

            field_name << "marker" << i;
            pose.x = j[field_name.str()]["x"];
            pose.y = j[field_name.str()]["y"];
            pose.theta = j[field_name.str()]["theta"];

            // convert units
            if (j[field_name.str()]["units"][0] == "inch") {
                pose.x *= INCH2METER;
                pose.y *= INCH2METER;
            }
            if (j[field_name.str()]["units"][1] == "deg") {
                pose.theta *= DEG2RAD;
            }

            // insert to the marker map
            int marker_id = j[field_name.str()]["id"];
            this->map_markers.insert({marker_id, pose});
        }
    }

    // ============================================================================
    void InsideOutTracker::load_board_map(json &j) {
        int num_boards = j["num_boards"];

        this->map_markers.clear();
        for (int i = 0; i < num_boards; i++) {
            std::stringstream board_name_stream;
            board_name_stream << "board" << i;
            std::string board_name = board_name_stream.str();

            double board_x = j[board_name]["x"];
            double board_y = j[board_name]["y"];
            double board_th = j[board_name]["theta"];
            double grid_height = j[board_name]["grid_height"];
            double grid_width = j[board_name]["grid_width"];
            int h = j[board_name]["height"];
            int w = j[board_name]["width"];

            std::string board_type = j[board_name]["type"];

            for (int gy = 0; gy < h; gy++) {
                for (int gx = 0; gx < w; gx++) {
                    geometry_msgs::Pose2D pose;

                    if (board_type == "vertical") {
                        pose.x = board_x + (double)gx * grid_width * cos(board_th);
                        pose.y = board_y + (double)gx * grid_width * sin(board_th);
                    }
                    else {
                        double xp = (double)gx * grid_width;
                        double yp = -(double)gy * grid_height;
                        pose.x = board_x + xp * cos(board_th) - yp * sin(board_th);
                        pose.y = board_y + xp * sin(board_th) + yp * cos(board_th);
                    }
                    pose.theta = board_th;

                    // convert units if necessary
                    if (j[board_name]["units"][0] == "inch") {
                        pose.x *= INCH2METER;
                        pose.y *= INCH2METER;
                    }
                    if (j[board_name]["units"][1] == "deg") {
                        pose.theta *= DEG2RAD;
                    }

                    int marker_id = j[board_name]["marker_id_list"][gy * w + gx];
                    this->map_markers.insert({marker_id, pose});

                    if (board_type == "vertical") {
                        this->type_markers.insert({marker_id, marker_vertical});
                    }
                    else {
                        this->type_markers.insert({marker_id, marker_horizontal});
                    }
                }
            }
        }
    }

    // ============================================================================
    void InsideOutTracker::load_filter_param(const std::string file_name) {
        std::ifstream filter_param_stream(file_name);
        json j;

        filter_param_stream >> j;

        for (int xx = 0; xx < 3; xx++) {
            for (int yy = 0; yy < 3; yy++) {
                int zz = xx * 3 + yy;
                this->m_cov_acc(xx, yy) = j["acc_cov"][zz];
                this->m_cov_gyro(xx, yy) = j["gyro_cov"][zz];
                this->m_rot_imu_to_world(xx, yy) = j["imu_to_world_rot"][zz];
            }
        }

        // FIXME temporarily set mu, cov
        this->m_mu.setZero(); this->m_mu[1] = 3;
        this->m_cov.setIdentity();

        // rotate the covariance matrices
        this->m_cov_acc = this->m_rot_imu_to_world * this->m_cov_acc * this->m_rot_imu_to_world.transpose();
        this->m_cov_gyro = this->m_rot_imu_to_world * this->m_cov_gyro * this->m_rot_imu_to_world.transpose();

        // multiply by the maximum magnitude
        this->m_cov_acc *= 9.81 * 9.81;
        this->m_cov_gyro *= 2.0 * 2.0;
    }

    // ============================================================================
    void InsideOutTracker::reset_filter() {
        Eigen::Vector3d avg_pose;
        Eigen::Matrix3d avg_cov;
        // reset mu to mean of the measurements
        avg_pose.setZero();
        for (auto pose = this->m_pose_reset.begin(); pose != this->m_pose_reset.end(); pose++) {
            avg_pose += *pose;
        }
        avg_pose /= this->m_num_sample_reset_max;
        this->m_mu.setZero();
        this->m_mu.topLeftCorner(3, 1) = avg_pose;

        // reset cov to covarance of the measurements
        avg_cov.setZero();
        for (auto pose = this->m_pose_reset.begin(); pose != this->m_pose_reset.end(); pose++) {
            Eigen::Vector3d t_pose_diff = *pose - avg_pose;
            avg_cov += t_pose_diff * t_pose_diff.transpose();
        }
        avg_cov /= this->m_num_sample_reset_max;
        this->m_cov.setZero();
        this->m_cov.topLeftCorner(3, 3) = avg_cov;

        // FIXME: set the vision covariance to be this
        this->m_cov_vision = avg_cov;

        // set the reset flag to false
        this->m_flag_reset_filter = false;
    }

    // ============================================================================
    void InsideOutTracker::detect_markers() {
        // detect the markers
        this->m_markers = this->m_detector.detect(this->m_image_input, this->m_cam_param,
                                                  (float)this->m_marker_size, true);

        if (m_flag_draw_markers) {
            // draw marker boundaries
            cv::Mat t_output_image;
            this->m_image_input.copyTo(t_output_image);

            for (int i = 0; i < this->m_markers.size(); i++) {
                this->m_markers[i].draw(t_output_image, cv::Scalar(0, 0, 255), 1);
            }

            // display the image
            cv::imshow("test", t_output_image);
            cv::waitKey(3);
        }
    }

    // ============================================================================
    void InsideOutTracker::odom_process_update(const nav_msgs::OdometryConstPtr &odom_msg) {
        double v = odom_msg->twist.twist.linear.x;
        double om = odom_msg->twist.twist.angular.z;
        double x = this->m_mu[0];
        double y = this->m_mu[1];
        double th = this->m_mu[2];
        const double &dt = m_dt_process;

        // compute the Jacobians Gx, Gu
        Eigen::Matrix3d Gx;
        Eigen::Matrix<double, 3, 2> Gu;
        double x_new, y_new, th_new;

        if (std::abs(om) > 1e-5) {
            th_new = wrap_to_pi(th + om * dt);
            x_new = x + v / om * (sin(th_new) - sin(th));
            y_new = y - v / om * (cos(th_new) - cos(th));

            Gx << 1.0, 0.0, v / om * (cos(th_new) - cos(th)),
                    0.0, 1.0, v / om * (sin(th_new) - sin(th)),
                    0.0, 0.0, 1.0;

            Gu << 1.0 / om * (sin(th_new) - sin(th)), v / (om*om) * (sin(th) - sin(th_new) + cos(th_new) * om * dt),
                    -1.0 / om * (cos(th_new) - cos(th)), v / (om*om) * (-cos(th) + cos(th_new) + sin(th_new) * om * dt),
                    0.0, dt;
        }
        else {
            th_new = wrap_to_pi(th + om * dt);
            x_new = x + v * cos(th) * dt;
            y_new = y + v * sin(th) * dt;

            Gx << 1.0, 0.0, -v * sin(th) * dt,
                    0.0, 1.0, v * cos(th) * dt,
                    0.0, 0.0, 1.0;

            Gu << cos(th) * dt, -0.5 * v *sin(th) * dt * dt,
                    sin(th) * dt, 0.5 * v * cos(th) * dt * dt,
                    0.0, dt;
        }

        // update mean and covariance
        m_mu[0] = x_new;
        m_mu[1] = y_new;
        m_mu[2] = th_new;

        Eigen::Matrix2d cov_process;
        cov_process << odom_msg->twist.covariance[0], odom_msg->twist.covariance[5],
                odom_msg->twist.covariance[30], odom_msg->twist.covariance[35];

        m_cov.topLeftCorner(3, 3) = Gx * m_cov.topLeftCorner(3, 3) * Gx.transpose() +
                dt * Gu * cov_process * Gu.transpose();

        // publish the new predicted pose
        this->m_body_pose.x = this->m_mu[0];
        this->m_body_pose.y = this->m_mu[1];
        this->m_body_pose.theta = this->m_mu[2];
        this->m_pose_pub.publish(this->m_body_pose);
        std::cout << "odom update!"<< std::endl;
    }

    // ============================================================================
    void InsideOutTracker::imu_process_update(Eigen::Vector3d acc_meas, Eigen::Vector3d gyro_meas) {
        // FIXME: simplest case for now, assuming imu only moves in x-y plane
        // FIXME: also assume fixed update rate
        // first rotate the measurements
        acc_meas = this->m_rot_imu_to_world * acc_meas;
        gyro_meas = this->m_rot_imu_to_world * gyro_meas;

        const double &th = this->m_mu[2];
        const double &dt = this->m_dt_process;
        Eigen::Matrix2d R_th;
        R_th << cos(th), -sin(th), sin(th), cos(th);

        double acc_x = acc_meas[0] * cos(th) - acc_meas[1] * sin(th);
        double acc_y = acc_meas[0] * sin(th) + acc_meas[1] * cos(th);

        Vector5d mu_diff;
        Matrix5d Gt, Rt;
        if (this->m_flag_use_acc) {
            mu_diff << this->m_mu[3] * dt + 0.5 * acc_x * dt * dt,
                    this->m_mu[4] * dt + 0.5 * acc_y * dt * dt,
                    gyro_meas[2] * dt,
                    acc_x * dt,
                    acc_y * dt;
            Gt << 1.0, 0.0, - 0.5 * acc_y * dt * dt, dt, 0,
                    0.0, 1.0, 0.5 * acc_x * dt * dt, 0, dt,
                    0.0, 0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 1.0;
        } else {
            mu_diff << this->m_mu[3] * dt,
                    this->m_mu[4] * dt,
                    gyro_meas[2] * dt,
                    0.0,
                    0.0;
            Gt << 1.0, 0.0, 0.0, dt, 0,
                    0.0, 1.0, 0.0, 0, dt,
                    0.0, 0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 1.0;
        }

        Rt.setZero();
        Eigen::Matrix2d cov_acc_rotated = R_th * this->m_cov_acc.topLeftCorner(2, 2) * R_th.transpose();

        Rt.topLeftCorner(2, 2) = this->m_cov_acc.topLeftCorner(2, 2) * std::pow(dt, 4) / 4.0;
        Rt(2, 2) = this->m_cov_gyro(2, 2) * dt * dt;
        Rt.bottomRightCorner(2, 2) = this->m_cov_acc.topLeftCorner(2, 2) * dt * dt;

        this->m_mu += mu_diff;
        this->m_mu[2] = wrap_to_pi(this->m_mu[2]);
        this->m_cov = Gt * this->m_cov * Gt.transpose();
        this->m_cov += Rt;

        // publish the new predicted pose
        this->m_body_pose.x = this->m_mu[0];
        this->m_body_pose.y = this->m_mu[1];
        this->m_body_pose.theta = this->m_mu[2];
        this->m_pose_pub.publish(this->m_body_pose);
    }

    // ============================================================================
    void InsideOutTracker::get_pose_from_markers(std::vector<double> &th_meas,
                                                 std::vector<Eigen::Vector2d> &pos_meas) {
        // reset the vectors
        th_meas.clear();
        pos_meas.clear();

        // get position and orientation from the markers
        for (int i = 0; i < this->m_markers.size(); i++) {
            // do nothing if the detected marker is not in the map
            int id = this->m_markers[i].id;
            if (this->map_markers.count(id) <= 0) {
                continue;
            }

            Eigen::Vector3d angle_axis;
            angle_axis << this->m_markers[i].Rvec.at<float>(0, 0),
                    this->m_markers[i].Rvec.at<float>(1, 0),
                    this->m_markers[i].Rvec.at<float>(2, 0);
            Eigen::Matrix3d rot_cam;
            rot_cam = Eigen::AngleAxisd(angle_axis.norm(), angle_axis.normalized());
            rot_cam = this->m_rot_cam_to_world * rot_cam;

            double yaw;
            if (this->type_markers[id] == marker_vertical) {
                // obtain yaw angle of the camera based on marker pose
                yaw = this->map_markers[id].theta
                       - std::atan2(rot_cam(1, 0), rot_cam(0, 0));
            }
            else {
                // obtain yaw with x-axis
                yaw = this->map_markers[id].theta
                      - std::atan2(rot_cam(1, 0), rot_cam(0, 0));
            }

            // obtain position of the camera based on marker pose
            Eigen::Matrix2d t_rot_cam;
            t_rot_cam << std::cos(yaw), -std::sin(yaw),
                    std::sin(yaw), std::cos(yaw);
            Eigen::Vector2d t_pos_marker_world(this->map_markers[id].x, this->map_markers[id].y);
            Eigen::Vector2d t_pos_marker_cam(this->m_markers[i].Tvec.at<float>(0, 0),
                                             this->m_markers[i].Tvec.at<float>(2, 0));
            Eigen::Vector2d t_pos = t_pos_marker_world - t_rot_cam * t_pos_marker_cam;

            // check the difference to the current pose
//            double pose_diff = std::abs(m_mu[0] - t_pos[0]) + std::abs(m_mu[1] - t_pos[1]) +
//                    std::abs(wrap_to_pi(m_mu[2] - roll));

//            if (pose_diff < m_pose_change_thresh || m_flag_reset_filter) {
//                t_theta.push_back(wrap_to_pi(roll));
//                pos += t_pos;
//                std::cout << roll << "  ";
//            }
//            std::cout << id << ": " << t_pos[0] << ",  " << t_pos[1] << ",  " << yaw << std::endl;

            th_meas.push_back(wrap_to_pi(yaw + M_PI_2));
            pos_meas.push_back(t_pos);
        }
    }

    // ============================================================================
    // measurement update
    void InsideOutTracker::measurement_update() {
        // don't update if no markers detected
        const unsigned long n = this->m_markers.size();
        if (n == 0)
            return;

        Matrix5d cov_prev = this->m_cov;
        vector<Eigen::Vector2d> xy_meas;
        vector<double> th_meas;

        this->get_pose_from_markers(th_meas, xy_meas);
        if (th_meas.size() == 0) {
            return;
        }

        for (int i = 0; i < th_meas.size(); i++) {
            // obtain the predicted measurements
            Eigen::Vector2d xy_pred(this->m_mu[0],
                                    this->m_mu[1]);
            double th_pred = this->m_mu[2];

            // calculate measurement Jacobian
            Eigen::Matrix<double, 3, 5> Ht;
            Eigen::Matrix3d Qt;
            Ht << 1.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0, 0.0;
            Qt = this->m_cov_vision;

            // calculate Kalman gain
            Eigen::Matrix3d cov_meas;
            Eigen::Matrix<double, 5, 3> Kt;
            cov_meas = Ht * this->m_cov * Ht.transpose(); cov_meas += Qt;
            Kt = this->m_cov * Ht.transpose() * cov_meas.inverse();

//            std::cout << Kt << std::endl;

            // update mean and covariance
            Eigen::Vector3d meas_diff(xy_meas[i][0] - xy_pred[0],
                                      xy_meas[i][1] - xy_pred[1],
                                      wrap_to_pi(th_meas[i] - th_pred));
//            std::cout << mu_diff << std::endl;
            this->m_mu += Kt * meas_diff;
            this->m_mu[2] = wrap_to_pi(this->m_mu[2]);
            this->m_cov -= Kt * Ht * this->m_cov;
        }
//        std::cout << "measurement update!" << std::endl;
    }

    // ============================================================================
    void InsideOutTracker::simple_update() {
        // don't update if no markers detected
        if (this->m_markers.size() == 0)
            return;

        // extract marker poses
        Eigen::Vector2d pos(0.0, 0.0);
        vector<Eigen::Vector2d> xy_meas;
        vector<double> th_meas;

        this->get_pose_from_markers(th_meas, xy_meas);
        if (th_meas.size() == 0) {
            return;
        }

        // calculate average position and orientation
        double th, dth = 0.0;
        for (int i = 1; i < th_meas.size(); i++) {
            dth += wrap_to_pi(th_meas[i] - th_meas[0]);
            pos += xy_meas[i];
        }

        pos /= th_meas.size();
        th = wrap_to_pi(th_meas[0] + dth / th_meas.size());
        std::cout << pos[0] << ",  " << pos[1] << ",  " << th << std::endl;

        if (this->m_flag_reset_filter) {
            Eigen::Vector3d t_pose(pos[0], pos[1], th);
            this->m_pose_reset.push_back(t_pose);

            if (this->m_pose_reset.size() >= this->m_num_sample_reset_max) {
                this->reset_filter();
            }
        } else {
            // update body pose with low-pass filter
            double alpha = 0.6, beta = 0.6, gamma = 0.25;
            this->m_mu[0] = alpha * this->m_mu[0] + (1 - alpha) * pos[0];
            this->m_mu[1] = beta * this->m_mu[1] + (1 - beta) * pos[1];
            this->m_mu[2] = gamma * this->m_mu[2] + (1 - gamma) * th;
        }

        // publish the tracking data
        this->m_body_pose.x = this->m_mu[0];
        this->m_body_pose.y = this->m_mu[1];
        this->m_body_pose.theta = this->m_mu[2];

        this->m_pose_pub.publish(this->m_body_pose);
    }

    // ============================================================================
    void InsideOutTracker::camera_rgb_callback(const sensor_msgs::ImageConstPtr &image_msg) {
        if (m_num_frames_skipped < m_num_frames_skip) {
            // skip this frame
            m_num_frames_skipped ++;
            return;
        }

        m_num_frames_skipped = 0;
        this->m_image_input = cv_bridge::toCvCopy(image_msg, "bgr8")->image;

        // detect markers
        this->detect_markers();
//        ROS_ERROR("something");

        if (this->filter_mode == "low_pass" || this->m_flag_reset_filter) {
            // perform simple position update
            this->simple_update();
        } else if (this->filter_mode == "kalman") {
            // perform measurement update
            this->measurement_update();
        }
    }

    // ============================================================================
    void InsideOutTracker::imu_callback(const std_msgs::Float32MultiArrayConstPtr &imu_msg) {
        if (this->filter_mode == "kalman" && this->odom_source == "imu" && !this->m_flag_reset_filter) {
            Eigen::Vector3d acc_meas, gyro_meas;
            acc_meas << imu_msg->data[0], imu_msg->data[1], imu_msg->data[2];
            gyro_meas << imu_msg->data[3], imu_msg->data[4], imu_msg->data[5];
            acc_meas *= -9.81;
            gyro_meas *= DEG2RAD;
            this->imu_process_update(acc_meas, gyro_meas);
        }
    }

    // ============================================================================
    void InsideOutTracker::odom_callback(const nav_msgs::OdometryConstPtr &odom_msg) {
        if (this->filter_mode == "kalman" && this->odom_source == "odom" && !this->m_flag_reset_filter)
            this->odom_process_update(odom_msg);
    }

    // ============================================================================
    void InsideOutTracker::reset_callback(const std_msgs::BoolConstPtr &reset_msg) {
        if (reset_msg->data != 0) {
            this->m_pose_reset.clear();
            this->m_flag_reset_filter = true;
        }
    }

} // namespace
