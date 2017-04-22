#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include "tf/transform_datatypes.h"

#include "json.hpp"
#include "inside_out_tracker.h"

using json = nlohmann::json;

#define INCH2METER 0.0254
#define DEG2RAD M_PI / 180.0
#define DEBUG_DRAW_MARKER

namespace inside_out_tracker {

    // ============================================================================
    // constructor
    InsideOutTracker::InsideOutTracker(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh) {
        std::string dict;
        std::string camera_info_file;
        std::string map_file, filter_param_file;

        // get parameters
        ros::param::param<std::string>("~dictionary", dict, "ARUCO");
        ros::param::param<std::string>("~camera_info_file", camera_info_file, "camera_calibration.yml");
        ros::param::param<std::string>("~map_file", map_file, "map.json");
        ros::param::param<std::string>("~filter_param_file", filter_param_file, "filter_param.json");

        ros::param::param<std::string>("~filter_mode", this->filter_mode, "low_pass");
        ros::param::param<std::string>("~odom_source", this->odom_source, "imu");
        ros::param::param<bool>("~use_accelerometer", this->m_flag_use_acc, false);

        ros::param::param<double>("~marker_size", this->m_marker_size, 0.204);
        ros::param::param<int>("~num_sample_reset", this->m_num_sample_reset_max, 30);
        ros::param::param<double>("~velocity_filter_alpha", this->m_vel_filter_alpha, 0.3);

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
        this->m_pose_pub = this->nh_.advertise<geometry_msgs::Pose2D>("tracking/pose2d", 1);
        this->m_camera_sub = this->nh_.subscribe<sensor_msgs::Image>("inside_out_tracker/image", 1,
                                                                     &InsideOutTracker::camera_rgb_callback, this);
        this->m_imu_sub = this->nh_.subscribe<std_msgs::Float32MultiArray>("tracking/imu_data_raw", 1,
                                                                    &InsideOutTracker::imu_callback, this);
        this->m_odom_sub = this->nh_.subscribe<nav_msgs::Odometry>("tracking/odom", 1,
                                                                   &InsideOutTracker::odom_callback, this);
        this->m_reset_sub = this->nh_.subscribe<std_msgs::Bool>("tracking/reset", 1,
                                                                &InsideOutTracker::reset_callback, this);

        // time interval for discretization
        this->m_dt_process = 0.02;

        // initialize velocity and pose history
        this->m_body_pose_last = geometry_msgs::Pose2D();
        this->m_body_vel_last = geometry_msgs::Vector3();

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

        map_stream >> j;

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
        std::cout << this->m_cov_vision << std::endl;

        // set the reset flag to false
        this->m_flag_reset_filter = false;
    }

    // ============================================================================
    void InsideOutTracker::detect_markers() {
        // detect the markers
        this->m_markers = this->m_detector.detect(this->m_image_input, this->m_cam_param,
                                                  (float)this->m_marker_size, true);

#ifdef DEBUG_DRAW_MARKER
        // draw marker boundaries
        cv::Mat t_output_image;
        this->m_image_input.copyTo(t_output_image);

        for (int i = 0; i < this->m_markers.size(); i++) {
            this->m_markers[i].draw(t_output_image, cv::Scalar(0, 0, 255), 1);
        }
//
//        // draw cubes on the marker
//        if (this->m_cam_param.isValid() && this->m_marker_size > 0) {
//            for (int i = 0; i < this->m_markers.size(); i++) {
//                aruco::CvDrawingUtils::draw3dCube(t_output_image, this->m_markers[i], this->m_cam_param, true);
//                aruco::CvDrawingUtils::draw3dAxis(t_output_image, this->m_markers[i], this->m_cam_param);
//            }
//        }

        // display the image
        cv::imshow("test", t_output_image);
        cv::waitKey(3);
#endif
    }

    // ============================================================================
    void InsideOutTracker::odom_process_update() {
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
        R_th << std::cos(th), -std::sin(th), std::sin(th), std::cos(th);

        double acc_x = acc_meas[0] * std::cos(th) - acc_meas[1] * std::sin(th);
        double acc_y = acc_meas[0] * std::sin(th) + acc_meas[1] * std::cos(th);

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

//        Rt.topLeftCorner(2, 2) = cov_acc_rotated * std::pow(dt, 4) / 4.0;
        Rt.topLeftCorner(2, 2) = this->m_cov_acc.topLeftCorner(2, 2) * std::pow(dt, 4) / 4.0;
        Rt(2, 2) = this->m_cov_gyro(2, 2) * dt * dt;
//        Rt.bottomRightCorner(2, 2) = cov_acc_rotated * dt * dt;
        Rt.bottomRightCorner(2, 2) = this->m_cov_acc.topLeftCorner(2, 2) * dt * dt;

        this->m_mu += mu_diff;
        this->m_mu[2] = wrap_to_pi(this->m_mu[2]);
        this->m_cov = Gt * this->m_cov * Gt.transpose();
        this->m_cov += Rt;

        std::cout << this->m_mu.transpose() << std::endl;

        // publish the new predicted pose
        this->m_body_pose.x = this->m_mu[0];
        this->m_body_pose.y = this->m_mu[1];
        this->m_body_pose.theta = this->m_mu[2];
        this->m_pose_pub.publish(this->m_body_pose);
    }

    // ============================================================================
    // measurement update
    void InsideOutTracker::measurement_update() {
        // don't update if no markers detected
        const unsigned long n = this->m_markers.size();
        if (n == 0)
            return;

        Matrix5d cov_prev = this->m_cov;
        for (int i = 0; i < n; i++) {
            // do nothing if the detected marker is not in the map
            int id = this->m_markers[i].id;
            if (this->map_markers.count(this->m_markers[i].id) == 0)
                continue;

            // extract the measured orientation and position
            Eigen::Vector3d angle_axis;
            Eigen::Vector2d xy_meas;
            double th_meas;
            angle_axis << this->m_markers[i].Rvec.at<float>(0, 0),
                    this->m_markers[i].Rvec.at<float>(1, 0),
                    this->m_markers[i].Rvec.at<float>(2, 0);
            Eigen::Matrix3d rot_marker;
            rot_marker = Eigen::AngleAxisd(angle_axis.norm(), angle_axis.normalized());
            rot_marker = this->m_rot_cam_to_world * rot_marker;

            th_meas = this->map_markers[id].theta
                      - std::atan2(rot_marker(1, 0), rot_marker(0, 0));

            // obtain position of the camera based on marker pose
            Eigen::Matrix2d t_rot_cam;
            t_rot_cam << std::cos(th_meas), -std::sin(th_meas),
                    std::sin(th_meas), std::cos(th_meas);
            Eigen::Vector2d t_pos_marker_world(this->map_markers[id].x, this->map_markers[id].y);
            Eigen::Vector2d t_pos_marker_cam(this->m_markers[i].Tvec.at<float>(0, 0),
                                             this->m_markers[i].Tvec.at<float>(2, 0));

            xy_meas = t_pos_marker_world - t_rot_cam * t_pos_marker_cam;

            // obtain the predicted measurements
            Eigen::Vector2d xy_pred(this->m_mu[0],
                                    this->m_mu[1]);
            double th_pred = this->m_mu[2];

//            std::cout << xy_pred[0] << "  " << xy_pred[1] << "  " << th_pred << std::endl;

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
            cov_meas = Ht * cov_prev * Ht.transpose(); cov_meas += Qt;
            Kt = cov_prev * Ht.transpose() * cov_meas.inverse();

//            std::cout << Kt << std::endl;

            // update mean and covariance
            Eigen::Vector3d meas_diff(xy_meas[0] - xy_pred[0],
                                    xy_meas[1] - xy_pred[1],
                                    wrap_to_pi(th_meas - th_pred));
//            std::cout << mu_diff << std::endl;
            this->m_mu += Kt * meas_diff;
            this->m_mu[2] = wrap_to_pi(this->m_mu[2]);
            this->m_cov -= Kt * Ht * cov_prev;
        }
    }

    // ============================================================================
    void InsideOutTracker::simple_update() {
        // don't update if no markers detected
        if (this->m_markers.size() == 0)
            return;

        // extract marker poses
        Eigen::Vector2d pos(0.0, 0.0);
        std::vector<double> t_theta;

        for (int i = 0; i < this->m_markers.size(); i++) {
            // do nothing if the detected marker is not in the map
            int id = this->m_markers[i].id;
            if (this->map_markers.count(this->m_markers[i].id) == 0)
                continue;

            Eigen::Vector3d angle_axis;
            angle_axis << this->m_markers[i].Rvec.at<float>(0, 0),
                          this->m_markers[i].Rvec.at<float>(1, 0),
                          this->m_markers[i].Rvec.at<float>(2, 0);
            Eigen::Matrix3d rot_cam;
            rot_cam = Eigen::AngleAxisd(angle_axis.norm(), angle_axis.normalized());
            rot_cam = this->m_rot_cam_to_world * rot_cam;

            // obtain roll angle of the camera based on marker pose
            double roll = this->map_markers[id].theta
                          - std::atan2(rot_cam(1, 0), rot_cam(0, 0));
            t_theta.push_back(wrap_to_pi(roll));

            // obtain position of the camera based on marker pose
            Eigen::Matrix2d t_rot_cam;
            t_rot_cam << std::cos(roll), -std::sin(roll),
                         std::sin(roll), std::cos(roll);
            Eigen::Vector2d t_pos_marker_world(this->map_markers[id].x, this->map_markers[id].y);
            Eigen::Vector2d t_pos_marker_cam(this->m_markers[i].Tvec.at<float>(0, 0),
                                      this->m_markers[i].Tvec.at<float>(2, 0));
            pos += t_pos_marker_world - t_rot_cam * t_pos_marker_cam;
        }

        // calculate average position and orientation
        double th, dth = 0.0;
        for (int i = 1; i < t_theta.size(); i++) {
            dth += wrap_to_pi(t_theta[i] - t_theta[0]);
        }

        pos /= t_theta.size();
        th = wrap_to_pi(t_theta[0] + dth / t_theta.size());

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
//        std::cout << this->m_body_pose;
    }

    // ============================================================================
    void InsideOutTracker::camera_rgb_callback(const sensor_msgs::ImageConstPtr &image_msg) {
        this->m_image_input = cv_bridge::toCvCopy(image_msg, "bgr8")->image;

        // detect markers
        this->detect_markers();

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
            this->odom_process_update();
    }

    // ============================================================================
    void InsideOutTracker::reset_callback(const std_msgs::BoolConstPtr &reset_msg) {
        if (reset_msg->data != 0) {
            this->m_pose_reset.clear();
            this->m_flag_reset_filter = true;
        }
    }

} // namespace
