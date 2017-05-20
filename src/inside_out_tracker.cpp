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
        pnh.param<std::string>("odom_source", this->odom_source, "odometer");
        pnh.param<std::string>("meas_source", this->meas_source, "aruco");
        pnh.param<bool>("use_accelerometer", this->m_flag_use_acc, false);
        pnh.param<bool>("draw_markers", this->m_flag_draw_markers, false);

        pnh.param<double>("marker_size", this->m_marker_size, 0.204);
        pnh.param<int>("num_sample_reset", this->m_num_sample_reset_max, 30);
        pnh.param<int>("num_frames_skip", this->m_num_frames_skip, 0);
        pnh.param<double>("filter_alpha_high", this->m_filter_alpha_high, 0.9);
        pnh.param<double>("filter_alpha_low", this->m_filter_alpha_low, 0.5);
        pnh.param<double>("pose_change_threshold", this->m_pose_change_thresh, 0.3);

        pnh.param<double>("image_scale", this->m_image_scale, 0.5);

        pnh.param<int>("num_sample_calibration", this->m_num_sample_calibration, 100);

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
        this->m_odom_sub = this->nh_.subscribe<nav_msgs::Odometry>("inside_out_tracker/odom", 1,
                                                                   &InsideOutTracker::odom_callback, this);
        this->m_opt_flow_sub = this->nh_.subscribe<std_msgs::Float32MultiArray>("opt_flow", 1,
                                                                                &InsideOutTracker::opt_flow_callback,
                                                                                this);
        this->m_reset_sub = this->nh_.subscribe<std_msgs::Bool>("inside_out_tracker/reset", 1,
                                                                &InsideOutTracker::reset_callback, this);

        // time interval for discretization
        double process_rate;
        pnh.param<double>("process_rate", process_rate, 20);
        this->m_dt_process = 1.0 / process_rate;

        this->m_num_frames_skipped = 0;

        this->m_flag_reset_filter = true;
        this->m_flag_run_calibration_aruco = false;
        this->m_mu.setZero(); this->m_mu[1] = 3;
        this->m_cov.setZero();

        // initialize velocities
        this->m_vel_x = 0.0;
        this->m_vel_y = 0.0;
        this->m_vel_linear = 0.0;
        this->m_vel_angular = 0.0;
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
        this->m_num_boards = num_boards;

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

            // convert units if necessary
            if (j[board_name]["units"][0] == "inch") {
                board_x *= INCH2METER;
                board_y *= INCH2METER;
                grid_width *= INCH2METER;
                grid_height *= INCH2METER;
            }
            if (j[board_name]["units"][1] == "deg") {
                board_th *= DEG2RAD;
            }

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

                    int marker_id = j[board_name]["marker_id_list"][gy * w + gx];
                    this->map_markers.insert({marker_id, pose});
                    this->board_id_markers.insert({marker_id, i});

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

        std::string meas_cov = this->meas_source + "_cov";
        std::string odom_cov = this->odom_source + "_cov";

        // load measurement covariance in row major order
        for (int xx = 0; xx < 3; xx++) {
            for (int yy = 0; yy < 3; yy++) {
                int zz = xx * 3 + yy;
                this->m_cov_meas(xx, yy) = j[meas_cov][zz];
            }
        }

        // load process covariance
        int x_max, y_max;
        if (odom_source == "odometer") {
            x_max = 2;
            y_max = 2;
        }
        else {
            x_max = 3;
            y_max = 3;
        }

        // in row major order
        this->m_cov_proc.resize(x_max, y_max);
        for (int xx = 0; xx < x_max; xx++) {
            for (int yy = 0; yy < y_max; yy++) {
                int zz = xx * y_max + yy;
                this->m_cov_proc(xx, yy) = j[odom_cov][zz];
            }
        }

        // load the measurement multipliers
        std::string meas_mul = this->meas_source + "_multiplier";
        for (int i = 0; i < 10; i++) {
            this->m_cov_meas_mul.push_back(j[meas_mul][i]);
        }
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
        this->m_cov = avg_cov;

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
    void InsideOutTracker::odom_process_update(const double v, const double om) {
        double x = this->m_mu[0];
        double y = this->m_mu[1];
        double th = this->m_mu[2];
        const double dt = m_dt_process;

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

            Gu << cos(th) * dt, -0.5 * v * sin(th) * dt * dt,
                    sin(th) * dt, 0.5 * v * cos(th) * dt * dt,
                    0.0, dt;
        }

        // update mean and covariance
        m_mu[0] = x_new;
        m_mu[1] = y_new;
        m_mu[2] = th_new;

        m_cov = Gx * m_cov * Gx.transpose() + dt * Gu * m_cov_proc * Gu.transpose();

        // publish the new predicted pose
        this->m_body_pose.x = this->m_mu[0];
        this->m_body_pose.y = this->m_mu[1];
        this->m_body_pose.theta = this->m_mu[2];
        this->m_pose_pub.publish(this->m_body_pose);

//        std::cout << m_cov << std::endl;
    }

    // ============================================================================
    void InsideOutTracker::opt_flow_process_update(const double vx, const double vy,
                                                   const double om, const double qual) {
        // don't update if quality is too low
        if (qual < 100) {
            ROS_WARN("Optical flow quality too low!");
            // increase covariance
            m_cov *= 1.1;
        } else {
            double x = this->m_mu[0];
            double y = this->m_mu[1];
            double th = this->m_mu[2];
            const double dt = m_dt_process;

            // compute the Jacobians Gx, Gu
            Eigen::Matrix3d Gx;
            Eigen::Matrix3d Gu;
            double x_new, y_new, th_new;

            th_new = wrap_to_pi(th + om * dt);
            x_new = x + dt * (vx * cos(th) - vy * sin(th));
            y_new = y + dt * (vx * sin(th) + vy * cos(th));

            Gx.setIdentity();

            Gu << dt * cos(th), -dt * sin(th), -dt * dt * (vx * sin(th) + vy * cos(th)),
                    dt * sin(th), dt * cos(th), dt * dt * (vx * cos(th) - vy * sin(th)),
                    0.0, 0.0, dt;

            // update mean and covariance
            m_mu[0] = x_new;
            m_mu[1] = y_new;
            m_mu[2] = th_new;

            m_cov = Gx * m_cov * Gx.transpose() + Gu * m_cov_proc * Gu.transpose();
        }

        // publish the new predicted pose
        this->m_body_pose.x = this->m_mu[0];
        this->m_body_pose.y = this->m_mu[1];
        this->m_body_pose.theta = this->m_mu[2];
        this->m_pose_pub.publish(this->m_body_pose);
    }

    // ============================================================================
    void InsideOutTracker::get_pose_from_markers(std::vector<double> &th_meas,
                                                 std::vector<Eigen::Vector2d> &pos_meas,
                                                 std::vector<double> &dist_meas) {
        // reset the vectors
        th_meas.clear();
        pos_meas.clear();
        dist_meas.clear();

        // average the orientation of each board
        Eigen::ArrayXd yaw_board(m_num_boards);
        Eigen::ArrayXd samples_board(m_num_boards);
        yaw_board.setZero();
        samples_board.setZero();

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
            Eigen::Matrix3d rot_marker;
            rot_marker = Eigen::AngleAxisd(angle_axis.norm(), angle_axis.normalized());
            rot_marker = this->m_rot_cam_to_world * rot_marker;

            yaw_board[board_id_markers[id]] += std::atan2(rot_marker(1, 0), rot_marker(0, 0));
            samples_board[board_id_markers[id]] += 1;
        }
        yaw_board /= samples_board;

        // get position and orientation from the markers
        for (int i = 0; i < this->m_markers.size(); i++) {
            // do nothing if the detected marker is not in the map
            int id = this->m_markers[i].id;
            if (this->map_markers.count(id) <= 0) {
                continue;
            }

//            Eigen::Vector3d angle_axis;
//            angle_axis << this->m_markers[i].Rvec.at<float>(0, 0),
//                    this->m_markers[i].Rvec.at<float>(1, 0),
//                    this->m_markers[i].Rvec.at<float>(2, 0);
//            Eigen::Matrix3d rot_marker;
//            rot_marker = Eigen::AngleAxisd(angle_axis.norm(), angle_axis.normalized());
//            rot_marker = this->m_rot_cam_to_world * rot_marker;
//
//            double yaw = this->map_markers[id].theta
//                         - std::atan2(rot_marker(1, 0), rot_marker(0, 0));
            double yaw = this->map_markers[id].theta - yaw_board[board_id_markers[id]];

            // obtain position of the camera based on marker pose
            Eigen::Matrix2d t_rot_cam;
            t_rot_cam << std::cos(yaw), -std::sin(yaw),
                    std::sin(yaw), std::cos(yaw);
            Eigen::Vector2d t_pos_marker_world(this->map_markers[id].x, this->map_markers[id].y);
            Eigen::Vector2d t_pos_marker_cam(this->m_markers[i].Tvec.at<float>(0, 0),
                                             this->m_markers[i].Tvec.at<float>(2, 0));

            double dist = t_pos_marker_cam.norm();
            Eigen::Vector2d t_pos_marker = t_rot_cam * t_pos_marker_cam;
            Eigen::Vector2d t_pos = t_pos_marker_world - t_rot_cam * t_pos_marker_cam;
//            std::cout << id << ": " << yaw_board[board_id_markers[id]] << ", ("
//                      << this->m_markers[i].Tvec.at<float>(0, 0) << ", "
//                      << this->m_markers[i].Tvec.at<float>(2, 0) << "), ("
//                      << this->map_markers[id].x << ", "
//                      << this->map_markers[id].y << "), ("
//                      << t_pos_marker[0] << ", "
//                      << t_pos_marker[1] << ")" << std::endl;
            std::cout << id << ": " << t_pos[0] << ",  " << t_pos[1] << ",  " << yaw << std::endl;

            th_meas.push_back(wrap_to_pi(yaw + M_PI_2));
            pos_meas.push_back(t_pos);
            dist_meas.push_back(dist);
        }
    }

    // ============================================================================
    double InsideOutTracker::get_measurement_cov_multiplier(double dist, double v, double om) {
        // the larger the distance, the larger the multiplier
        int id = (int) dist;
        if (id > 9) {
            id = 9;
        }

        return m_cov_meas_mul[id];
    }

    // ============================================================================
    // measurement update
    void InsideOutTracker::measurement_update() {
        // don't updat if angular velocity is larger than 0.3
        if (std::abs(this->m_vel_angular) > 0.2 && this->m_qual > 128) {
            return;
        }

        // don't update if no markers detected
        const unsigned long n = this->m_markers.size();
        if (n == 0)
            return;

        vector<Eigen::Vector2d> xy_meas;
        vector<double> th_meas;
        vector<double> dist_meas;

        this->get_pose_from_markers(th_meas, xy_meas, dist_meas);
        if (th_meas.size() == 0) {
            return;
        }

        for (int i = 0; i < th_meas.size(); i++) {
            // obtain the predicted measurements
            Eigen::Vector2d xy_pred(this->m_mu[0],
                                    this->m_mu[1]);
            double th_pred = this->m_mu[2];

            // calculate measurement Jacobian
            Eigen::Matrix3d Ht;
            Eigen::Matrix3d Qt;
            Ht << 1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0;
            Qt = this->get_measurement_cov_multiplier(dist_meas[i],
                                                      m_vel_linear, m_vel_angular) * this->m_cov_meas;

            // calculate Kalman gain
            Eigen::Matrix3d cov_meas;
            Eigen::Matrix3d Kt;
            cov_meas = Ht * this->m_cov * Ht.transpose(); cov_meas += Qt;
            Kt = this->m_cov * Ht.transpose() * cov_meas.inverse();

            if (i == 0) {
//                std::cout << "Measured pose:" << xy_meas[i][0] << ", " << xy_meas[i][1]
//                          << ", " << th_meas[i] << std::endl;
//                std::cout << "Kalman Gain:" << std::endl;
//                std::cout << Kt << std::endl;
            }

            // update mean and covariance
            Eigen::Vector3d meas_diff(xy_meas[i][0] - xy_pred[0],
                                      xy_meas[i][1] - xy_pred[1],
                                      wrap_to_pi(th_meas[i] - th_pred));

            this->m_mu += Kt * meas_diff;
            this->m_mu[2] = wrap_to_pi(this->m_mu[2]);
            this->m_cov -= Kt * Ht * this->m_cov;
        }
    }

    // ============================================================================
    void InsideOutTracker::simple_update() {
        // don't update if no markers detected
        if (this->m_markers.size() == 0)
            return;

        // extract marker poses
        Eigen::Vector2d pos;
        vector<Eigen::Vector2d> xy_meas;
        vector<double> th_meas;
        vector<double> dist_meas;

        this->get_pose_from_markers(th_meas, xy_meas, dist_meas);
        if (th_meas.size() == 0) {
            return;
        }

        // calculate average position and orientation
        double th, dth = 0.0;
        pos = xy_meas[0];
        for (int i = 1; i < th_meas.size(); i++) {
            dth += wrap_to_pi(th_meas[i] - th_meas[0]);
            pos += xy_meas[i];
        }

        pos /= th_meas.size();
        th = wrap_to_pi(th_meas[0] + dth / th_meas.size());
//        std::cout << pos[0] << ",  " << pos[1] << ",  " << th << std::endl;

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
    void InsideOutTracker::calibrate_cov_aruco() {
        // get poses from detection
        Eigen::Vector2d pos;
        vector<Eigen::Vector2d> xy_meas;
        vector<double> th_meas;
        vector<double> dist_meas;

        this->get_pose_from_markers(th_meas, xy_meas, dist_meas);

        for (int i = 0; i < th_meas.size(); i++) {
            Eigen::Vector3d pose(xy_meas[i][0], xy_meas[i][1], th_meas[i]);
            this->m_pose_calibration.push_back(pose);
        }

        // run calibration if enough samples collected
        int num_sample = (int) this->m_pose_calibration.size();
        std::cout << "Number of samples collected: " << num_sample << std::endl;
        if (num_sample >= this->m_num_sample_calibration) {
            Eigen::Vector3d avg_pose;
            Eigen::Matrix3d avg_cov;

            // calculate mean pose
            avg_pose.setZero();
            for (auto pose = this->m_pose_calibration.begin(); pose != this->m_pose_calibration.end(); pose++) {
                avg_pose += *pose;
            }
            avg_pose /= num_sample;

            // calculate covariance of the measurements
            avg_cov.setZero();
            for (auto pose = this->m_pose_calibration.begin(); pose != this->m_pose_calibration.end(); pose++) {
                Eigen::Vector3d t_pose_diff = *pose - avg_pose;
                avg_cov += t_pose_diff * t_pose_diff.transpose();
            }
            avg_cov /= num_sample;

            std::cout << "======================= Calibration Result =========================" << std::endl;
            std::cout << "Average pose is:" << std::endl;
            std::cout << avg_pose << std::endl;
            std::cout << std::endl;
            std::cout << "Covariance is:" << std::endl;
            std::cout << avg_cov << std::endl;
            std::cout << "====================================================================" << std::endl;

            // set the reset flag to false
            this->m_flag_run_calibration_aruco = false;

            // end node?
            ros::shutdown();
        }
    }

    // ============================================================================
    void InsideOutTracker::run_calibration_aruco() {
        this->m_flag_run_calibration_aruco = true;
    }

    // ============================================================================
    void InsideOutTracker::camera_rgb_callback(const sensor_msgs::ImageConstPtr &image_msg) {
        if (m_num_frames_skipped < m_num_frames_skip) {
            // skip this frame
            m_num_frames_skipped ++;
            return;
        }

        m_num_frames_skipped = 0;
        cv::resize(cv_bridge::toCvShare(image_msg, "mono8")->image, this->m_image_input,
                    cv::Size(), this->m_image_scale, this->m_image_scale);
//        this->m_image_input = cv_bridge::toCvCopy(image_msg, "mono8")->image;

        // detect markers
        this->detect_markers();

        // run calibration if requested
        if (this->m_flag_run_calibration_aruco) {
            this->calibrate_cov_aruco();
            return;
        }

        if (this->filter_mode == "low_pass" || this->m_flag_reset_filter) {
            // perform simple position update
            this->simple_update();
        } else if (this->filter_mode == "kalman") {
            // perform measurement update
            this->measurement_update();
        }
    }

    // ============================================================================
    void InsideOutTracker::opt_flow_callback(const std_msgs::Float32MultiArrayConstPtr &opt_flow_msg) {
        if (this->filter_mode == "kalman" && (!this->m_flag_reset_filter)) {
            if (this->odom_source == "optical_flow") {
                // low-pass filter the velocity
                double alpha = m_filter_alpha_low;
                m_qual = opt_flow_msg->data[3];
                if (m_qual < 100) {
                    alpha = m_filter_alpha_high;
                }

                m_vel_x = alpha * m_vel_x + (1 - alpha) * (-opt_flow_msg->data[0]);
                m_vel_y = alpha * m_vel_y + (1 - alpha) * opt_flow_msg->data[1];
                m_vel_angular = alpha * m_vel_angular + (1 - alpha) * (-opt_flow_msg->data[2]);

                this->opt_flow_process_update(m_vel_x, m_vel_y, m_vel_angular, m_qual);
            }
        }
    }

    // ============================================================================
    void InsideOutTracker::odom_callback(const nav_msgs::OdometryConstPtr &odom_msg) {
        if (this->filter_mode == "kalman" && (!this->m_flag_reset_filter)) {
            std::cout << "odom update" << std::endl;

            if (this->odom_source == "odometer") {
                this->m_vel_linear = odom_msg->twist.twist.linear.x;
                this->m_vel_angular = odom_msg->twist.twist.angular.z;
                this->odom_process_update(odom_msg->twist.twist.linear.x,
                                          odom_msg->twist.twist.angular.z);
            }
        }
    }

    // ============================================================================
    void InsideOutTracker::reset_callback(const std_msgs::BoolConstPtr &reset_msg) {
        if (reset_msg->data != 0) {
            this->m_pose_reset.clear();
            this->m_flag_reset_filter = true;
        }
    }

} // namespace
