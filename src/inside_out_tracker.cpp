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
        std::string map_file;

        // get parameters
        ros::param::param<std::string>("~dictionary", dict, "ARUCO");
        ros::param::param<std::string>("~camera_info_file", camera_info_file, "camera_calibration.yml");
        ros::param::param<std::string>("~map_file", map_file, "map.json");

        ros::param::param<std::string>("~filter_mode", this->filter_mode, "low_pass");

        ros::param::param<float>("~marker_size", this->m_marker_size, 0.19);
        ros::param::param<float>("~velocity_filter_alpha", this->m_vel_filter_alpha, 0.3);

        // initialize aruco trackers
        this->m_detector.setThresholdParams(7, 7);
        this->m_detector.setThresholdParamRange(2, 0);
        this->m_detector.setDictionary(dict);

        // get camera info
        this->m_cam_param.readFromXMLFile(camera_info_file);

        // load map from data file
        this->load_map(map_file);

        // initialize publishers and subscribers
        this->m_human_pose_pub = this->nh_.advertise<geometry_msgs::Pose2D>("tracking/human_pos2d", 1);
        this->m_camera_sub = this->nh_.subscribe<sensor_msgs::Image>("inside_out_tracker/image", 1,
                                                                     &InsideOutTracker::camera_rgb_callback, this);

        // time interval for discretization
        this->m_dt = 0.03333333333;

        // initialize velocity and pose history
        this->m_body_pose_last = geometry_msgs::Pose2D();
        this->m_body_vel_last = geometry_msgs::Vector3();
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
    void InsideOutTracker::detect_markers() {
        // detect the markers
        this->m_markers = this->m_detector.detect(this->m_image_input, this->m_cam_param,
                                                  this->m_marker_size, true);

#ifdef DEBUG_DRAW_MARKER
        // draw marker boundaries
        cv::Mat t_output_image;
        this->m_image_input.copyTo(t_output_image);

        for (int i = 0; i < this->m_markers.size(); i++) {
            this->m_markers[i].draw(t_output_image, cv::Scalar(0, 0, 255), 1);
        }

        // draw cubes on the marker
        if (this->m_cam_param.isValid() && this->m_marker_size > 0) {
            for (int i = 0; i < this->m_markers.size(); i++) {
                aruco::CvDrawingUtils::draw3dCube(t_output_image, this->m_markers[i], this->m_cam_param);
                aruco::CvDrawingUtils::draw3dAxis(t_output_image, this->m_markers[i], this->m_cam_param);
            }
        }

        // display the image
        cv::imshow("test", t_output_image);
        cv::waitKey(3);
#endif
    }

    // ============================================================================
    void InsideOutTracker::process_update() {
    }

    // ============================================================================
    // measurement update
    void InsideOutTracker::measurement_update() {
    }

    // ============================================================================
    void InsideOutTracker::simple_update() {
        // extract marker poses
        Eigen::Vector2f pos;
        std::vector<float> t_theta;

        for (int i = 0; i < this->m_markers.size(); i++) {
            Eigen::Vector3f angle_axis;
            angle_axis << this->m_markers[i].Rvec.at<float>(0, 0),
                          this->m_markers[i].Rvec.at<float>(1, 0),
                          this->m_markers[i].Rvec.at<float>(2, 0);
            Eigen::Matrix3f rot_cam;
            rot_cam = Eigen::AngleAxisf(angle_axis.norm(), angle_axis.normalized());

            // obtain roll angle of the marker
            double roll = std::atan2(rot_cam(1, 0), rot_cam(0, 0));
            t_theta.push_back((float)roll);

            pos += Eigen::Vector2f(this->m_markers[i].Tvec.at<float>(0, 0),
                                   this->m_markers[i].Tvec.at<float>(1, 0));

            std::cout << roll << " " << this->m_markers[i].Tvec.at<float>(0, 0)
                      << " " << this->m_markers[i].Tvec.at<float>(1, 0) << std::endl;
        }

        // calculate average position and orientation
        float th, dth = 0.0;
        for (int i = 1; i < t_theta.size(); i++) {
            dth += wrap_to_pi(t_theta[i] - t_theta[0]);
        }

        pos /= t_theta.size();
        th = wrap_to_pi(t_theta[0] + dth / t_theta.size());

        this->m_body_pose.x = pos[0];
        this->m_body_pose.y = pos[1];
        this->m_body_pose.theta = th;

        // publish the tracking data
        this->m_human_pose_pub.publish(this->m_body_pose);
    }

    // ============================================================================
    void InsideOutTracker::camera_rgb_callback(const sensor_msgs::ImageConstPtr &image_msg) {
        this->m_image_input = cv_bridge::toCvCopy(image_msg, "bgr8")->image;

        // detect markers
        this->detect_markers();

        if (this->filter_mode == "low_pass") {
            // perform simple position update
            this->simple_update();
        } else if (this->filter_mode == "kalman") {
            // perform measurement update
            this->measurement_update();
        }
    }

} // namespace
