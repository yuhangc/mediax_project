#include <string>
#include <iostream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>

#include "human_localization.h"

#define PI 3.141592653589793
#define DRAW_MARKER

// ============================================================================
// constructor
InsideOutTracker::InsideOutTracker()
{
    std::string dict;
    std::string camera_info_file;
    // get parameters
    ros::param::param<std::string>("~dictionary", dict, "ARUCO");
    ros::param::param<std::string>("~camera_info_file", camera_info_file, "camera_calibration.yml");

    ros::param::param<float>("~marker_size", this->m_marker_size, 0.19);
    ros::param::param<float>("~velocity_filter_alpha", this->m_vel_filter_alpha, 0.3);

    // field of view and focal length
    ros::param::param<float>("~field_of_view", this->m_fov, 57.0 * PI / 180.0);

    this->m_width = 640;
    this->m_height = 480;
    this->m_fw = (float) this->m_width / 2.0 / std::tan(this->m_fov / 2.0);
    this->m_fh = (float) this->m_height / 2.0 / std::tan(this->m_fov / 2.0);

    // initialize aruco trackers
    this->m_detector.setThresholdParams(7, 7);
    this->m_detector.setThresholdParamRange(2, 0);
    this->m_detector.setDictionary(dict);

    // get camera info
    this->m_cam_param.readFromXMLFile(camera_info_file);

    // initialize publishers and subscribers
    this->m_human_pose_pub = this->nh.advertise<geometry_msgs::Pose2D>("tracking/human_pos2d", 1);
    this->m_human_vel_pub = this->nh.advertise<geometry_msgs::Vector3>("tracking/human_vel2d", 1);
    this->m_camera_rgb_sub = this->nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 1,
                                                                    &InsideOutTracker::camera_rgb_callback, this);

    // time interval for discretization
    this->m_dt = 0.03333333333;

    // initialize velocity and pose history
    this->m_body_pose_last = geometry_msgs::Pose2D();
    this->m_body_vel_last = geometry_msgs::Vector3();
}

// ============================================================================
// distructor
InsideOutTracker::~InsideOutTracker()
{
}

// ============================================================================
// main update
void InsideOutTracker::update()
{
    // detect the markers
    this->m_markers = this->m_detector.detect(this->m_image_input, this->m_cam_param, this->m_marker_size);

#ifdef DRAW_MARKER
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

    // clear the marker rotation and position vector
    this->m_marker_pos.clear();
    this->m_marker_rot.clear();

    // convert axis-angle representation to rotation matrix
    for (int i = 0; i < this->m_markers.size(); i++) {
        Eigen::Matrix3d t_rot;
        Eigen::Vector3d t_pos;

        Eigen::Vector3d t_angle_axis;
        t_angle_axis << this->m_markers[i].Rvec.at<float>(0, 0),
                        this->m_markers[i].Rvec.at<float>(1, 0),
                        this->m_markers[i].Rvec.at<float>(2, 0);
        t_rot = Eigen::AngleAxisd(t_angle_axis.norm(), t_angle_axis.normalized());

        t_pos << this->m_markers[i].Tvec.at<float>(0, 0),
                 this->m_markers[i].Tvec.at<float>(1, 0),
                 this->m_markers[i].Tvec.at<float>(2, 0);

        this->m_marker_rot.push_back(t_rot);
        this->m_marker_pos.push_back(t_pos);
    }

    // set state to find
    this->m_tracking_status.data = "Find";

    // publish the tracking data
    this->m_human_pose_pub.publish(this->m_body_pose);
    this->m_human_vel_pub.publish(this->m_body_vel);
}

// ============================================================================
void InsideOutTracker::camera_rgb_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
    this->m_image_input = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
    this->m_flag_rgb_image_received = true;
}

// ============================================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "human_localization");
    InsideOutTracker human_tracker;

    ros::Rate loop_rate(10);
    for (int i = 0; i < 50; i++) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    while (!ros::isShuttingDown())
    {
        ros::spinOnce();
        human_tracker.update();
        loop_rate.sleep();
    }
}
