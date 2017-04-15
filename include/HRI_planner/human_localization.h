#include <fstream>

#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"

#include "Eigen/Dense"

#define NUM_MARKER 3

class InsideOutTracker
{
public:
    // constructor
    InsideOutTracker();

    // destructor
    ~InsideOutTracker();

    // main update function
    void update();

private:
    // node handler
    ros::NodeHandle nh;

    // subscriber and publisher
    ros::Subscriber m_camera_rgb_sub;
    ros::Publisher m_human_pose_pub;
    ros::Publisher m_human_vel_pub;
    ros::Publisher m_tracking_status_pub;

    // Aruco marker detector
    aruco::MarkerDetector m_detector;
    std::vector<aruco::Marker> m_markers;

    // orientation and position of the markers
    std::vector<Eigen::Matrix3d> m_marker_rot;
    std::vector<Eigen::Vector3d> m_marker_pos;

    // detected body pose and velocity
    geometry_msgs::Pose2D m_body_pose;
    geometry_msgs::Pose2D m_body_pose_last;
    geometry_msgs::Vector3 m_body_vel;
    geometry_msgs::Vector3 m_body_vel_last;

    // velocity filter coefficient
    float m_vel_filter_alpha;

    // time interval for discretization
    double m_dt;

    // tracking status
    std_msgs::String m_tracking_status;

    // camera parameters
    aruco::CameraParameters m_cam_param;

    // marker size
    float m_marker_size;

    // camera infos
    int m_width;
    int m_height;
    float m_fov;
    float m_fw;
    float m_fh;

    // image from camera
    cv::Mat m_image_input;

    // flags
    bool m_flag_rgb_image_received;

    // callback functions
    void camera_rgb_callback(const sensor_msgs::ImageConstPtr& image_msg);
};
