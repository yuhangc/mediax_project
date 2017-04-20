#include <unordered_map>
#include <vector>

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

#define _USE_MATH_DEFINES

namespace inside_out_tracker {

inline float wrap_to_pi(float angle) {
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    return angle;
}

class InsideOutTracker {
public:
    // constructor
    InsideOutTracker(ros::NodeHandle &nh, ros::NodeHandle &pnh);

    // destructor
    ~InsideOutTracker();

private:
    // node handler
    ros::NodeHandle nh_;

    // subscriber and publisher
    ros::Subscriber m_camera_sub;
    ros::Publisher m_human_pose_pub;

    // aruco marker detector
    aruco::MarkerDetector m_detector;
    std::vector<aruco::Marker> m_markers;

    // camera to world rotation transformation
    Eigen::Matrix3f m_rot_cam_to_world;

    // orientation and position of the markers
    std::vector<Eigen::Matrix3d> m_marker_rot;
    std::vector<Eigen::Vector3d> m_marker_pos;

    // marker map
    std::unordered_map<int, geometry_msgs::Pose2D> map_markers;

    // detected body pose and velocity
    geometry_msgs::Pose2D m_body_pose;
    geometry_msgs::Pose2D m_body_pose_last;
    geometry_msgs::Vector3 m_body_vel;
    geometry_msgs::Vector3 m_body_vel_last;

    // velocity filter coefficient
    float m_vel_filter_alpha;

    // time interval for discretization
    double m_dt;

    // camera parameters
    aruco::CameraParameters m_cam_param;

    // marker size
    float m_marker_size;

    // image from camera
    cv::Mat m_image_input;

    // filter mode
    std::string filter_mode;

    // callback functions
    void camera_rgb_callback(const sensor_msgs::ImageConstPtr &image_msg);

    // function to load map from .json file
    void load_map(const std::string file_name);

    // update functions
    void detect_markers();
    void process_update();
    void measurement_update();
    void simple_update();
};

} // namespace