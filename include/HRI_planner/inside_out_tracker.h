#include <unordered_map>
#include <vector>

#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/Odometry.h"

#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"

#include "Eigen/Dense"

#define _USE_MATH_DEFINES

namespace inside_out_tracker {

inline double wrap_to_pi(double angle) {
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    return angle;
}

// define 5x5 matrix and 5x1 vector
typedef Eigen::Matrix<double, 5, 5> Matrix5d;
typedef Eigen::Matrix<double, 5, 1> Vector5d;

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
    ros::Subscriber m_imu_sub;
    ros::Subscriber m_odom_sub;
    ros::Subscriber m_reset_sub;
    ros::Publisher m_pose_pub;

    // aruco marker detector
    aruco::MarkerDetector m_detector;
    std::vector<aruco::Marker> m_markers;

    // camera to world rotation transformation
    Eigen::Matrix3d m_rot_cam_to_world;
    Eigen::Matrix3d m_rot_imu_to_world;

    // marker map
    std::unordered_map<int, geometry_msgs::Pose2D> map_markers;

    // detected body pose and velocity
    geometry_msgs::Pose2D m_body_pose;
    geometry_msgs::Pose2D m_body_pose_last;
    geometry_msgs::Vector3 m_body_vel;
    geometry_msgs::Vector3 m_body_vel_last;

    // body pose and covariance for Kalman Filtering
    Vector5d m_mu;
    Matrix5d m_cov;

    // Kalman filter parameters
    Eigen::Matrix3d m_cov_acc;
    Eigen::Matrix3d m_cov_gyro;
    Eigen::Matrix3d m_cov_odom;
    Eigen::Matrix3d m_cov_vision;

    // low_pass filter velocity filter coefficient
    double m_vel_filter_alpha;

    // time interval for discretization
    double m_dt_process;

    // camera parameters
    aruco::CameraParameters m_cam_param;

    // marker size
    double m_marker_size;

    // image from camera
    cv::Mat m_image_input;

    // parameters for filter settings
    std::string filter_mode;
    std::string odom_source;

    // flag for resetting the filter
    bool m_flag_reset_filter;
    bool m_flag_use_acc;

    // poses for reset
    int m_num_sample_reset_max;
    std::vector<Eigen::Vector3d> m_pose_reset;

    // callback functions
    void camera_rgb_callback(const sensor_msgs::ImageConstPtr &image_msg);
    void imu_callback(const std_msgs::Float32MultiArrayConstPtr &imu_msg);
    void odom_callback(const nav_msgs::OdometryConstPtr &odom_msg);
    void reset_callback(const std_msgs::BoolConstPtr &reset_msg);

    // function to load parameters from .json file
    void load_map(const std::string file_name);
    void load_filter_param(const std::string file_name);

    // update functions
    void detect_markers();
    void odom_process_update();
    void imu_process_update(Eigen::Vector3d acc_meas, Eigen::Vector3d gyro_meas);
    void measurement_update();
    void simple_update();

    void reset_filter();
};

} // namespace