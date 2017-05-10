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

#include "json.hpp"
using json = nlohmann::json;

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

// define marker types
typedef enum {
    marker_vertical,
    marker_horizontal
} MarkerType;

class InsideOutTracker {
public:
    // constructor
    InsideOutTracker(ros::NodeHandle &nh, ros::NodeHandle &pnh);

    // destructor
    ~InsideOutTracker();

    // calibrate the covariance of the aruco detection
    void run_calibration_aruco();

private:
    // node handler
    ros::NodeHandle nh_;

    // subscriber and publisher
    ros::Subscriber m_camera_sub;
    ros::Subscriber m_odom_sub;
    ros::Subscriber m_reset_sub;
    ros::Publisher m_pose_pub;

    // aruco marker detector
    aruco::MarkerDetector m_detector;
    std::vector<aruco::Marker> m_markers;

    // camera to world rotation transformation
    Eigen::Matrix3d m_rot_cam_to_world;

    // marker map
    std::unordered_map<int, geometry_msgs::Pose2D> map_markers;
    std::unordered_map<int, MarkerType> type_markers;

    // detected body pose and velocity
    geometry_msgs::Pose2D m_body_pose;

    // body pose and covariance for Kalman Filtering
    Eigen::Vector3d m_mu;
    Eigen::Matrix3d m_cov;
    double m_vel_linear;
    double m_vel_angular;

    // Kalman filter parameters
    Eigen::Matrix3d m_cov_meas;
    Eigen::MatrixXd m_cov_proc;

    // pose changing threshold
    double m_pose_change_thresh;

    // low_pass filter velocity filter coefficient
    double m_vel_filter_alpha;

    // time interval for discretization
    double m_dt_process;

    // skipping frames
    int m_num_frames_skip;
    int m_num_frames_skipped;

    // camera parameters
    aruco::CameraParameters m_cam_param;

    // marker size
    double m_marker_size;

    // image from camera
    cv::Mat m_image_input;

    // scale for the image
    double m_image_scale;

    // parameters for filter settings
    std::string filter_mode;
    std::string odom_source;
    std::string meas_source;

    // flag for resetting the filter
    bool m_flag_reset_filter;
    bool m_flag_run_calibration_aruco;
    bool m_flag_use_acc;

    bool m_flag_draw_markers;

    // poses for reset
    int m_num_sample_reset_max;
    std::vector<Eigen::Vector3d> m_pose_reset;

    // for calibration
    int m_num_sample_calibration;
    std::vector<Eigen::Vector3d> m_pose_calibration;

    // callback functions
    void camera_rgb_callback(const sensor_msgs::ImageConstPtr &image_msg);
    void odom_callback(const nav_msgs::OdometryConstPtr &odom_msg);
    void reset_callback(const std_msgs::BoolConstPtr &reset_msg);

    // function to load parameters from .json file
    void load_map(const std::string file_name);
    void load_marker_map(json &j);
    void load_board_map(json &j);
    void load_filter_param(const std::string file_name);

    // update functions
    void detect_markers();
    void get_pose_from_markers(std::vector<double> &th_meas, std::vector<Eigen::Vector2d> &pos_meas,
                               std::vector<double> &dist_meas);
    void odom_process_update(const double v, const double om);
    void opt_flow_process_update(const double vx, const double vy, const double om);
    void measurement_update();
    void simple_update();

    void reset_filter();

    // calibrate the covariance of the aruco detection
    double get_measurement_cov_multiplier(double dist, double v, double om);
    void calibrate_cov_aruco();
};

} // namespace
