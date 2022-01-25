#ifndef PARAM_H_
#define PARAM_H_

#include <string>
#include <cmath>
#include <jsoncpp/json/json.h>
#include <eigen3/Eigen/Dense>

const double kPI = std::atan(1.0)*4;

class Param {
public:
    Param(const std::string& json_file);
    ~Param() = default;

    // configuration for UKF
    bool use_lidar;
    bool use_radar;
    bool use_camera;
    int n_state;
    int n_state_aug;
    int n_lidar;
    int n_radar;
    int n_camera;
    double std_acceleration;
    double std_yaw_rate;
    double std_lidar_x;
    double std_lidar_y;
    double std_radar_x;
    double std_radar_y;
    double std_radar_v;
    Eigen::MatrixXd R_lidar, R_radar;

    // configuration for JPDA
    double p_d;
    double p_g; 
    double association_cost; // Euclidean distance threshold
    double g_sigma; // ellipse gate threshold
    double dist_threshold;
    double iou_threshold;

    // for Tracker Management
    int min_accept_detections;
    int max_no_detections;
};

#endif /* PARAM_H */
