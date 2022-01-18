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
    float std_acceleration;
    float std_yaw_rate;
    float std_lidar_x;
    float std_lidar_y;
    float std_radar_x;
    float std_radar_y;
    float std_radar_v;
    Eigen::MatrixXd R_lidar, R_radar;

    // configuration for JPDA
    float p_d;
    float g_sigma;
    float dist_threshold;
    float iou_threshold;

    // for Tracker Management
    int min_accept_detections;
    int max_no_detections;
};

#endif /* PARAM_H */
