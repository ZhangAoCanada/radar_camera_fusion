#include <fstream>
#include <iostream>
#include "param.h" 

Param::Param(const std::string& json_file) {
    Json::Value root;
    Json::Reader reader;
    std::ifstream fin(json_file.c_str());

    if (!fin.is_open())
        std::cerr << "[ERR] Json file not found: " << json_file << std::endl;

    reader.parse(fin, root);

    /***************** load configuration for UKF ********************/
    Json::Value config_ukf = root["UKF"];

    use_lidar = config_ukf["use_lidar"].asBool();
    use_radar = config_ukf["use_radar"].asBool();
    use_camera = config_ukf["use_camera"].asBool();
    n_state = config_ukf["n_state"].asInt();
    n_state_aug = config_ukf["n_state_aug"].asInt();
    n_lidar = config_ukf["n_lidar"].asInt();
    n_radar = config_ukf["n_radar"].asInt();
    n_camera = config_ukf["n_camera"].asInt();
    std_acceleration = config_ukf["std_acceleration"].asDouble();
    std_yaw_rate = config_ukf["std_yaw_rate"].asDouble();
    std_lidar_x = config_ukf["std_lidar_x"].asDouble();
    std_lidar_y = config_ukf["std_lidar_y"].asDouble();
    std_radar_x = config_ukf["std_radar_x"].asDouble();
    std_radar_y = config_ukf["std_radar_y"].asDouble();
    std_radar_v = config_ukf["std_radar_v"].asDouble();
    std_camera_x = config_ukf["std_camera_x"].asDouble();
    std_camera_y = config_ukf["std_camera_y"].asDouble();
    std_camera_v = config_ukf["std_camera_v"].asDouble();

    // The dimension of R_lidar is n_lidar x n_lidar
    R_lidar = Eigen::MatrixXd(n_lidar, n_lidar);
    R_lidar <<  std_lidar_x * std_lidar_x, 0,
                0, std_lidar_y * std_lidar_y;

    // The dimension of R_radar is n_radar x n_radar
    R_radar = Eigen::MatrixXd(n_radar, n_radar);
    R_radar <<  std_radar_x * std_radar_x,   0,    0,
                0, std_radar_y * std_radar_y,      0,
                0,      0, std_radar_v * std_radar_v;

    // The dimension of R_camera is n_camera x n_camera
    R_camera = Eigen::MatrixXd(n_camera, n_camera);
    R_camera << std_camera_x * std_camera_x, 0, 0,
                0, std_camera_y * std_camera_y, 0,
                0, 0, std_camera_v * std_camera_v;



    std::cout << "[UKF parameters loaded: ]" << std::endl;
    std::cout << "\t use_lidar: " << use_lidar << std::endl;
    std::cout << "\t use_radar: " << use_radar << std::endl;
    std::cout << "\t use_camera: " << use_camera << std::endl;
    std::cout << "\t n_state: " << n_state << std::endl;
    std::cout << "\t n_state_aug: " << n_state_aug << std::endl;
    std::cout << "\t n_lidar: " << n_lidar << std::endl;
    std::cout << "\t n_radar: " << n_radar << std::endl;
    std::cout << "\t n_camera: " << n_camera << std::endl;
    std::cout << "\t std_acceleration: " << std_acceleration << std::endl;
    std::cout << "\t std_yaw_rate: " << std_yaw_rate << std::endl;
    std::cout << "\t std_lidar_x: " << std_lidar_x << std::endl;
    std::cout << "\t std_lidar_y: " << std_lidar_y << std::endl;
    std::cout << "\t std_radar_x: " << std_radar_x << std::endl;
    std::cout << "\t std_radar_y: " << std_radar_y << std::endl;
    std::cout << "\t std_radar_v: " << std_radar_v << std::endl;
    std::cout << "\t std_camera_x: " << std_camera_x << std::endl;
    std::cout << "\t std_camera_y: " << std_camera_y << std::endl;
    std::cout << "\t std_camera_v: " << std_camera_v << std::endl;

    /***************** load configuration for JPDA ********************/
    Json::Value config_jpda = root["JPDA"];

    p_d = config_jpda["p_d"].asDouble();
    p_g = config_jpda["p_g"].asDouble();
    association_cost = config_jpda["association_cost"].asDouble();
    g_sigma = config_jpda["g_sigma"].asDouble();
    dist_threshold = config_jpda["distance_threshold"].asDouble();
    iou_threshold = config_jpda["iou_threshold"].asDouble();

    std::cout << "[JPDA parameters loaded: ]" << std::endl;
    std::cout << "\t p_d: " << p_d << std::endl;
    std::cout << "\t g_sigma: " << g_sigma << std::endl;
    std::cout << "\t dist_threshold: " << dist_threshold << std::endl;
    std::cout  << "\t iou_threshold: " << iou_threshold << std::endl;

    /***************** load configuration for JPDA ********************/
    Json::Value config_tacker = root["Tracker"];

    min_accept_detections = config_tacker["min_accept_detections"].asInt();
    max_no_detections = config_tacker["max_no_detections"].asInt();

    std::cout << "[Tracker parameters loaded: ]" << std::endl;
    std::cout << "\t min_accept_detections: " << min_accept_detections << std::endl;
    std::cout << "\t max_no_detections: " << max_no_detections << std::endl;

    fin.close();
}