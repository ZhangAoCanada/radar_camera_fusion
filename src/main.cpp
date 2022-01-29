#include <iostream>
#include <string>

#include "track.h"
#include "tracker.h"
#include "readparam.h"

inline double normAngle(double phi) {
    if (phi > M_PI) {
        phi -= 2 * M_PI;
    } else if (phi < -M_PI) {
        phi += 2 * M_PI;
    }
    return phi;
}



int main(int argc, char* argv[]) {
    std::string in_filename = "/home/ao/1R1V/experiments/jpda_ukf_test/data/unit_test_camcam_txt.txt";
    std::string out_filename = "/home/ao/1R1V/experiments/jpda_ukf_test/data/unit_test_camcam_out.txt";
    std::string config_filename = "/home/ao/1R1V/experiments/jpda_ukf_test/config.json";

    std::ifstream in_stream(in_filename.c_str(), std::ifstream::in);
    std::ofstream out_stream(out_filename.c_str(), std::ofstream::out);

    std::string line;
    int frame_id;
    long double timestamp_sec;
    long long timestamp;
    int sensor_id;
    double z, x, y, vz, vx, vy;
    double v, yaw;

    Param param;
    Tracker tracker(param);
    Detect::SensorType sensor_type;
    
    std::vector<std::vector<Detect>> all_sensor_data;
    std::vector<Detect> frame_sensor_data;
    int frame_id_count = 1;

    while (getline(in_stream, line)) {
        Detect sensor_data;

        std::istringstream iss(line);
        iss >> frame_id;
        iss >> timestamp_sec;
        iss >> sensor_id;
        iss >> z;
        iss >> x;
        iss >> y;
        iss >> vz;
        iss >> vx;
        iss >> vy;
        // other parameters 
        timestamp = timestamp_sec * 1000000; // standard timestamp is in microseconds
        sensor_type = sensor_id == 1? Detect::CAMERA : Detect::RADAR;
        v = sqrt(vx * vx + vz * vz);
        yaw = atan2(vx, vz);

        // NOTE: normalize the yaw angle
        while (abs(yaw) > M_PI / 2) {
            yaw = yaw > 0? yaw - M_PI : yaw + M_PI;
        }

        sensor_data.timestamp = timestamp;
        sensor_data.timestamp_sec = timestamp_sec;
        sensor_data.sensor_type = sensor_type;
        sensor_data.position = Eigen::VectorXd(2);
        sensor_data.position << x, z;
        sensor_data.v = v;
        sensor_data.yaw = yaw;

        if (frame_id_count == frame_id) {
            frame_sensor_data.push_back(sensor_data);
        } else {
            all_sensor_data.push_back(frame_sensor_data);
            frame_sensor_data.clear();
            frame_id_count = frame_id;
            frame_sensor_data.push_back(sensor_data);
        }
    }

    out_stream << "frame_id" << "\t";
    out_stream << "time_stamp" << "\t";
    out_stream << "track_id" << "\t";
    out_stream << "x" << "\t";
    out_stream << "y" << "\t";
    out_stream << "angle" << "\t";
    out_stream << "mx" << "\t";
    out_stream << "my" << "\n";
    // out_stream << "sensor_type" << "\t";
    // out_stream << "NIS" << "\t";
    // out_stream << "x_measured" << "\t";
    // out_stream << "y_measured" << "\t";
    // out_stream << "v_measured" << "\t";
    // out_stream << "yaw_measured" << "\n";

    /************ NOTE: Start tracking system from here. *************/

    for(int k = 0; k < all_sensor_data.size(); k++){
        // if (k > 100) continue;
        std::vector<Detect> cam_data;
        std::vector<Detect> radar_data;
        std::vector<Eigen::VectorXd> result;

        auto sensor_data = all_sensor_data[k];

        if (sensor_data.size() == 0 || k <= 1)
            continue;

        // std::cout << "[SENSOR INFO] the " << k << "-th frame \n";
        // for (auto& data: sensor_data) {
        //     std::string sensor_name = data.sensor_type == Detect::CAMERA? "CAMERA" : "RADAR";
        //     std::cout << sensor_name << "\t";
        //     for (int l = 0; l < data.position.size(); l++) {
        //         std::cout << data.position(l) << " ";
        //     }
        //     std::cout << "\n";
        // }
        
        for (const auto& data: sensor_data) {
            if (data.sensor_type == Detect::CAMERA) {
                cam_data.push_back(data);
            } else if (data.sensor_type == Detect::RADAR) {
                radar_data.push_back(data);
            }
        }

        if (cam_data.size() > 0) {
            tracker.track(cam_data, cam_data[0].timestamp_sec, result);
        }
        // if (radar_data.size() > 0) {
        //     tracker.track(radar_data);
        // }
        std::cout << "[INFO] number of resutls: " << result.size() << std::endl;


        /************ NOTE: write results to txt  ************/
        for (const auto& data: result) {
            out_stream << k << "\t";
            out_stream << cam_data[0].timestamp_sec << "\t";
            out_stream << data(0) << "\t";
            out_stream << data(1) << "\t";
            out_stream << data(2) << "\t";
            out_stream << data(3) << "\t";
            out_stream << data(4) << "\t";
            out_stream << data(5) << "\n";
            std::cout << "[TRACK RESULT] the " << k << "-th frame ";
            for (int l=0; l < data.size(); l++) {
                std::cout << data(l) << " ";
            }
            std::cout << "\n";
        }

    }

    std::cout << "Done.\n";

    return 0;
}



/* ******************************************************************** */
/* **************************** UKF test ****************************** */
/* ******************************************************************** */
// int main(int argc, char* argv[]) {

//     // Tools::checkArgs(argc, argv);
//     // std::string in_filename = argv[1];
//     // std::string out_filename = argv[2];

//     std::string in_filename = "/home/ao/1R1V/experiments/jpda_ukf_test/data/data-3.txt";
//     std::string out_filename = "/home/ao/1R1V/experiments/jpda_ukf_test/data/out-3.txt";
//     std::string config_filename = "/home/ao/1R1V/experiments/jpda_ukf_test/config.json";

//     std::ifstream in_file(in_filename.c_str(), std::ifstream::in);
//     std::ofstream out_file(out_filename.c_str(), std::ofstream::out);
//     const Param param(config_filename);

//     checkFiles(in_file, in_filename, out_file, out_filename);

//     std::vector<Detection> all_sensor_data;
//     std::vector<Detection> all_truth_data;

//     double val1, val2, val3;
//     double x, y, vx, vy, v, yaw, yawrate;
//     long long timestamp;
//     std::string sensor_id;
//     SensorType sensor_type;

//     std::string line;

//     while(getline(in_file, line)){

//         std::istringstream iss(line);

//         iss >> sensor_id;

//         if(sensor_id.compare("L") == 0){

//             iss >> val1;
//             iss >> val2;
//             iss >> timestamp;
//             sensor_type = SensorType::LIDAR;

//         }else if(sensor_id.compare("R") == 0){

//             iss >> val1;
//             iss >> val2;
//             iss >> val3;
//             iss >> timestamp;
//             double tmp1 = val1 * std::cos(val2);
//             double tmp2 = val1 * std::sin(val2);
//             val1 = tmp1;
//             val2 = tmp2;
//             sensor_type = SensorType::RADAR;

//         }

//         Detection sensor_data(param, timestamp, val1, val2, val3, 0.0, 0.0, sensor_type);

//         iss >> x;
//         iss >> y;
//         iss >> vx;
//         iss >> vy;
//         iss >> yaw;
//         iss >> yawrate;

//         v = sqrt(vx * vx + vy * vy);
//         sensor_type = SensorType::TRUTH;

//         Detection truth_data(param, timestamp, x, y, v, yaw, yawrate, sensor_type);


//         // std::cout << "[Sensor] " << sensor_data.getSensorType() << " " << sensor_data.getVector().transpose() << std::endl;
//         // std::cout << "[Gt] " << truth_data.getSensorType() << " " << truth_data.getVector().transpose() << std::endl;

//         all_sensor_data.push_back(sensor_data);
//         all_truth_data.push_back(truth_data);
//     }

//     out_file << "time_stamp" << "\t";
//     out_file << "px_state" << "\t";
//     out_file << "py_state" << "\t";
//     out_file << "v_state" << "\t";
//     out_file << "yaw_angle_state" << "\t";
//     out_file << "yaw_rate_state" << "\t";
//     out_file << "sensor_type" << "\t";
//     out_file << "NIS" << "\t";
//     out_file << "px_measured" << "\t";
//     out_file << "py_measured" << "\t";
//     out_file << "px_ground_truth" << "\t";
//     out_file << "py_ground_truth" << "\t";
//     out_file << "vx_ground_truth" << "\t";
//     out_file << "vy_ground_truth" << "\n";

//     /************************************
//      * UKF sensor fusion starting here.
//      ***********************************/
//     UKF ukf(param);

//     // std::vector<Eigen::VectorXd> predictions;
//     // std::vector<Eigen::VectorXd> ground_truths;
//     // std::vector<Eigen::VectorXd> estimations_vec;
//     // std::vector<Eigen::VectorXd> ground_truths_vec;

//     Eigen::VectorXd prediction;
//     Eigen::VectorXd measurement;


//     double nis;

//     for(int k = 0; k < all_sensor_data.size(); ++k){
//         int sensor_type;
//         long long timestamp;
//         std::string sensor_name;

//         auto gt =  all_truth_data[k];
//         auto sensor_data = all_sensor_data[k];

//         ukf.process(sensor_data);

//         timestamp = sensor_data.getTimestamp();
//         sensor_type = sensor_data.getSensorType();
//         sensor_name = ((sensor_type == SensorType::RADAR) ? "radar" : "lidar");
//         measurement = sensor_data.getVector();

//         prediction = ukf.getState();
//         nis = sensor_name == "radar" ? ukf.getNISRadar() : ukf.getNISLidar();

//         out_file << timestamp << "\t";
//         out_file << prediction(0) << "\t";
//         out_file << prediction(1) << "\t";
//         out_file << prediction(2) << "\t";
//         out_file << prediction(3) << "\t";
//         out_file << prediction(4) << "\t";

//         out_file << sensor_name << "\t";
//         out_file << nis << "\t";

//         out_file << measurement(0) << "\t";
//         out_file << measurement(1) << "\t";


//         Eigen::VectorXd gt_vec = gt.getVector();
//         out_file << gt_vec(0) << "\t";
//         out_file << gt_vec(1) << "\t";
//         out_file << gt_vec(2) << "\t";
//         out_file << gt_vec(3) << "\n";

//         // estimation.set(timestamp, DataPointType::STATE, prediction);
//         // estimations_vec.push_back(estimation.get_vec());
//         // predictions.push_back(prediction);

//         // ground_truths_vec.push_back(truth);
//         // ground_truths.push_back(all_truth_data[k].get_state());
//     }



//     // /*******************************************************************
//     //  * CALCULATE ROOT MEAN SQUARE ERROR
//     //  *******************************************************************/
//     //  VectorXd RMSE;

//     //  RMSE = calculate_RMSE(estimations_vec, ground_truths_vec);
//     //  cout << "RMSE:" << endl << RMSE << endl;

//     // /*******************************************************************
//     //  * PRINT TO CONSOLE IN A NICE FORMAT FOR DEBUGGING
//     //  *******************************************************************/
//     //  //print_EKF_data(RMSE, predictions, ground_truths, all_sensor_data);

//     // /*******************************************************************
//     //  * CLOSE FILES
//     //  *******************************************************************/
//     // if(out_file.is_open()) { 
//     //   out_file.close();
//     // }
//     // if(in_file.is_open()) {
//     //   in_file.close();
//     // }

//     std::cout << "Done!" << std::endl;
//     return 0;
// }
