// #pragma once
#ifndef DETECTION_H_
#define DETECTION_H_

#include <string>
#include <eigen3/Eigen/Dense>

#include "param.h"

enum SensorType {
    LIDAR,
    RADAR,
    CAMERA,
    TRUTH
};

/* Define the sensor data object to store the detections from the sensors 
 * @param vector_: the vector used in KF for fusion. */
class Detection {
public:
    Detection(const Param& param, const long long& timestamp, const float& x, const float& y, const float& v, const float& yaw, const float& yaw_rate, const SensorType& sensor_type)
        : timestamp_(timestamp), x_(x), y_(y), v_(v), yaw_(yaw), yaw_rate_(yaw_rate), sensor_type_(sensor_type) {
        // define sensor vector length
        // TODO: put these param in the param.h
        if (sensor_type_ == SensorType::LIDAR) {
            vector_ = Eigen::VectorXd(param.n_lidar);
            vector_ << x_, y_;
        }
        else if (sensor_type_ == SensorType::RADAR) {
            vector_ = Eigen::VectorXd(param.n_radar);
            vector_ << x_, y_, v_;
        }
        else {
            vector_ = Eigen::VectorXd(param.n_state);
            vector_ << x_, y_, v_, yaw_, yaw_rate_;
        }
    }

    ~Detection() = default;

    Detection& operator=(const Detection& other) {
        timestamp_ = other.timestamp_;
        x_ = other.x_;
        y_ = other.y_;
        v_ = other.v_;
        yaw_ = other.yaw_;
        yaw_rate_ = other.yaw_rate_;
        vector_ = other.vector_;
        return *this;
    }

    const SensorType& getSensorType() const { return sensor_type_; }
    const long long getTimestamp() const { return timestamp_; }
    const float getX() const { return x_; }
    const float getY() const { return y_; }
    const float getV() const { return v_; }
    const float getYaw() const { return yaw_; }
    const float getYawRate() const { return yaw_rate_; }
    const Eigen::VectorXd getVector() const { return vector_; }

private:
    SensorType sensor_type_;
    long long timestamp_;
    float x_, y_, v_, yaw_, yaw_rate_; 
    Eigen::VectorXd vector_;
};


#endif // DETECTION_H_