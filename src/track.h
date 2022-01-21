#ifndef TRACK_H_
#define TRACK_H_

#include <iostream>
#include <vector>
#include <memory>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "param.h"
#include "ukf.h"
#include "detection.h"


class Track {
public:
    Track(const Param& param);
    Track(const Param& param, Detection& detection);
    ~Track() = default;

    enum TrackState {
        NONE,
        INITIALIZED,
        ACCEPT,
        DISCARD
    };

    void process(Detection& detection);
    void predict(double delta_t);
    void update(Detection& detection, double delta_t);

    void setId(int id) { id_ = id; }
    void increaseLifeTime() { life_time_++; }
    void notDetected() { no_detections_++; }
    bool isAlive() { return no_detections_ < max_no_detections_; }
    const int getId() const { return id_; }
    const Eigen::VectorXd getState() const { return ukf_->getState(); }
    const Eigen::MatrixXd getS() const { return ukf_->getS(); }
    const Eigen::VectorXd getMeasurementPred() const { 
        return ukf_->getMeasurementPred(); 
    }
    const TrackState getTrackState() const { return track_state_; }
    const double getEllipseVolume() const { return ellipse_volume_; }
    const double getDeltaT(Detection& detection) const { 
        return ukf_->getDeltaT(detection); 
        }

private:
    std::shared_ptr<UKF> ukf_;
    bool initialized_;
    int id_;
    int life_time_;
    int no_detections_;
    int min_accept_detections_;
    int max_no_detections_;

    double g_sigma_;
    double gamma_;
    double ellipse_volume_;
    // double entropy_initial_;
    TrackState track_state_;

}; // class Track

#endif // TRACK_H_