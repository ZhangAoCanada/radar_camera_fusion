#ifndef TRACK_H_
#define TRACK_H_

#include <iostream>
#include <vector>
#include <memory>
#include <eigen3/Eigen/Dense>

#include "param.h"
#include "ukf.h"
#include "detection.h"


class Track {
public:
    Track() = default;
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
    const TrackState getTrackState() const { return track_state_; }

private:
    std::shared_ptr<UKF> ukf_;
    bool initialized_;
    int id_;
    int life_time_;
    int no_detections_;
    int min_accept_detections_;
    int max_no_detections_;

    float g_sigma_;
    float gamma_;
    // float entropy_initial_;
    TrackState track_state_;
};

#endif