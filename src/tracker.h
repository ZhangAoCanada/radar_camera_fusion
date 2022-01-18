#ifndef TRACKER_H_
#define TRACKER_H_

#include <iostream>
#include <vector>
#include <memory>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "param.h"
#include "track.h"
#include "detection.h"

class Tracker {
public:
    Tracker(const Param& param);
    ~Tracker() = default;

    virtual void track(Detection& detection) = 0;
    virtual void track(Detection& detection, std::vector<bool> is_associated, int& track_id) = 0;
    void addTrack(std::shared_ptr<Track> track) { tracks_.push_back(track); }
    const int size() const { return tracks_.size(); }
    const std::vector<std::shared_ptr<Track>>& getTracks() const { return tracks_; }

protected:
    const static int MAX_ASSOCIATIONS = 10000;
    bool initialized_;
    bool start_tracking_;
    Param param_;
    std::vector<std::shared_ptr<Track>> tracks_;
    std::vector<Eigen::VectorXd> prev_detections_;
    std::vector<Eigen::VectorXd> not_associated_;
    Eigen::MatrixXd beta_;
    Eigen::VectorXd last_beta;

    std::vector<bool> analyzeTracks(const Eigen::MatrixXd& q, std::vector<Detection> detections);
    std::vector<Eigen::MatrixXd> generateHypothesis(const std::vector<Eigen::VectorXd>& selected_detections, const Eigen::MatrixXd& q);
    Eigen::MatrixXd jointProbability(const std::vector<Eigen::MatrixXd>& association_matrices, const std::vector<Eigen::VectorXd>& selected_detections);

// private:
//     virtual void delete_tracks() = 0;
//     virtual void manage_new_tracks() = 0;
//     virtual void associate(std::vector<Eigen::VectorXd>& seleted_detections, Eigen::MatrixXd& q, std::vector<Detection>& detections, std::vector<bool> is_associated);

}; // class Tracker

#endif // TRACKER_H_