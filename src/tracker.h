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

    virtual void track(std::vector<Detection>& detections) = 0;
    // virtual void track(Detection& detection, std::vector<bool> is_associated, int& track_id) = 0;
    void addTrack(std::shared_ptr<Track> track) { tracks_.push_back(track); }
    const int size() const { return tracks_.size(); }
    const std::vector<std::shared_ptr<Track>>& getTracks() const { return tracks_; }

protected:
    const static int MAX_ASSOCIATIONS = 10000;
    bool initialized_;
    bool start_tracking_;
    int track_id_;
    Param param_;
    std::vector<std::shared_ptr<Track>> tracks_;
    std::vector<std::shared_ptr<Track>> tracks_tmp_;
    // std::vector<Eigen::VectorXd> prev_detections_;
    // std::vector<Eigen::VectorXd> not_associated_;
    std::vector<Detection> prev_detections_;
    std::vector<Detection> not_associated_;
    Eigen::MatrixXd beta_;
    Eigen::VectorXd last_beta_;

    Eigen::MatrixXd jointProbability(const std::vector<Eigen::MatrixXd>& association_matrices, const std::vector<Detection>& selected_detections);
    std::vector<Eigen::MatrixXd> generateHypothesis(const std::vector<Detection>& selected_detections, const cv::Mat& q);
    std::vector<bool> analyzeTracks(const cv::Mat& q, std::vector<Detection>& detections);

private:
    virtual void deleteTracks() = 0;
    virtual void manageNewTracks() = 0;
    virtual void associate(std::vector<Detection>& selected_detections, cv::Mat& q, std::vector<Detection>& detections, std::vector<bool>& is_associated) = 0;

}; // class Tracker

#endif // TRACKER_H_
