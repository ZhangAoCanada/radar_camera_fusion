#ifndef JPDA_TRACKER_H_
#define JPDA_TRACKER_H_

#include <iostream>
#include <vector>
#include <memory>
#include <numeric>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "hungarianAlg.h"
#include "track.h"
#include "tracker.h"

class JPDATracker: public Tracker {
public:
    JPDATracker(const Param& param);
    ~JPDATracker() = default;

    void track(std::vector<Detection>& detections);
    // void track(Detection& detection, std::vector<bool> is_associated, int& track_id) {;}

private:
    std::vector<std::shared_ptr<JPDATracker>> trackers_; // for tmp trackers

    void deleteTracks();
    void manageNewTracks();

    void associate(std::vector<Detection>& selected_detections, cv::Mat& q, std::vector<Detection>& detections, std::vector<bool>& is_associated);
};

#endif // JPDA_TRACKER_H_
