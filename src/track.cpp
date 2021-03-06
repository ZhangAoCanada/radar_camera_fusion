#include "track.h"


Track::Track(const Param& param) {
    initialized_ = true;

    ukf_ = std::make_shared<UKF>(param);
    id_ = -1;
    life_time_ = 0;
    no_detections_ = 0;
    min_accept_detections_ = param.min_accept_detections;
    max_no_detections_ = param.max_no_detections;
    track_state_ = TrackState::NONE;
    g_sigma_ = param.g_sigma;
    // gamma_ = param.gamma_;
}


Track::Track(const Param& param, Detection& detection) {
    initialized_ = true;

    ukf_ = std::make_shared<UKF>(param);
    id_ = -1;
    life_time_ = 0;
    no_detections_ = 0;
    min_accept_detections_ = param.min_accept_detections;
    max_no_detections_ = param.max_no_detections;
    track_state_ = TrackState::NONE;
    g_sigma_ = param.g_sigma;
    // gamma_ = param.gamma_;

    process(detection);
}


void Track::process(Detection& detection) {
    if (!initialized_) {
        ukf_->process(detection);
        initialized_ = true;
    } else {
        ukf_->process(detection);
    }
}

void Track::predict(double delta_t) {
    ukf_->prediction(delta_t);
    if (life_time_ == 0)
        track_state_ = TrackState::INITIALIZED;
    else if (no_detections_ >= max_no_detections_)
        track_state_ = TrackState::DISCARD;
    else if (life_time_ >= min_accept_detections_)
        track_state_ = TrackState::ACCEPT;
    else
        track_state_ = TrackState::NONE;

    Eigen::MatrixXd S = ukf_->getS();
    ellipse_volume_ = M_PI * g_sigma_ * sqrt(S.determinant());
}


void Track::update(Detection& detection, double delta_t) {
    increaseLifeTime();
    no_detections_ = 0;
    ukf_->update(detection, delta_t);
}


void Track::update(std::vector<Detection>& selected_detections, const Eigen::VectorXd& beta, const float& last_beta) {
    increaseLifeTime();
    no_detections_ = 0;
    ukf_->update(selected_detections, beta, last_beta);
}
