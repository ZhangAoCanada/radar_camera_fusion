#include "jpda_tracker.h"


JPDATracker::JPDATracker(const Param& param) 
    : Tracker(param) {
    initialized_ = true;
    start_tracking_ = false;
    track_id_ = 0;
}


void JPDATracker::track(std::vector<Detection>& detections) {
    if (initialized_) {
        prev_detections_.clear();
        for (const auto& det: detections)
            prev_detections_.push_back(det);
        initialized_ = false;
    } else if (!initialized_ && !start_tracking_) {
        not_associated_.clear();
        for (const auto& det: detections)
            not_associated_.push_back(det);
        tracks_tmp_.clear();
        manageNewTracks();
        if (tracks_tmp_.size() > 0) {
            tracks_.clear();
            tracks_ = tracks_tmp_;
            start_tracking_ = true;
        } else {
            prev_detections_ = not_associated_;
        }
    } else {
        not_associated_.clear();
        std::vector<bool> is_associated(detections.size(), false);
        int i;
        int j = 0;
        for (auto& track: tracks_) {
            // if (track->getS().size() == 0) {
            //     std::cout << "[TRACK INFO] S is empty...\n";
            // }
            // NOTE: start tracking
            const double dt = track->getDeltaT(detections.at(0));
            track->predict(dt);
            if (track->getId() == -1 && track->isAlive() && track->getTrackState() == Track::ACCEPT) {
                track->setId(track_id_++);
            }
            ++j;
        }

        cv::Mat_<int> q(cv::Size(tracks_.size(), detections.size()), int(0));
        std::vector<Detection> selected_detections;
        std::vector<bool> not_associated;

        associate(selected_detections, q, detections, is_associated);

		// NOTE: check association
		if (q.total() == 0) {
			for (const auto& track: tracks_) {
				track->notDetected();
			}
		} else {
			not_associated = analyzeTracks(q, detections);
		}

		const std::vector<Eigen::MatrixXd>& association_matrices = generateHypothesis(selected_detections, q);
		// TODO: association gate EllipseVolume tooooo small
		beta_ = jointProbability(association_matrices, selected_detections);
    }
}


void JPDATracker::deleteTracks()
{
    for(int i = tracks_.size() - 1; i >= 0; --i) {
        if(!tracks_.at(i)->isAlive() && tracks_.at(i)->getId() != -1) {
        tracks_.erase(tracks_.begin() + i);
        }
    }
}


void JPDATracker::manageNewTracks()
{
    const uint& prevDetSize = prev_detections_.size();
    const uint& deteSize = not_associated_.size();
    if(prevDetSize == 0) {
        prev_detections_ = not_associated_;
    } else if (deteSize == 0) {
        prev_detections_.clear();
    } else {
        cv::Mat assigmentsBin = cv::Mat::zeros(cv::Size(deteSize, prevDetSize), CV_32SC1);
        cv::Mat costMat = cv::Mat(cv::Size(deteSize, prevDetSize), CV_32FC1);
        
        auto euclideanDist = [](const Eigen::Vector2f& p1, const Eigen::Vector2f& p2)
                    { 
                    const Eigen::Vector2f& tmp = p1 - p2; 
                    return sqrt(tmp(0) * tmp(0) + tmp(1) * tmp(1));
                    };

        assignments_t assignments;
        distMatrix_t costs(deteSize * prevDetSize);

        for(uint i = 0; i < prevDetSize; ++i) {
            for(uint j = 0; j < deteSize; ++j) {
                Eigen::Vector2f p1, p2;
                p1(0) = not_associated_.at(j).getX();
                p1(1) = not_associated_.at(j).getY();
                p2(0) = prev_detections_.at(i).getX();
                p2(1) = prev_detections_.at(i).getY();
                costs.at(i + j * prevDetSize ) = euclideanDist(p1, p2);
                // costMat.at<double>(i, j) = costs.at(i + j * prevDetSize );
                costMat.at<float>(i, j) = costs.at(i + j * prevDetSize ); 
            }
        }
        
        AssignmentProblemSolver APS;
        APS.Solve(costs, prevDetSize, deteSize, assignments, AssignmentProblemSolver::optimal);

        const uint& assSize = assignments.size();
        
        for(uint i = 0; i < assSize; ++i) {
            if( assignments[i] != -1 && costMat.at<double>(i, assignments[i]) < param_.association_cost) {
                assigmentsBin.at<int>(i, assignments[i]) = 1;
            }
        }
        
        const uint& rows = assigmentsBin.rows;
        const uint& cols = assigmentsBin.cols;
        
        // NOTE: as I observed, this tracker is to store all the tracks tmperorily
        // std::shared_ptr<JPDATracker> tracker = std::make_shared<JPDATracker>(param_);
            
        
        for(uint i = 0; i < rows; ++i) {
            for(uint j = 0; j < cols; ++j) {
                if(assigmentsBin.at<int>(i, j)) {
                    std::shared_ptr<Track> tr = std::make_shared<Track>(param_, prev_detections_.at(i));
					tr->process(not_associated_.at(j));
                    tracks_tmp_.push_back(tr);
                }
            }
        }

        // if(tracker->size() > 0) {
        //     trackers_.push_back(tracker);
        // }
        
        cv::Mat notAssignedDet(cv::Size(assigmentsBin.cols, 1), CV_32SC1, cv::Scalar(0));
        for(uint i = 0; i < assigmentsBin.rows; ++i) {
            notAssignedDet += assigmentsBin.row(i);
        }
        
        notAssignedDet.convertTo(notAssignedDet, CV_8UC1);
        notAssignedDet = notAssignedDet == 0;
        
        cv::Mat dets;
        cv::findNonZero(notAssignedDet, dets);
        prev_detections_.clear();
        for(uint i = 0; i < dets.total(); ++i) {
            const uint& idx = dets.at<cv::Point>(i).x;
            prev_detections_.push_back(not_associated_.at(i));
        }
    }
}


void JPDATracker::associate(std::vector<Detection>& selected_detections, cv::Mat& q, std::vector<Detection>& detections, std::vector<bool>& is_associated)
{
    //Extracting the measurements inside the validation gate for all the tracks
    //Create a q matrix with a width = clutter + number of tracks
    q = cv::Mat_<int>(cv::Size(tracks_.size() + 1, detections.size()), int(0));
    uint validationIdx = 0;
    not_associated_.clear();
    uint j = 0;
    
    auto euclideanDist = [](const Eigen::VectorXd& _p1, const Eigen::VectorXd& _p2)
                    { 
                    const Eigen::VectorXd& tmp = _p1 - _p2; 
                    return sqrt(tmp(0) * tmp(0) + tmp(1) * tmp(1));
                    };
    
    for(const auto& detection : detections) {
		Eigen::VectorXd det = detection.getVector();
		uint i = 1;
		bool found = false;
		cv::Mat det_cv;
		cv::eigen2cv(det, det_cv);
		det_cv = det_cv.t();
		int det_size = det.size();
		for (auto& track : tracks_) {
			const Eigen::VectorXd& tr = track->getState();
			Eigen::VectorXd tr_pnt(det_size);
			for (int i = 0; i < det_size; i++) {
				tr_pnt(i) = tr(i);
			}
			cv::Mat tr_cv;
			cv::eigen2cv(tr_pnt, tr_cv);
			tr_cv = tr_cv.t();
			const int& id = track->getId();
			const Eigen::MatrixXd& S = track->getS().inverse();
			cv::Mat S_cv;
			cv::eigen2cv(S, S_cv);
			const double& mah = cv::Mahalanobis(tr_cv, det_cv, S_cv);
			const double& eucl = euclideanDist(det, tr_pnt);
			if(mah <= param_.g_sigma && eucl <= param_.association_cost) {
				q.at<int>(validationIdx, 0) = 1;
				q.at<int>(validationIdx, i) = 1;
				found = true;
			}
			++i;
		}
		if(found) {
			selected_detections.push_back(detection);
			is_associated.at(j) = true;
			validationIdx++;
		} else {
			not_associated_.push_back(detection);
		}
		++j;
	}
	q = q(cv::Rect(0, 0, tracks_.size() + 1, validationIdx));
}

		// NOTE: original implementation
		/*Eigen::Vector2f det;*/
		/*det << detection.getX(), detection.getY();*/
		/*uint i = 1;*/
		/*bool found = false;*/
		/*cv::Mat det_cv(cv::Size(2, 1), CV_32FC1);*/
		/*det_cv.at<double>(0) = det(0);*/
		/*det_cv.at<double>(1) = det(1);*/
		/*for(auto& track : tracks_) {*/
			/*const Eigen::VectorXd& tr = track->getState();*/
			/*Eigen::Vector2f tr_pnt;*/
			/*tr_pnt(0) = tr(0);*/
			/*tr_pnt(1) = tr(1);*/
			/*cv::Mat tr_cv(cv::Size(2, 1), CV_32FC1);*/
			/*tr_cv.at<double>(0) = tr(0);*/
			/*tr_cv.at<double>(1) = tr(1);*/
			/*const int& id = track->getId();*/
			/*const Eigen::MatrixXd& S = track->getS().inverse();*/
			/*cv::Mat S_cv;*/
			/*cv::eigen2cv(S, S_cv);*/
			/*const double& mah = cv::Mahalanobis(tr_cv, det_cv, S_cv);*/
			/*const double& eucl = euclideanDist(det, tr_pnt);*/
			/*if(mah <= param_.g_sigma && eucl <= param_.association_cost) {*/
				/*q.at<int>(validationIdx, 0) = 1;*/
				/*q.at<int>(validationIdx, i) = 1;*/
				/*found = true;*/
			/*}*/
			/*++i;*/
		/*}*/
		/*if(found) {*/
			/*selected_detections.push_back(detection);*/
			/*is_associated.at(j) = true;*/
			/*validationIdx++;*/
		/*} else {*/
			/*not_associated_.push_back(detection);*/
		/*}*/
		/*++j;*/
	/*}*/
	/*q = q(cv::Rect(0, 0, tracks_.size() + 1, validationIdx));*/
/*}*/
