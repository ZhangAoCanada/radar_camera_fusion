#include "cal_iou.h"

inline float euclideanDist(const Eigen::VectorXd& _p1, const Eigen::VectorXd& _p2){
		return sqrt((_p1(0) - _p2(0))* (_p1(0) - _p2(0)) + (_p1(1) - _p2(1)) * (_p1(1) - _p2(1)));
}

void setupdate(std::set<int>& setin, int input){
	if(!setin.count(input)){
		setin.insert(input);
	}
}

/**
 * @brief cluster the associate matrix
 * @details https://github.com/StanfordVL/JRMOT_ROS python version
 * @param  cost_matrix
 *         cltrack:  cluster track
 *         dettrack : cluster detection
 */
void clusterq(const cv::Mat& cost_matrix, std::vector<std::vector<int> >& cltrack, std::vector<std::vector<int> >& dettrack){
	//cols for track, rows for det
	int num_tracks = cost_matrix.cols-1;
	int total_tracks = 0;
	int total_detections = 0;
	std::set<int> all_tracks;

	for(int i=1; i<num_tracks+1; ++i){
		all_tracks.insert(i);
	}

	std::set<int> all_visited_tracks;

	while(total_tracks<num_tracks){
		std::set<int> visited_detections;
		std::set<int> visited_tracks;
		std::set<int> potential_track;

		std::set_difference(all_tracks.begin(), all_tracks.end(), all_visited_tracks.begin(), all_visited_tracks.end(),
				std::insert_iterator<std::set<int>>(potential_track,potential_track.begin()));

 		auto it = potential_track.begin();
		//auto it = next(beg);

 		std::set<int> potential_tracks;
 		std::vector<int> potential_tracksv;
		potential_tracks.insert(*it);
		potential_tracksv.push_back(*it);

		while(!potential_tracks.empty()){
			int current_track = potential_tracksv[potential_tracksv.size()-1];
			potential_tracksv.pop_back();

			potential_tracks.erase(current_track);
			for(int j=0; j<cost_matrix.rows; ++j){
				if(cost_matrix.at<int>(j,current_track)==1){
					setupdate(visited_detections, j);
				}
			}
			visited_tracks.insert(current_track);
			for(auto& det:visited_detections){
				std::vector<int> connected_tracks;
				for(int j=1; j<cost_matrix.cols; ++j){
					if(cost_matrix.at<int>(det, j)==1){
						connected_tracks.push_back(j);
					}
				}
				for(auto tr:connected_tracks){
					if(visited_tracks.count(tr) || potential_tracks.count(tr)){
						continue;
					}
					potential_tracks.insert(tr);
					potential_tracksv.push_back(tr);
				}
			}
		}

		total_tracks += visited_tracks.size();

		std::vector<int> visited_tracksv;
		for(auto& x:visited_tracks){
			visited_tracksv.push_back(x);
		}

		std::vector<int> visited_detectionsv;
		for(auto& x:visited_detections){
			visited_detectionsv.push_back(x);
		}

		if(!visited_tracks.empty() && !visited_detections.empty()){
			cltrack.push_back(visited_tracksv);
			dettrack.push_back(visited_detectionsv);
		}

		total_detections += visited_detections.size();

		std::set<int> temp;
		std::set_union(all_visited_tracks.begin(), all_visited_tracks.end(), visited_tracks.begin(),
				visited_tracks.end(), std::insert_iterator<std::set<int>>(temp, temp.begin()));
		all_visited_tracks = temp;
	}

}


inline float Angle_cal(float x, float y) {
	float temp_tangle = 0;
	if (x == 0 && y == 0) {
		temp_tangle = 0;
	} else if (y >= 0) {
		temp_tangle = (float) atan2(y, x);
	} else if (y <= 0) {
		temp_tangle = (float) atan2(y, x) + 2 * 3.1415926;
	}
	return temp_tangle;
}

//初始化
void Tracker::Init(const Detection& detections, float& time){

	if(!init_){//表示第一帧存下的检测结果
		prev_detections_.clear();
		prev_detections_.swap(prev_detections_);

		for(auto& det:detections){
			prev_detections_.push_back(det);
		}

		init_ = true;
	}else if(init_ && !start_tracking_){//第二帧数据进这个
		not_associated_.clear();
		not_associated_.swap(not_associated_);

		for(auto& det:detections){
			not_associated_.push_back(det);
		}

		manage_tracks(time);//进行匹配初始化跟踪器

		if(tracks_.size()>0){//如果产生跟踪器表明开始跟踪，否则还当做第一帧做初始化
			start_tracking_= true;
		}else{
			prev_detections_ = not_associated_;
		}
	}

	pretime = time;
}
		
void Tracker::trackCam(const Detection& detections,float& time, std::vector<Eigen::VectorXd>& result){
	
	if(!init_ || !start_tracking_){
		Init(detections,time);
	}else{	//开始跟踪

		not_associated_.clear();
		not_associated_.swap(not_associated_);

		for (auto& track:tracks_){
			// if(track->GetTrackState()==Track_state_.Confirmed && track->Age()==0){
			if(track->GetTrackState()==Track_state_.Confirmed){
				confirmed_tracks_.push_back(track);
			}else{
				unconfirmed_tracks_.push_back(track);
			}
		}

		tracks_.clear();
		tracks_.swap(tracks_);

		std::vector<bool> MeaisAsso(detections.size(),false);

		cv::Mat_<int> q(cv::Size(confirmed_tracks_.size(), detections.size()), int(0));//基本关联矩阵q
		
    		Detection selected_detections;//关联上的量测
		for(const auto& track:confirmed_tracks_){
			track->Prediction(time);
		}

		std::vector<track_ptr> prun_tracks;
        associate(selected_detections, q, detections);

		if(q.total()==0){
			for(auto& track:confirmed_tracks_){
				track->MarkMissed();
			}
		}else{
			std::vector<std::vector<int> > cltrack;
			std::vector<std::vector<int> > cldet;
			std::vector<bool> missed_tracks(confirmed_tracks_.size(),true);

            // This is to cluster all the connected (joint) tracks (cltrack) and detections (cldet) for the JPDA computations. By doing so, we can effectively save computation time.
			clusterq(q, cltrack, cldet);
			int clustersize = cltrack.size();
			for(int i =0; i<clustersize; ++i){
				int colsize = cltrack[i].size();
				int rowsize = cldet[i].size();

				cv::Mat x = cv::Mat_<int>(cv::Size(colsize + 1, rowsize), int(1));

				for(int row=0; row<rowsize; ++row){
					int rowinq = cldet[i][row];
					for(int col=1; col<x.cols; ++col){
						x.at<int>(row,col) = q.at<int>(rowinq, cltrack[i][col-1]);
					}
				}
				//q(cv::Rect(colmin, rowmin, colsize, rowsize)).copyTo(x(cv::Rect(1, 0, colsize, rowsize)));
				std::vector<track_ptr> tracks;
				for(auto trackid:cltrack[i]){
					missed_tracks[trackid-1] = false;
					tracks.push_back(confirmed_tracks_[trackid-1]);
				}
				Detection selectdets;
				for(auto detid:cldet[i]){
					selectdets.push_back(selected_detections[detid]);
				}
				const Matrices& association_matrices = generate_hypothesis(x);//生成假设矩阵

				Eigen::MatrixXd beta = joint_probability(association_matrices, selectdets, tracks);//JPDAF
				int b=0;
				for(const auto& track:tracks){
					std::vector<Eigen::VectorXd> Zv;
					for(auto det : selectdets){
						Zv.push_back(det.position);
					}
					track->Update(Zv, beta.col(b), beta(beta.rows() - 1, b), time);//TODO change the imm_ukf
					b++;
					tracks_.push_back(track);
					prun_tracks.push_back(track);
				}
			}

			for(int i=0; i<confirmed_tracks_.size(); ++i){
				if(missed_tracks[i]){
					confirmed_tracks_[i]->MarkMissed();
					tracks_.push_back(confirmed_tracks_[i]);
				}
			}
		}


    		/*std::cout<<"q \n "<<q<<"\n q.total() "<<q.total()<<std::endl;
		if(q.total()==0){
			for(auto& track:confirmed_tracks_){
				track->MarkMissed();
			}
		}else{
			MeaisAsso = analyze_tracks(q);//分析哪些measure没有关联
			const Matrices& association_matrices = generate_hypothesis(q);//生成假设矩阵

			Eigen::MatrixXd beta = joint_probability(association_matrices, selected_detections);//JPDAF

			std::cout<<"########## BETA ##########\n"<<beta<<std::endl;

			int i=0;
			for(const auto& track:confirmed_tracks_){
		    	std::cout<<"MeaisAsso "<<MeaisAsso[i]<<std::endl;
				if(MeaisAsso[i]){
					std::vector<Eigen::VectorXd> Zv;
					for(auto det : selected_detections){
						Zv.push_back(det.position);
					}
					track->Update(Zv, beta.col(i), beta(beta.rows() - 1, i), time);//TODO change the imm_ukf
				}else{
					track->MarkMissed();
				}
				++i;
			}
		}*/

		std::vector<int> final_select;
		pruning(selected_detections,final_select,prun_tracks);//TODO 剪枝

        /* ------------------- debug ---------------- */
        // std::cout << "[INFO] number of confirmed tracks: " << confirmed_tracks_.size() << std::endl;
        // std::cout << "[INFO] number of tracks: " << tracks_.size() << std::endl;
        // for (auto& track : tracks_) {
        //     std::cout << "id " << track->GetId() << " "; 
        //     auto track_state = track->GetState();
        //     std::cout << "[ ";
        //     for (int l=0; l<track_state.size(); ++l) {
        //         std::cout << track_state[l] << " ";
        //     }
        //     std::cout << "]\n";
        // }

		// for(auto& tr:tracks_){
		// 	Eigen::VectorXd x = tr->GetState();
		// 	Eigen::Vector2f p = tr->GetMeasure();
		// 	int id = tr->GetId();
		// 	// Eigen::VectorXd save(11);//id, fx, fy, angle, mx, my, yaw, l, w, h, z
		// 	Eigen::VectorXd save(7);//id, fx, fy, angle, mx, my, yaw 
		// 	// Box tempb = tr->GetBox();
		// 	// save<<id,x(0),x(1),x(2), p(0), p(1), tempb.yaw,tempb.length,tempb.width,tempb.height,tempb.z;
		// 	save<<id,x(0),x(1),x(2),x(3), p(0), p(1);
		// 	if(tr->Age()<3)
		// 		result.push_back(save);
		// }


		if(unconfirmed_tracks_.size()>0){
			Detection seconda_sso_detections;

			if(not_associated_.size()==0){
				for(const auto& track:unconfirmed_tracks_){
					track->MarkMissed();
				}
			}
            else {
                for(auto det:not_associated_){
                    seconda_sso_detections.push_back(det);
                }

                not_associated_.clear();
                not_associated_.swap(not_associated_);

                for(const auto& track:unconfirmed_tracks_){
                    track->Prediction(time);
                }

                const uint& UconfirmTrackSize = unconfirmed_tracks_.size();//这里指的上一时刻那些没有匹配或是没有形成track的目标
                const uint& detSize = seconda_sso_detections.size(); //当前时刻没有匹配的目标

                cv::Mat assigmentsBin = cv::Mat::zeros(cv::Size(detSize, UconfirmTrackSize), CV_32SC1);
                cv::Mat costMat = cv::Mat(cv::Size(detSize, UconfirmTrackSize), CV_32FC1);//cosmatrix (cols rows)
                // cv::Mat IoucostMat = cv::Mat(cv::Size(detSize, UconfirmTrackSize), CV_32FC1);//cosmatrix (cols rows)

                std::vector<int> assignments;
                std::vector<float> costs(detSize * UconfirmTrackSize);
                // std::vector<float> ioucosts(detSize * UconfirmTrackSize);

                for(uint i = 0; i < UconfirmTrackSize; ++i){
                    for(uint j = 0; j < detSize; ++j){
                        costs.at(i + j * UconfirmTrackSize ) = euclideanDist(seconda_sso_detections[j].position,
                            unconfirmed_tracks_[i]->GetZ());
                        //ioucosts.at(i + j * prevDetSize ) = RectIou(not_associated_.at(j).rotbox, prev_detections_.at(i).rotbox);
                        costMat.at<float>(i, j) = costs.at(i + j * UconfirmTrackSize );
                    }
                }

                AssignmentProblemSolver APS;//匈牙利算法
                APS.Solve(costs, UconfirmTrackSize, detSize, assignments, AssignmentProblemSolver::optimal);

                const uint& assSize = assignments.size();//这个的大小应该是检测结果的大小，里边对应的是目标的编号
                for(uint i = 0; i < assSize; ++i){
                    if( assignments[i] != -1 && costMat.at<float>(i, assignments[i]) < param_.pdist_thresh){
                        assigmentsBin.at<int>(i, assignments[i]) = 1;
                    }
                }
                const uint& rows = assigmentsBin.rows;
                const uint& cols = assigmentsBin.cols;

                std::vector<bool> choosen(detSize,false);
                std::vector<bool> trackst(rows,false);
                std::vector<bool> detst(cols,false);
                for(uint i = 0; i < rows; ++i){
                    for(uint j = 0; j < cols; ++j){
                        if(assigmentsBin.at<int>(i, j)){
                            unconfirmed_tracks_[i]->Update(seconda_sso_detections[j].position);//TODO change the imm_ukf
                            // unconfirmed_tracks_[i]->UpdateBox(seconda_sso_detections[j]);
                            unconfirmed_tracks_[i]->UpdateMeasure(seconda_sso_detections[j].position(0),seconda_sso_detections[j].position(1));
                            trackst[i] = true;
                            detst[j] = true;
                        }
                    }
                }

                for(uint j = 0; j < cols; ++j){
                    if(!detst[j]){
                        not_associated_.push_back(seconda_sso_detections[j]);
                    }
                }
                for(uint i=0 ; i<rows; ++i){
                    if(!trackst[i]){
                        unconfirmed_tracks_[i]->MarkMissed();
                    }
                }
            }
		}

		// for(auto& track:unconfirmed_tracks_){
		// 	tracks_.push_back(track);
		// 	Eigen::VectorXd x = track->GetState();
		// 	Eigen::Vector2f p = track->GetMeasure();
		// 	int id = track->GetId();
		// 	Eigen::VectorXd save(7);
		// 	// Box tempb = track->GetBox();
		// 	save<<id,x(0),x(1), x(2),x(3), p(0), p(1);
		// 	if(track->Age()<=1)
		// 		result.push_back(save);
		// }

		confirmed_tracks_.clear();
		confirmed_tracks_.swap(confirmed_tracks_);
		unconfirmed_tracks_.clear();
		unconfirmed_tracks_.swap(unconfirmed_tracks_);

		delete_tracks();
		manage_tracks(time);

		pretime = time;
	}
}

		
void Tracker::trackRadar(const Detection& detections,float& time, std::vector<Eigen::VectorXd>& result){
	
	if(!init_ || !start_tracking_){
        return;
	}else{	//开始跟踪

		not_associated_.clear();
		not_associated_.swap(not_associated_);

		for (auto& track:tracks_){
			if(track->GetTrackState()==Track_state_.Confirmed){
				confirmed_tracks_.push_back(track);
			}else{
				unconfirmed_tracks_.push_back(track);
			}
		}

		// tracks_.clear();
		// tracks_.swap(tracks_);

		std::vector<bool> MeaisAsso(detections.size(),false);

		cv::Mat_<int> q(cv::Size(confirmed_tracks_.size(), detections.size()), int(0));//基本关联矩阵q
		
        Detection selected_detections;//关联上的量测
		for(const auto& track:confirmed_tracks_){
			track->Prediction(time);
		}

		std::vector<track_ptr> prun_tracks;
        associate(selected_detections, q, detections);

		if(q.total()==0){
			for(auto& track:confirmed_tracks_){
				track->MarkMissed();
			}
		}else{
			std::vector<std::vector<int> > cltrack;
			std::vector<std::vector<int> > cldet;
			std::vector<bool> missed_tracks(confirmed_tracks_.size(),true);

            // This is to cluster all the connected (joint) tracks (cltrack) and detections (cldet) for the JPDA computations. By doing so, we can effectively save computation time.
			clusterq(q, cltrack, cldet);
			int clustersize = cltrack.size();
			for(int i =0; i<clustersize; ++i){
				int colsize = cltrack[i].size();
				int rowsize = cldet[i].size();

				cv::Mat x = cv::Mat_<int>(cv::Size(colsize + 1, rowsize), int(1));

				for(int row=0; row<rowsize; ++row){
					int rowinq = cldet[i][row];
					for(int col=1; col<x.cols; ++col){
						x.at<int>(row,col) = q.at<int>(rowinq, cltrack[i][col-1]);
					}
				}
				std::vector<track_ptr> tracks;
				for(auto trackid:cltrack[i]){
					missed_tracks[trackid-1] = false;
					tracks.push_back(confirmed_tracks_[trackid-1]);
				}
				Detection selectdets;
				for(auto detid:cldet[i]){
					selectdets.push_back(selected_detections[detid]);
				}
				const Matrices& association_matrices = generate_hypothesis(x);//生成假设矩阵

				Eigen::MatrixXd beta = joint_probability(association_matrices, selectdets, tracks);//JPDAF
				int b=0;
				for(const auto& track:tracks){
					std::vector<Eigen::VectorXd> Zv;
					for(auto det : selectdets){
						Zv.push_back(det.position);
					}
					track->Update(Zv, beta.col(b), beta(beta.rows() - 1, b), time);//TODO change the imm_ukf
					b++;
					// tracks_.push_back(track);
					prun_tracks.push_back(track);
				}
			}

			// for(int i=0; i<confirmed_tracks_.size(); ++i){
			// 	if(missed_tracks[i]){
			// 		confirmed_tracks_[i]->MarkMissed();
			// 		tracks_.push_back(confirmed_tracks_[i]);
			// 	}
			// }
		}

		std::vector<int> final_select;
		pruning(selected_detections,final_select,prun_tracks);//TODO 剪枝

        /* ------------------- debug ---------------- */
        // std::cout << "[INFO] number of confirmed tracks: " << confirmed_tracks_.size() << std::endl;
        // std::cout << "[INFO] number of tracks: " << tracks_.size() << std::endl;
        // for (auto& track : tracks_) {
        //     std::cout << "id " << track->GetId() << " "; 
        //     auto track_state = track->GetState();
        //     std::cout << "[ ";
        //     for (int l=0; l<track_state.size(); ++l) {
        //         std::cout << track_state[l] << " ";
        //     }
        //     std::cout << "]\n";
        // }

		for(auto& tr:tracks_){
			Eigen::VectorXd x = tr->GetState();
			Eigen::Vector2f p = tr->GetMeasure();
			int id = tr->GetId();
			Eigen::VectorXd save(7);//id, fx, fy, angle, mx, my, yaw 
			save<<id,x(0),x(1),x(2),x(3), p(0), p(1);
			// if(tr->Age()<3)
				result.push_back(save);
		}


		if(unconfirmed_tracks_.size()>0){
			Detection seconda_sso_detections;

			if(not_associated_.size()==0){
				for(const auto& track:unconfirmed_tracks_){
					track->MarkMissed();
				}
			}
            else {
                for(auto det:not_associated_){
                    seconda_sso_detections.push_back(det);
                }

                not_associated_.clear();
                not_associated_.swap(not_associated_);

                for(const auto& track:unconfirmed_tracks_){
                    track->Prediction(time);
                }

                const uint& UconfirmTrackSize = unconfirmed_tracks_.size();//这里指的上一时刻那些没有匹配或是没有形成track的目标
                const uint& detSize = seconda_sso_detections.size(); //当前时刻没有匹配的目标

                cv::Mat assigmentsBin = cv::Mat::zeros(cv::Size(detSize, UconfirmTrackSize), CV_32SC1);
                cv::Mat costMat = cv::Mat(cv::Size(detSize, UconfirmTrackSize), CV_32FC1);//cosmatrix (cols rows)

                std::vector<int> assignments;
                std::vector<float> costs(detSize * UconfirmTrackSize);

                for(uint i = 0; i < UconfirmTrackSize; ++i){
                    for(uint j = 0; j < detSize; ++j){
                        costs.at(i + j * UconfirmTrackSize ) = euclideanDist(seconda_sso_detections[j].position,
                            unconfirmed_tracks_[i]->GetZ());
                        costMat.at<float>(i, j) = costs.at(i + j * UconfirmTrackSize );
                    }
                }

                AssignmentProblemSolver APS;//匈牙利算法
                APS.Solve(costs, UconfirmTrackSize, detSize, assignments, AssignmentProblemSolver::optimal);

                const uint& assSize = assignments.size();//这个的大小应该是检测结果的大小，里边对应的是目标的编号
                for(uint i = 0; i < assSize; ++i){
                    if( assignments[i] != -1 && costMat.at<float>(i, assignments[i]) < param_.pdist_thresh){
                        assigmentsBin.at<int>(i, assignments[i]) = 1;
                    }
                }
                const uint& rows = assigmentsBin.rows;
                const uint& cols = assigmentsBin.cols;

                std::vector<bool> choosen(detSize,false);
                std::vector<bool> trackst(rows,false);
                std::vector<bool> detst(cols,false);
                for(uint i = 0; i < rows; ++i){
                    for(uint j = 0; j < cols; ++j){
                        if(assigmentsBin.at<int>(i, j)){
                            unconfirmed_tracks_[i]->Update(seconda_sso_detections[j].position);//TODO change the imm_ukf
                            // unconfirmed_tracks_[i]->UpdateBox(seconda_sso_detections[j]);
                            unconfirmed_tracks_[i]->UpdateMeasure(seconda_sso_detections[j].position(0),seconda_sso_detections[j].position(1));
                            trackst[i] = true;
                            detst[j] = true;
                        }
                    }
                }

                for(uint j = 0; j < cols; ++j){
                    if(!detst[j]){
                        not_associated_.push_back(seconda_sso_detections[j]);
                    }
                }
                for(uint i=0 ; i<rows; ++i){
                    if(!trackst[i]){
                        unconfirmed_tracks_[i]->MarkMissed();
                    }
                }
            }
		}

		// for(auto& track:unconfirmed_tracks_){
		// 	tracks_.push_back(track);
		// 	Eigen::VectorXd x = track->GetState();
		// 	Eigen::Vector2f p = track->GetMeasure();
		// 	int id = track->GetId();
		// 	Eigen::VectorXd save(7);
		// 	save<<id,x(0),x(1), x(2),x(3), p(0), p(1);
		// 	if(track->Age()<=1)
		// 		result.push_back(save);
		// }

		confirmed_tracks_.clear();
		confirmed_tracks_.swap(confirmed_tracks_);
		unconfirmed_tracks_.clear();
		unconfirmed_tracks_.swap(unconfirmed_tracks_);

		delete_tracks();
		pretime = time;
	}
}




//计算关联矩阵
void Tracker::associate(Detection& _selected_detections, cv::Mat& _q, 
			const Detection& _detections)
{	

	//Extracting the measurements inside the validation gate for all the tracks
  	//Create a q matrix with a width = clutter + number of tracks
  	_q = cv::Mat_<int>(cv::Size(confirmed_tracks_.size() + 1, _detections.size()), int(0));
  	uint validationIdx = 0;
  	not_associated_.clear();
  	uint j = 0;

  	for(const auto& detection : _detections){
  		uint i = 1;
    	bool found = false;

    	cv::Mat det_cv(cv::Size(2, 1), CV_64FC1);
    	det_cv.at<double>(0) = detection.position(0);
    	det_cv.at<double>(1) = detection.position(1);

    	for(auto& track : confirmed_tracks_){
      		const Eigen::VectorXd& tr = track->GetZ();//TODO GET STATE VECTOR
			
      		cv::Mat tr_cv(cv::Size(2, 1), CV_64FC1);
      		tr_cv.at<double>(0) = tr(0);
      		tr_cv.at<double>(1) = tr(1);
      		const float& Sdt = track->S().determinant();

			const Eigen::MatrixXd& Sin = track->S().inverse();//TODO GET MEASURE COVARIANCE
      		cv::Mat S_cv;
      		cv::eigen2cv(Sin, S_cv);
      		const double& mah = cv::Mahalanobis(tr_cv, det_cv, S_cv);
      		const float& eucl = euclideanDist(detection.position, tr);
            // remove the situation where there may appear nan Mahal Dist
			if(std::isnan(mah) || std::isnan(eucl) ||std::isnan((param_.pi * param_.pg_sigma * std::sqrt(fabs(Sdt))))) {
				std::abort();
            }
			//mah <= (param_.pi * param_.pg_sigma * std::sqrt(fabs(Sdt))) &&
            // TODO: tune the threshold values...
			if((eucl <= 1 && mah<chi2in975[2]) || eucl< 1.2){
			// if(eucl <= 3 && mah < 10){
    	  		_q.at<int>(validationIdx, 0) = 1;
    	  		_q.at<int>(validationIdx, i) = 1;
    	  		found = true;
      		}
            ++i;
    	}

    	if(found){
      		_selected_detections.push_back(detection);//有关联的检测
      		validationIdx++;//实际的关联矩阵的列数，也就是实际detection的数量
    	}else{
      		not_associated_.push_back(detection);
    	}
   		++j;
  	}

  	_q = _q(cv::Rect(0, 0, confirmed_tracks_.size() + 1, validationIdx));
}

//分析关联后哪些track没有关联上
std::vector<bool> Tracker::analyze_tracks(const cv::Mat& _q){
	const cv::Mat& m_q = _q(cv::Rect(1, 0, _q.cols - 1, _q.rows));
  	cv::Mat col_sum(cv::Size(m_q.cols, 1), _q.type(), cv::Scalar(0));

	std::vector<bool> not_associate(m_q.cols, true); //ALL TRACKS ARE ASSOCIATED
  	for(uint i = 0; i < m_q.rows; ++i){
	    col_sum += m_q.row(i);
  	}
  	cv::Mat nonZero;
  	col_sum.convertTo(col_sum, CV_8UC1);
  
  
  	cv::Mat zero = col_sum == 0;
  	cv::Mat zeroValues;
  	cv::findNonZero(zero, zeroValues);
  
  	for(uint i = 0; i < zeroValues.total(); ++i){
    		not_associate.at(zeroValues.at<cv::Point>(i).x) = false;
  	}   
    return not_associate;
}

//生成互联事件
Tracker::Matrices Tracker::generate_hypothesis(const cv::Mat& _q)
{
	uint validationIdx = _q.rows;
  	//All the measurements can be generated by the clutter track
  	Eigen::MatrixXd A_Matrix(_q.rows, _q.cols);
  	A_Matrix = Eigen::MatrixXd::Zero(_q.rows, _q.cols);
  	A_Matrix.col(0).setOnes();
  	Matrices tmp_association_matrices(MAX_ASSOC, A_Matrix);
  
  	uint hyp_num = 0;
  	//Generating all the possible association matrices from the possible measurements
    
  	if(validationIdx != 0){
    		for(uint i = 0; i < _q.rows; ++i){
      			for(uint j = 1; j < _q.cols; ++j){
    	  			if(_q.at<int>(i, j)){
    		  			tmp_association_matrices.at(hyp_num)(i, 0) = 0;
    		  			tmp_association_matrices.at(hyp_num)(i, j) = 1;
    		  			++hyp_num;
    		 			if ( j == _q.cols - 1 ) continue;
    		  			for(uint l = 0; l < _q.rows; ++l){
    			 			 if(l != i){
    				 			for(uint m = j + 1; m < _q.cols; ++m) {// CHECK Q.COLS - 1
    					  			if(_q.at<int>(l, m)){
    						  			tmp_association_matrices.at(hyp_num)(i, 0) = 0;
    						  			tmp_association_matrices.at(hyp_num)(i, j) = 1;
    						  			tmp_association_matrices.at(hyp_num)(l, 0) = 0;
    						  			tmp_association_matrices.at(hyp_num)(l, m) = 1;
    						  			++hyp_num;
    					  			}//if(q.at<int>(l, m))
    				  			}//m
    			  			} // if l != i
    		  			} //l
    	  			} //if q(i, j) == 1
      			} //j
    		} //i
  	} 
  Matrices association_matrices(hyp_num + 1);
  std::copy(tmp_association_matrices.begin(), tmp_association_matrices.begin() + hyp_num + 1, 
	    association_matrices.begin());

  return association_matrices;
}

//计算beta
Eigen::MatrixXd Tracker::joint_probability(const Matrices& _association_matrices,
						const Detection& selected_detections,
						std::vector<track_ptr>& tracks_in)//TODO CHANGE THE FORM OF DETECTION
{


	uint hyp_num = _association_matrices.size();
  	Eigen::VectorXd Pr(_association_matrices.size());
  	uint validationIdx = _association_matrices.at(0).rows();
  	uint tracksize = tracks_in.size();
  	float prior;

  	//Compute the total volume
  	float V = 0.;
  	for(const auto& track : tracks_in){
    		V += M_PI * param_.pg_sigma * std::sqrt(track->S().determinant());
  	}

  	for(uint i = 0; i < hyp_num; ++i){
    		//I assume that all the measurments can be false alarms
    		int false_alarms = validationIdx ;
    		float N = 1.;
    		//For each measurement j: I compute the measurement indicator ( tau(j, X) )
    		// and the target detection indicator ( lambda(t, X) )
    		for(uint j = 0; j < validationIdx; ++j){
      			//Compute the MEASURAMENT ASSOCIATION INDICATOR
      			const Eigen::MatrixXd& A_matrix = _association_matrices.at(i).block(j, 1, 1, tracksize);
      			const int& mea_indicator = A_matrix.sum();

      			if(mea_indicator == 1){
    	  			//Update the total number of wrong measurements in X
    	  			--false_alarms;
    	  			//Detect which track is associated to the measurement j
    	  			//and compute the probability
    	  			for(uint notZero = 0; notZero < tracksize; ++notZero){
    		  			if(A_matrix(0, notZero) == 1){
    			  			const Eigen::VectorXd& z_predict = tracks_in.at(notZero)->GetZ();//TODO!!!!
    			  			const Eigen::MatrixXd& S = tracks_in.at(notZero)->S();//TODO!!!!
    			  			const Eigen::VectorXd& diff = selected_detections.at(j).position - z_predict;
    			  			cv::Mat S_cv;
    			  			cv::eigen2cv(S, S_cv);
    			  			//const float& b = diff.transpose() * S.inverse() * diff;
    			  			cv::Mat z_cv(cv::Size(2, 1), CV_64FC1);
    			  			cv::Mat det_cv(cv::Size(2, 1), CV_64FC1);
    			  			z_cv.at<double>(0) = z_predict(0);
    			  			z_cv.at<double>(1) = z_predict(1);
    			  			det_cv.at<double>(0) = selected_detections.at(j).position(0);
    			  			det_cv.at<double>(1) = selected_detections.at(j).position(1);
    			  			const double& b = cv::Mahalanobis(z_cv, det_cv, S_cv.inv());
    			 			N = N / sqrt((2*CV_PI*S).determinant())*exp(-b);//正态分布
    		  			}
    	  			}
      			}

    		}

    		const float& likelyhood = N / float(std::pow(V, false_alarms));

    		if(param_.pd == 1){//pd代表检测概率，是一个设定值
    			prior = 1.;
    		}
    		else{
      			//Compute the TARGET ASSOCIATION INDICATOR
    			prior = 1.;
      			for(uint j = 0; j < tracksize; ++j){
    				const Eigen::MatrixXd& target_matrix = _association_matrices.at(i).col(j+1);
    				const int& target_indicator = target_matrix.sum();
    				prior = prior * std::pow(param_.pd, target_indicator) * std::pow((1 - param_.pd), (1 - target_indicator));//二项分布
      			}
    		}

    		//Compute the number of events in X for which the same target
    		//set has been detected
    		int a = 1;
    		for(int j = 1; j <= false_alarms; ++j){
    			a = a * j;
    		}

    		Pr(i) = a * likelyhood * prior;
		    //std::cout<<Pr(i)<<std::endl;
  	}

  	const float& prSum = Pr.sum();

  	if(prSum != 0.)
    	Pr = Pr / prSum; //normalization

  	//Compute Beta Coefficients
  	Eigen::MatrixXd beta(validationIdx + 1, tracksize);
  	beta = Eigen::MatrixXd::Zero(validationIdx + 1, tracksize);


 	Eigen::VectorXd sumBeta(tracksize);
  	sumBeta.setZero();

  	for(uint i = 0; i < tracksize; ++i){
    		for(uint j = 0; j < validationIdx; ++j){
      			for(uint k = 0; k < hyp_num; ++k){
    	  			beta(j, i) = beta(j, i) + Pr(k) * _association_matrices.at(k)(j, i+1);
      			}
      			sumBeta(i) += beta(j, i);
    		}
    		sumBeta(i) = 1 - sumBeta(i);
  	}

  	beta.row(validationIdx) = sumBeta;
    return beta;
}

/**
 * @brief   calculate beta
 * @details https://github.com/apennisi/jpdaf_tracking
 * @param  
 */
Eigen::MatrixXd Tracker::joint_probability(const Matrices& _association_matrices,
						const Detection& selected_detections)//TODO CHANGE THE FORM OF DETECTION
{


	uint hyp_num = _association_matrices.size();
  	Eigen::VectorXd Pr(_association_matrices.size());
  	uint validationIdx = _association_matrices.at(0).rows();
  	uint tracksize = confirmed_tracks_.size();
  	float prior;
  
  	//Compute the total volume
  	float V = 0.;
  	for(const auto& track : confirmed_tracks_){
    		V += M_PI * param_.pg_sigma * std::sqrt(track->S().determinant());
  	}

  	for(uint i = 0; i < hyp_num; ++i){
    		//I assume that all the measurments can be false alarms
    		int false_alarms = validationIdx ;
    		float N = 1.;
    		//For each measurement j: I compute the measurement indicator ( tau(j, X) ) 
    		// and the target detection indicator ( lambda(t, X) ) 
    		for(uint j = 0; j < validationIdx; ++j){
      			//Compute the MEASURAMENT ASSOCIATION INDICATOR      
      			const Eigen::MatrixXd& A_matrix = _association_matrices.at(i).block(j, 1, 1, tracksize);
      			const int& mea_indicator = A_matrix.sum();     
      
      			if(mea_indicator == 1){
    	  			//Update the total number of wrong measurements in X
    	  			--false_alarms;
    	  			//Detect which track is associated to the measurement j
    	  			//and compute the probability
    	  			for(uint notZero = 0; notZero < tracksize; ++notZero){
    		  			if(A_matrix(0, notZero) == 1){
    			  			const Eigen::VectorXd& z_predict = confirmed_tracks_.at(notZero)->GetZ();//TODO!!!!
    			  			const Eigen::MatrixXd& S = confirmed_tracks_.at(notZero)->S();//TODO!!!!
    			  			const Eigen::VectorXd& diff = selected_detections.at(j).position - z_predict;
    			  			cv::Mat S_cv;
    			  			cv::eigen2cv(S, S_cv);
    			  			//const float& b = diff.transpose() * S.inverse() * diff;
    			  			cv::Mat z_cv(cv::Size(2, 1), CV_64FC1);
    			  			cv::Mat det_cv(cv::Size(2, 1), CV_64FC1);
    			  			z_cv.at<double>(0) = z_predict(0);
    			  			z_cv.at<double>(1) = z_predict(1);
    			  			det_cv.at<double>(0) = selected_detections.at(j).position(0);
    			  			det_cv.at<double>(1) = selected_detections.at(j).position(1);
    			  			const double& b = cv::Mahalanobis(z_cv, det_cv, S_cv.inv());
    			 			N = N / sqrt((2*CV_PI*S).determinant())*exp(-b);//正态分布
    		  			}
    	  			}
      			}
      
    		}
    
    		const float& likelyhood = N / float(std::pow(V, false_alarms));
       
    		if(param_.pd == 1){//pd代表检测概率，是一个设定值
    			prior = 1.;
    		}
    		else{
      			//Compute the TARGET ASSOCIATION INDICATOR
    			prior = 1.;
      			for(uint j = 0; j < tracksize; ++j){
    				const Eigen::MatrixXd& target_matrix = _association_matrices.at(i).col(j+1);
    				const int& target_indicator = target_matrix.sum();
    				prior = prior * std::pow(param_.pd, target_indicator) * std::pow((1 - param_.pd), (1 - target_indicator));//二项分布
      			}
    		}
    
    		//Compute the number of events in X for which the same target 
    		//set has been detected
    		int a = 1;
    		for(int j = 1; j <= false_alarms; ++j){
    			a = a * j;
    		}
    
    		Pr(i) = a * likelyhood * prior;
  	}
  
  	const float& prSum = Pr.sum();
  
  	if(prSum != 0.)
    	Pr = Pr / prSum; //normalization
    
  	//Compute Beta Coefficients
  	Eigen::MatrixXd beta(validationIdx + 1, tracksize);
  	beta = Eigen::MatrixXd::Zero(validationIdx + 1, tracksize);
  
   
 	Eigen::VectorXd sumBeta(tracksize);
  	sumBeta.setZero();

  	for(uint i = 0; i < tracksize; ++i){
    		for(uint j = 0; j < validationIdx; ++j){
      			for(uint k = 0; k < hyp_num; ++k){
    	  			beta(j, i) = beta(j, i) + Pr(k) * _association_matrices.at(k)(j, i+1);
      			}
      			sumBeta(i) += beta(j, i);
    		}
    		sumBeta(i) = 1 - sumBeta(i);
  	}
 
  	beta.row(validationIdx) = sumBeta;
    return beta;
}


/**
 * @brief   manage tracks
 * @details https://github.com/apennisi/jpdaf_tracking
 * @param   time
 */
void Tracker::manage_tracks(float& time){
	const uint& prevDetSize = prev_detections_.size();
	const uint& deteSize = not_associated_.size(); 
	
	if(prevDetSize == 0){
		prev_detections_ = not_associated_;

	}else if(deteSize == 0){
		prev_detections_.clear();
		prev_detections_.swap(prev_detections_);
	}else{

		cv::Mat assigmentsBin = cv::Mat::zeros(cv::Size(deteSize, prevDetSize), CV_32SC1);
    	cv::Mat costMat = cv::Mat(cv::Size(deteSize, prevDetSize), CV_32FC1);
    	// cv::Mat IoucostMat = cv::Mat(cv::Size(deteSize, prevDetSize), CV_32FC1);//cosmatrix

    	std::vector<int> assignments;
    	std::vector<float> costs(deteSize * prevDetSize);
    	// std::vector<float> ioucosts(deteSize * prevDetSize);

   		for(uint i = 0; i < prevDetSize; ++i){
     		for(uint j = 0; j < deteSize; ++j){
    	  		costs.at(i + j * prevDetSize ) = euclideanDist(not_associated_.at(j).position, prev_detections_.at(i).position);
    	  		// ioucosts.at(i + j * prevDetSize ) = RectIou(not_associated_.at(j).rotbox, prev_detections_.at(i).rotbox);
    	  		costMat.at<float>(i, j) = costs.at(i + j * prevDetSize );
    	  		// IoucostMat.at<float>(i, j) = ioucosts.at(i + j * prevDetSize );
      		}
    	}

    	AssignmentProblemSolver APS;//匈牙利算法
    	APS.Solve(costs, prevDetSize, deteSize, assignments, AssignmentProblemSolver::optimal);

    	const uint& assSize = assignments.size();
    	for(uint i = 0; i < assSize; ++i){
      		if( assignments[i] != -1 && (costMat.at<float>(i, assignments[i]) < param_.pdist_thresh ) ) {
      				// || IoucostMat.at<float>(i, assignments[i]) < 0.5) ){
    	  		assigmentsBin.at<int>(i, assignments[i]) = 1;
      		}
    	}
    
    	const uint& rows = assigmentsBin.rows;
    	const uint& cols = assigmentsBin.cols;

    	for(uint i = 0; i < rows; ++i){
    		for(uint j = 0; j < cols; ++j){
    			if(assigmentsBin.at<int>(i, j)){
                    /************* TODO: use more sensor data **************/
    				const float& vx = (not_associated_.at(j).position(0) - prev_detections_.at(i).position(0))/(float)(time-pretime);
    				const float& vy = (not_associated_.at(j).position(1) - prev_detections_.at(i).position(1))/(float)(time-pretime);
					float velo = std::sqrt(vx*vx + vy*vy);
					float angle = atan2(vy,vx);
    				std::shared_ptr<Track> tr(new Track(param_, track_id_, time, not_associated_.at(j), velo, angle));
    				track_id_++;
    				tracks_.push_back(tr);
    			}
    		}
    	}

    	cv::Mat notAssignedDet(cv::Size(assigmentsBin.cols, 1), CV_32SC1, cv::Scalar(0));
    	for(uint i = 0; i < assigmentsBin.rows; ++i){
      		notAssignedDet += assigmentsBin.row(i);
    	}
    
    	notAssignedDet.convertTo(notAssignedDet, CV_8UC1);
    	notAssignedDet = notAssignedDet == 0;//反转 使未配对的检测标志为1
    
    	cv::Mat dets;
    	cv::findNonZero(notAssignedDet, dets);
    	prev_detections_.clear();
    	for(uint i = 0; i < dets.total(); ++i){
      		const uint& idx = dets.at<cv::Point>(i).x;
      		prev_detections_.push_back(not_associated_.at(i));//把当前帧未匹配的检测存起来留至下一帧匹配
    	}
  	}
}

void Tracker::delete_tracks(){	
	for(int i= tracks_.size()-1; i>=0; --i){
		if(tracks_[i]->GetTrackState() == Track_state_.Delete){//TODO
			tracks_.erase(tracks_.begin()+i);
		}
	}
}

/**
 * @brief   pruning tracks
 * @details 
 * @param   selected_detections
 *	    final_select: final choosed detection
 *          tacks_in
 */
void Tracker::pruning(Detection&  selected_detections,
						std::vector<int>& final_select,
						std::vector<track_ptr>& tacks_in){//TODO
	
	const uint& TrackSize = tacks_in.size();
	const uint& detSize = selected_detections.size();

	final_select.resize(detSize);

	cv::Mat assigmentsBin = cv::Mat::zeros(cv::Size(detSize, TrackSize), CV_32SC1);
	cv::Mat costMat = cv::Mat(cv::Size(detSize, TrackSize), CV_32FC1);//cosmatrix (cols rows)
	std::vector<int> assignments;
	std::vector<float> costs(detSize * TrackSize);

	for(uint i = 0; i < TrackSize; ++i){
 		for(uint j = 0; j < detSize; ++j){
	  		costs.at(i + j * TrackSize ) = euclideanDist(selected_detections[j].position,  tracks_[i]->GetState());
	  		costMat.at<float>(i, j) = costs.at(i + j * TrackSize );
  		}
	}
	AssignmentProblemSolver APS;//匈牙利算法
	APS.Solve(costs, TrackSize, detSize, assignments, AssignmentProblemSolver::optimal);

	const uint& assSize = assignments.size();//这个的大小应该是检测结果的大小，里边对应的是目标的编号
	for(uint i = 0; i < assSize; ++i){
        // TODO: tune the threshold value
  		if( assignments[i] != -1 && costMat.at<float>(i, assignments[i]) < 1.2){
	  		assigmentsBin.at<int>(i, assignments[i]) = 1;
  		}
	}
	const uint& rows = assigmentsBin.rows;
	const uint& cols = assigmentsBin.cols;

	std::vector<bool> choosen(detSize,false);
	std::vector<bool> trackchoosen(TrackSize,false);


	for(uint i = 0; i < rows; ++i){
		for(uint j = 0; j < cols; ++j){
			if(assigmentsBin.at<int>(i, j)){
				final_select[j] = tacks_in[i]->GetId();
				trackchoosen[i] = true;
				// tracks_[i]->UpdateBox(selected_detections[j]);
				tracks_[i]->UpdateMeasure(selected_detections[j].position(0),selected_detections[j].position(1));
				choosen[j] = true;
			}
		}
	}

	for(int i=0; i<choosen.size(); ++i){
		if(!choosen[i]){
			not_associated_.push_back(selected_detections[i]);
		}
	}

	for(int i=0; i<trackchoosen.size(); ++i){
		if(!trackchoosen[i]){
			tracks_[i]->MarkMissed();
		}
	}

}
