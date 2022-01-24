#include "tracker.h"


Tracker::Tracker(const Param& param) 
    : param_(param) {
    initialized_ = false;
    start_tracking_ = false;
}


Eigen::MatrixXd Tracker::jointProbability(const std::vector<Eigen::MatrixXd>& association_matrices, const std::vector<Detection>& selected_detections) {
    uint hyp_num = association_matrices.size();
    Eigen::VectorXd Pr(association_matrices.size());
    uint validationIdx = association_matrices.at(0).rows();
    uint tracksize = tracks_.size();
    double prior;

    
    //Compute the total volume
    double V = 0.;
    for(const auto& track : tracks_) {
        V += track->getEllipseVolume();

	}

    for(uint i = 0; i < hyp_num; ++i) {
        //I assume that all the measurments can be false alarms
        int false_alarms = validationIdx ;
        double N = 1.;
        //For each measurement j: I compute the measurement indicator ( tau(j, X) ) 
        // and the target detection indicator ( lambda(t, X) ) 
        for(uint j = 0; j < validationIdx; ++j)
        {
            //Compute the MEASURAMENT ASSOCIATION INDICATOR      
            const Eigen::MatrixXd& A_matrix = association_matrices.at(i).block(j, 1, 1, tracksize);
            const int& mea_indicator = A_matrix.sum();     
            
            if(mea_indicator == 1) {
                //Update the total number of wrong measurements in X
                --false_alarms;
                
                //Detect which track is associated to the measurement j 
                //and compute the probability
                for(uint notZero = 0; notZero < tracksize; ++notZero) {
                    if(A_matrix(0, notZero) == 1) {
                        const Eigen::VectorXd& z_predict = tracks_.at(notZero)->getMeasurementPred();
                        const Eigen::MatrixXd& S = tracks_.at(notZero)->getS();
                        const Eigen::VectorXd& diff = selected_detections.at(j).getVector() - z_predict;
                        cv::Mat S_cv;
                        cv::eigen2cv(S, S_cv);

						/* NOTE: adapt to the shape of sensor detection */
						cv::Mat z_cv, det_cv;
						cv::eigen2cv(z_predict, z_cv);
						cv::eigen2cv(selected_detections.at(j).getVector(), det_cv);
						z_cv = z_cv.t();
						det_cv = det_cv.t();
						/* NOTE: original implementation */
                        //cv::Mat z_cv(cv::Size(2, 1), CV_32FC1);
                        //cv::Mat det_cv(cv::Size(2, 1), CV_32FC1);
                        //z_cv.at<double>(0) = z_predict(0);
                        //z_cv.at<double>(1) = z_predict(1);
                        //det_cv.at<double>(0) = selected_detections.at(j).getVector()(0);
                        //det_cv.at<double>(1) = selected_detections.at(j).getVector()(1);
                        const double& b = cv::Mahalanobis(z_cv, det_cv, S_cv.inv());
                        N = N / sqrt((2*CV_PI*S).determinant())*exp(-b);
                    }
                }
            }
        }
        
		/* TODO: discuss whether it is necessary to add a 1e-5 */
        const double& likelyhood = N / (double(std::pow(V, false_alarms)) + 1e-5);

        std::cout << "[DEBUG INFO] Likelyhood: " << likelyhood << std::endl;
        
        if(param_.p_d == 1) {
            prior = 1.;
        }
        else {
            //Compute the TARGET ASSOCIATION INDICATOR
            prior = 1.;
            for(uint j = 0; j < tracksize; ++j) {
                const Eigen::MatrixXd& target_matrix = association_matrices.at(i).col(j+1);
                const int& target_indicator = target_matrix.sum();
                prior = prior * std::pow(param_.p_d, target_indicator) * std::pow((1 - param_.p_d), (1 - target_indicator));
            }
        }
        
        //Compute the number of events in X for which the same target 
        //set has been detected
        int a = 1;
        for(int j = 1; j <= false_alarms; ++j) {
            a = a * j;
        }
        
        Pr(i) = a * likelyhood * prior;
    }
    
    const double& prSum = Pr.sum();
    
    if(prSum != 0.)
        Pr = Pr / prSum; //normalization
        
    //Compute Beta Coefficients
    Eigen::MatrixXd beta(validationIdx + 1, tracksize);
    beta = Eigen::MatrixXd::Zero(validationIdx + 1, tracksize);
    
    
    Eigen::VectorXd sumBeta(tracksize);
    sumBeta.setZero();
    
    
    for(uint i = 0; i < tracksize; ++i)
    {
        for(uint j = 0; j < validationIdx; ++j)
        {
        for(uint k = 0; k < hyp_num; ++k)
        {
        beta(j, i) = beta(j, i) + Pr(k) * association_matrices.at(k)(j, i+1);
        }
        sumBeta(i) += beta(j, i);
        }
        sumBeta(i) = 1 - sumBeta(i);
    }
    
    
    beta.row(validationIdx) = sumBeta;

    return beta;
}


std::vector<Eigen::MatrixXd> Tracker::generateHypothesis(const std::vector<Detection>& selected_detections,  const cv::Mat& q)
{
    uint validationIdx = q.rows;
    //All the measurements can be generated by the clutter track
    Eigen::MatrixXd A_Matrix(q.rows, q.cols); 
    A_Matrix = Eigen::MatrixXd::Zero(q.rows, q.cols);
    A_Matrix.col(0).setOnes();
    std::vector<Eigen::MatrixXd> tmp_association_matrices(MAX_ASSOCIATIONS, A_Matrix);
    
    uint hyp_num = 0;

    //Generating all the possible association matrices from the possible measurements
    if(validationIdx != 0) {
        for(uint i = 0; i < q.rows; ++i) {
            for(uint j = 1; j < q.cols; ++j) {
                if(q.at<int>(i, j)) {
                    tmp_association_matrices.at(hyp_num)(i, 0) = 0;
                    tmp_association_matrices.at(hyp_num)(i, j) = 1;
                    ++hyp_num;
                    if ( j == q.cols - 1 ) continue;
                    for(uint l = 0; l < q.rows; ++l) {
                        if(l != i) {
                            for(uint m = j + 1; m < q.cols; ++m) {
                                if(q.at<int>(l, m)) {
                                    tmp_association_matrices.at(hyp_num)(i, 0) = 0;
                                    tmp_association_matrices.at(hyp_num)(i, j) = 1;
                                    tmp_association_matrices.at(hyp_num)(l, 0) = 0;
                                    tmp_association_matrices.at(hyp_num)(l, m) = 1;
                                    ++hyp_num;
                                } //if(q.at<int>(l, m))
                            } // m
                        } // if l != i
                    } // l
                } // if q(i, j) == 1
            } // j
        } // i
    } 
    std::vector<Eigen::MatrixXd> association_matrices(hyp_num + 1);
    std::copy(tmp_association_matrices.begin(), tmp_association_matrices.begin() + hyp_num + 1, 
            association_matrices.begin());
    return association_matrices;
}


std::vector<bool> Tracker::analyzeTracks(const cv::Mat& q, std::vector<Detection>& detections)
{
    const cv::Mat& m_q = q(cv::Rect(1, 0, q.cols - 1, q.rows));
    cv::Mat col_sum(cv::Size(m_q.cols, 1), q.type(), cv::Scalar(0));

    std::vector<bool> not_associate(m_q.cols, true); //ALL TRACKS ARE ASSOCIATED
    for(uint i = 0; i < m_q.rows; ++i)
    {
        col_sum += m_q.row(i);
    }
    cv::Mat nonZero;
    col_sum.convertTo(col_sum, CV_8UC1);
    
    // NOTE: in the matrix not_association, all associated tracks are set to 
    // true, and all not-associated tracks are set to false 
    cv::Mat zero = col_sum == 0;
    cv::Mat zeroValues;
    cv::findNonZero(zero, zeroValues);
    
    for(uint i = 0; i < zeroValues.total(); ++i)
    {
        not_associate.at(zeroValues.at<cv::Point>(i).x) = false;
    }   
    return not_associate;
}
