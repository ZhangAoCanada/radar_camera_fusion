#include <iostream>
#include "ukf.h"


UKF::UKF(const Param& param) {
    is_initialized_ = false;

    use_lidar_ = param.use_lidar;
    use_radar_ = param.use_radar;
    use_cam_ = param.use_camera;
    n_x_ = param.n_state;
    x_ = Eigen::VectorXd(n_x_);
    P_ = Eigen::MatrixXd::Identity(n_x_, n_x_);
    std_a_ = param.std_acceleration;
    std_yawdd_ = param.std_yaw_rate;
    std_lidpx_ = param.std_lidar_x; 
    std_lidpy_ = param.std_lidar_y;
    std_radpx_ = param.std_radar_x;
    std_radpy_ = param.std_radar_y;
    std_radv_ = param.std_radar_v;

    r_lidar_ = param.R_lidar;
    r_radar_ = param.R_radar;

    n_aug_ = param.n_state_aug;

    // Note that the column is augmented but the row is not.
    Xsig_pred_ = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);

    // Scaling parameter for unscented transformation.
    lambda_ = 3 - n_aug_;

    // Set weights
    weights_ = Eigen::VectorXd(2 * n_aug_ + 1);
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i=1; i < 2 * n_aug_ + 1; ++i) {
        weights_(i) = 0.5 / (lambda_ + n_aug_);
    }

}


UKF::~UKF() = default;


void UKF::process(Detection& detection) {
    // Initialization
    if (! is_initialized_) {

        if (detection.getSensorType() == SensorType::RADAR) {
        Eigen::VectorXd values = detection.getVector(); 
        const double p_x = values[0];
        const double p_y = values[1];
        const double p_v = values[2];
        x_ << p_x, p_y, p_v, 0.0, 0.0;

        } else if (detection.getSensorType() == SensorType::LIDAR) {
        Eigen::VectorXd values = detection.getVector(); 
        const double p_x = values[0];
        const double p_y = values[1];
        x_ << p_x, p_y, 0.0, 0.0, 0.0;

        } else if (detection.getSensorType() == SensorType::CAMERA) {
        Eigen::VectorXd values = detection.getVector(); 
        const double p_x = values[0];
        const double p_y = values[1];
        const double p_v = values[2];
        x_ << p_x, p_y, p_v, 0.0, 0.0;

        } else {
        std::cerr << "Unknown sensor_type: " << detection.getSensorType()
                    << std::endl;
        exit(EXIT_FAILURE);
        }

        time_us_ = detection.getTimestamp();

        Xsig_pred_ = generateSigmaPoints(x_, P_, n_aug_, lambda_);

        is_initialized_ = true;

        // TODO: to dicuss whether we can use the initialization detection for udpate.
		return;
    }

    const double dt = (detection.getTimestamp() - time_us_)/1000000.0;

    // Measurement update
    if (detection.getSensorType() == SensorType::RADAR && use_radar_) {
        updateRadar(detection, dt);
    } else if (detection.getSensorType() == SensorType::LIDAR && use_lidar_) {
        updateLidar(detection, dt);
    } else if (detection.getSensorType() == SensorType::CAMERA && use_cam_) {
        updateCamera(detection, dt);
    } else return;

    // The latest time update should be put here since the update may
    // be skipped, e.g. for the RADAR measurement.
    time_us_ = detection.getTimestamp();
}


void UKF::prediction(double delta_t) {
    //Augment the sigma points
    Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(n_aug_, 2 * n_aug_ + 1);
    Eigen::VectorXd x_aug = Eigen::VectorXd(n_aug_);
    Eigen::MatrixXd P_aug = Eigen::MatrixXd::Zero(n_aug_, n_aug_);

    //Set augmented mean state
    x_aug.head(5) = x_;
    x_aug(5) = 0.0;
    x_aug(6) = 0.0;

    //Set augmented covariance matrix
    P_aug.block<5, 5>(0, 0) = P_;
    P_aug(5, 5) = std_a_ * std_a_;
    P_aug(6, 6) = std_yawdd_ * std_yawdd_;

    //Calculate square root of Psig_aug
    Eigen::MatrixXd Psig_aug = P_aug.llt().matrixL();

    //Set augmented sigma points matrix
    Xsig_aug = generateSigmaPoints(x_aug, P_aug, n_aug_, lambda_);

    //Predict sigma points
    for (int i = 0; i< 2 * n_aug_ + 1; ++i) {
        const double p_x = Xsig_aug(0,i);
        const double p_y = Xsig_aug(1,i);
        const double v = Xsig_aug(2,i);
        const double yaw = Xsig_aug(3,i);
        const double yawd = Xsig_aug(4,i);
        const double nu_a = Xsig_aug(5,i);
        const double nu_yawdd = Xsig_aug(6,i);

        //Predicted state values
        double p_x_p, p_y_p;

        //Use different formulas for yawd != 0 and yawd == 0
        if (std::abs(yawd) > 0.001) {
        p_x_p = p_x + v / yawd * (std::sin(yaw + yawd * delta_t) - std::sin(yaw));
        p_y_p = p_y + v / yawd * (std::cos(yaw) - std::cos(yaw + yawd * delta_t));
        }
        else {
        p_x_p = p_x + v * delta_t * cos(yaw);
        p_y_p = p_y + v * delta_t * sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd * delta_t;
        double yawd_p = yawd;

        //Add noise
        double delta_t2 = delta_t * delta_t;
        p_x_p += 0.5 * nu_a * delta_t2 * std::cos(yaw);
        p_y_p += 0.5 * nu_a * delta_t2 * std::sin(yaw);
        v_p += nu_a * delta_t;
        yaw_p += 0.5 * nu_yawdd * delta_t2;
        yawd_p += nu_yawdd * delta_t;

        //Write predicted sigma point into right column
        Xsig_pred_(0, i) = p_x_p;
        Xsig_pred_(1, i) = p_y_p;
        Xsig_pred_(2, i) = v_p;
        Xsig_pred_(3, i) = yaw_p;
        Xsig_pred_(4, i) = yawd_p;
    }

    //Calculate state mean
    x_.fill(0.0);
    for (int i=0; i<2 * n_aug_ + 1; ++i) {
        x_ += weights_(i) * Xsig_pred_.col(i);
    }

    //Calculate state covariance matrix
    P_.fill(0.0);
    for (int i=0; i<2 * n_aug_+1; ++i) {
        Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;

        x_diff(3) = normalizeAngle(x_diff(3));

        P_ += weights_(i) * x_diff * x_diff.transpose();
    }
}


void UKF::updateLidar(Detection& detection, double delta_t) {

    prediction(delta_t);

    const int n_z = 2;
    Eigen::MatrixXd Zsig = Eigen::MatrixXd(n_z, 2 * n_aug_ + 1);

    //transform sigma points into measurement space
    for (int i=0; i < 2 * n_aug_ + 1; ++i) {
        Zsig(0, i) = Xsig_pred_(0, i);
        Zsig(1, i) = Xsig_pred_(1, i);
    }

    measurementUpdate(detection, Zsig, n_z);
}


void UKF::updateRadar(Detection& detection, double delta_t) {

    prediction(delta_t);

    const int n_z = 3;
    Eigen::MatrixXd Zsig = Eigen::MatrixXd(n_z, 2 * n_aug_ + 1);

    for (int i=0; i < 2 * n_aug_ + 1; ++i) {
        // NOTE: for Zsig(2, i), which is the velocity, is currently set to 0
        // TODO: for Zsig(2, i), careful define the noise standard deviation
        Zsig(0, i) = Xsig_pred_(0, i);
        Zsig(1, i) = Xsig_pred_(1, i);
        Zsig(2, i) = Xsig_pred_(2, i);
    }

    measurementUpdate(detection, Zsig, n_z);
}


void UKF::updateCamera(Detection& detection, double delta_t) {

    prediction(delta_t);

    const int n_z = 3;
    Eigen::MatrixXd Zsig = Eigen::MatrixXd(n_z, 2 * n_aug_ + 1);

    for (int i=0; i < 2 * n_aug_ + 1; ++i) {
        Zsig(0, i) = Xsig_pred_(0, i);
        Zsig(1, i) = Xsig_pred_(1, i);
        Zsig(2, i) = Xsig_pred_(2, i);
    }

    measurementUpdate(detection, Zsig, n_z);
}


void UKF::measurementUpdate(Detection& detection,
                            const Eigen::MatrixXd &Zsig,
                            int n_z) {
    // Eigen::VectorXd z_pred = Eigen::VectorXd(n_z);
    z_pred_ = Eigen::VectorXd(n_z);

    //calculate mean predicted measurement
    z_pred_.fill(0.0);
    for (int i=0; i< 2 * n_aug_ + 1; ++i) {
        z_pred_ += weights_(i) * Zsig.col(i);
    }

    //calculate measurement covariance matrix S
    // Eigen::MatrixXd S = Eigen::MatrixXd(n_z, n_z);
    S_ = Eigen::MatrixXd(n_z, n_z);

    S_.fill(0.0);
    for (int i=0; i< 2 * n_aug_ + 1; ++i) {
        Eigen::VectorXd z_diff = Zsig.col(i) - z_pred_;

        S_ += weights_(i) * z_diff * z_diff.transpose();
    }

    if (detection.getSensorType() == SensorType::RADAR) {
        S_ += r_radar_;
    } else if (detection.getSensorType() == SensorType::LIDAR){
        S_ += r_lidar_;
    }

    //calculate cross correlation matrix
    Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, n_z);

    Tc.fill(0.0);
    for (int i=0; i < 2 * n_aug_ + 1; i++) {
        //residual
        Eigen::VectorXd z_diff = Zsig.col(i) - z_pred_;

        // state difference
        Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;
        x_diff(3) = normalizeAngle(x_diff(3));

        Tc += weights_(i) * x_diff * z_diff.transpose();
    }

    //Kalman gain K;
    Eigen::MatrixXd Si = S_.inverse();
    Eigen::MatrixXd K = Tc * Si;

    //residual
    Eigen::VectorXd z_diff = detection.getVector() - z_pred_;

    //update state mean and covariance matrix
    x_ += K * z_diff;
    P_ -= K * S_ * K.transpose();

    // Calculate the normalized innovation squared()
    if (detection.getSensorType() == SensorType::RADAR) {
        nis_radar_ = z_diff.transpose() * Si * z_diff;
    } else if (detection.getSensorType() == SensorType::LIDAR){
        nis_lidar_ = z_diff.transpose() * Si * z_diff;
    }

}


Eigen::MatrixXd UKF::generateSigmaPoints(const Eigen::VectorXd &x,
                                         const Eigen::MatrixXd &P,
                                         int n_aug, double lambda) {

    long n = x.size();

    Eigen::MatrixXd A = P.llt().matrixL();
    Eigen::MatrixXd Xsig = Eigen::MatrixXd(n, 2 * n_aug + 1);

    Xsig.col(0) = x;
    const double c = std::sqrt(lambda + n);
    for (std::size_t i = 0; i < n; ++i) {
        Xsig.col(i + 1) = x + c * A.col(i);
        Xsig.col(i + n + 1) = x - c * A.col(i);
    }

    return Xsig;
}


inline double UKF::normalizeAngle(double phi) {
    double phi_norm = std::fmod(phi, 2*kPI);
    if (phi_norm <= -kPI) phi_norm += 2*kPI;
    if (phi_norm > kPI) phi_norm -= 2*kPI;

    return phi_norm;
}

