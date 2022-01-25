#ifndef UKF_H_
#define UKF_H_

#include <eigen3/Eigen/Dense>

#include "detection.h"
#include "param.h"


class UKF {
public:
    UKF(const Param& param);

    ~UKF();

    // processMeasurement
    // @param ms_pack The latest measurement data of either radar or laser
    void process(Detection& detection);

    // prediction Predicts sigma points, the state, and the state covariance matrix.
    // @param delta_t: Time step in s
    void prediction(double delta_t);

    // Updates the state and the state covariance matrix using a lidar measurement.
    // @param m_pkg: Measurement
    // @param delta_t: Time step in second
    void updateLidar(Detection& detection, double delta_t);

    // Updates the state and the state covariance matrix using a radar measurement.
    // @param m_pkg: Measurement
    // @param delta_t: Time step in second
    void updateRadar(Detection& detection, double delta_t);

    // Updates the state and the state covariance matrix using a camera measurement.
    // @param m_pkg: Measurement
    // @param delta_t: Time step in second
    void updateCamera(Detection& detection, double delta_t);

    const Eigen::VectorXd getState() const { return x_; }
    const Eigen::VectorXd getMeasurementPred() const { return z_pred_; }
    const double getNISRadar() const { return nis_radar_; }
    const double getNISLidar() const { return nis_lidar_; }
    const Eigen::MatrixXd getS() const { return S_; }
    const double getDeltaT(Detection& detection) const { 
        return (detection.getTimestamp() - time_us_) / 1000000.0; 
        }

private:

    bool is_initialized_; // initially set to false, set to true in first call of processMeasurement
    long long time_us_; // time when the state is true, in us
    bool use_lidar_; // if this is false, lidar measurements will be ignored (except for init)
    bool use_radar_; // if this is false, radar measurements will be ignored (except for init)
    bool use_cam_; // if this is false, camera measurements will be ignored (except for init)
    Eigen::MatrixXd P_; // state covariance matrix
    Eigen::MatrixXd S_; // measurement covariance matrix
    Eigen::MatrixXd Xsig_pred_; // predicted sigma points matrix
    double std_a_; // Process noise standard deviation longitudinal acceleration in m/s^2
    double std_yawdd_; // Process noise standard deviation yaw acceleration in rad/s^2
    double std_lidpx_; // Lidar measurement noise standard deviation position1 in m
    double std_lidpy_; // Lidar measurement noise standard deviation position2 in m
    double std_radpx_; // Radar measurement noise standard deviation radius in m
    double std_radpy_; // Radar measurement noise standard deviation angle in rad
    double std_radv_; // Radar measurement noise standard deviation radius change in m/s
    Eigen::MatrixXd r_lidar_; // Measurement covariance matrices for LIDAR and RADAR.
    Eigen::MatrixXd r_radar_;
    Eigen::VectorXd weights_; // Weights of sigma points
    int n_x_; // State dimension
    int n_aug_; // Augmented state dimension
    double lambda_; // Sigma point spreading parameter

    // parameters that can be accessed
    Eigen::VectorXd x_;// state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    Eigen::VectorXd z_pred_; // predicted measurement with size n_z
    double nis_radar_; // the current NIS (normalized innovation squared) for radar
    double nis_lidar_; // the current NIS for lidar

    // Generate sigma points.
    // @param x: State vector
    // @param P: Covariance matrix
    // @param n_aug: Length of the augmented state vector
    // @param lambda: Sigma point spreading parameter
    Eigen::MatrixXd generateSigmaPoints(const Eigen::VectorXd &x,
                                        const Eigen::MatrixXd &P,
                                        int n_aug,
                                        double lambda);

    // Kalman filter measurement update.
    // @param m_pkg: Measurement
    // @param Zsig: Measurement sigma points matrix
    // @param n_z: Length of the measurement vector
    void measurementUpdate(Detection& detection,
                            const Eigen::MatrixXd &Zsig,
                            int n_z);

    // Normalize angle
    inline double normalizeAngle(double phi);

};

#endif /* UKF_H_ */
