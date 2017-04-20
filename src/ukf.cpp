/*______________________________________________________________________________
                                                                           80->|
  ukf.cpp
  This module implements the UKF class
*/

#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include "math.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  //+ Additional inits
  previous_timestamp_ = 0;
  is_initialized_ = false;
  n_x_ = 5;
  n_z_radar_ = 3;
  n_z_laser_ = 2;
  n_aug_ = n_x_ + 2;       // 7
  lambda_ = 3 - n_aug_;    // -4
  n_sig_ = 2 * n_aug_ + 1; // 15 sigma pts
  
  //+ Set weights
  weights_ = VectorXd::Zero(n_sig_);
  double lambda_aug = lambda_ + n_aug_;
  weights_.fill(0.5 / lambda_aug);
  weights_(0) = lambda_ / lambda_aug;
  
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  //+ state vector initialized to zero
  // px, py, velocity, yaw, yaw rate
  x_ = VectorXd::Zero(n_x_);              // 5x1

  //+ covariance matrix initialized to Identity
  P_ = MatrixXd::Identity(n_x_, n_x_);    // 5x5

  //+ Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.5;      //+ more reasonable than 30; tested with values 0.5 to 3.5

  //+ Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.61; //+ more reasonable than 30; tested with values 0.2 to 1.2

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  //+ Augmented mean vector initialized to zero
  x_aug_ = VectorXd::Zero(n_aug_);      // 7x1

  //+ Augmented state covariance
  P_aug_ = MatrixXd(n_aug_, n_aug_);    // 7x7

  //+ Sigma point matrices
  Xsig_aug_ = MatrixXd(n_aug_, n_sig_); // 7x15
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);  // 5x15
  
  //+ Noise covariance matrices for radar and lidar
  R_radar_ = MatrixXd(n_z_radar_, n_z_radar_); // 3x3
  R_radar_ << std_radr_ * std_radr_, 0, 0,
              0, std_radphi_ * std_radphi_, 0,
              0, 0, std_radrd_ * std_radrd_;
  R_laser_ = MatrixXd(n_z_laser_, n_z_laser_); // 2x2
  R_laser_ << std_laspx_ * std_laspx_, 0,
              0, std_laspy_ * std_laspy_;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} measurement_pack The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {

  /*****************************************************************************
  *  Initialization
  *****************************************************************************/
  if (!is_initialized_) {
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates
      double ro = measurement_pack.raw_measurements_[0];
      double phi = fmod(measurement_pack.raw_measurements_[1], 2 * M_PI);
      double ro_dot = measurement_pack.raw_measurements_[2];
      double cosphi = cos(phi);
      double sinphi = sin(phi);
      double px = ro * cosphi;
      double py = ro * sinphi;
      double vx = ro_dot * cosphi;
      double vy = ro_dot * sinphi;
      // Initialize the state (leaving yaw & yaw rate at zero)
      x_.head(n_x_-2) << px, py, sqrt(vx*vx + vy*vy);
      x_aug_.head(n_x_) = x_;
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize the state (leaving velocity, yaw, & yaw rate at zero)
      x_.head(n_x_-3) << measurement_pack.raw_measurements_[0], 
                         measurement_pack.raw_measurements_[1];
      x_aug_.head(n_x_) = x_;
    }
    previous_timestamp_ = measurement_pack.timestamp_;
    // Initialization complete, no need to predict or update
    is_initialized_ = true;
    return;
  }
  

  /*****************************************************************************
  *  Prediction
  *****************************************************************************/

  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  Prediction(dt);

  /*****************************************************************************
  *  Update 
  *****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    UpdateRadar(measurement_pack);
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    UpdateLidar(measurement_pack);
  }

}

// Support function I wrote for quiz L7.20
VectorXd UKF::PredictColumn(VectorXd& col, double dt) {
  double v = col(2);
  double psi = col(3);
  double psid = col(4);
  double va = col(5);
  double vPsiDot = col(6);
  double sinPsi = sin(psi);
  double cosPsi = cos(psi);
  double vpsi = 0;
  double inner = psi + (psid * dt);
  double vdt = v * dt;
  double dt2 = dt * dt;
  double vadt2 = va * dt2;

  // Add modeled process state change prediction
  VectorXd vt = VectorXd::Zero(n_x_);
  if (fabs(psid) > 1e-9) {
    vpsi = v / psid;
    vt(0) = vpsi * (sin(inner) - sinPsi);
    vt(1) = vpsi * (cosPsi - cos(inner));
    vt(3) = psid * dt;
  } else { // Avoid division by zero
    vt(0) = cosPsi * vdt;
    vt(1) = sinPsi * vdt;
  }

  // Add modeled process noise
  VectorXd vk = VectorXd(n_x_);
  vk << 0.5 * vadt2 * cosPsi,
        0.5 * vadt2 * sinPsi,
        dt * va,
        0.5 * dt2 * vPsiDot,
        dt * vPsiDot;

  return col.head(5) + vt + vk;
}


// The essence of my answer to quiz L7.17&20
void UKF::PredictSigmaPoints(double delta_t) {

  // Update augmented mean state
  x_aug_.head(n_x_) = x_;
  x_aug_(n_x_) = 0;
  x_aug_(n_x_+1) = 0;

  // Build augmented covariance matrix
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(n_x_,n_x_) = P_;
  P_aug_(n_x_,n_x_) = std_a_ * std_a_;
  P_aug_(n_x_+1,n_x_+1) = std_yawdd_ * std_yawdd_;

  // Update square root matrix
  MatrixXd L = P_aug_.llt().matrixL();

  // Determine augmented sigma points
  Xsig_aug_.col(0) = x_aug_;
  for (int i = 0; i< n_aug_; i++) {
    Xsig_aug_.col(i + 1) = x_aug_ + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug_.col(i + 1 + n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  // Predict sigma points
  Xsig_pred_.fill(0.0);
  for (int i = 0; i < Xsig_aug_.cols(); ++i) {
    VectorXd col = Xsig_aug_.col(i);
    // Write predicted points into columns
    Xsig_pred_.col(i) += PredictColumn(col, delta_t);
  }
}


// The essence of my answer to quiz L7.23
void UKF::PredictMeanAndCovariance() {

  // Predict mean
  x_.fill(0.0);
  x_ = Xsig_pred_ * weights_;
  x_(3) = atan2( sin(x_(3)), cos(x_(3)) ); // normalize yaw angle
  
  /* NOTE: If both parameters to atan2 are float zero, 
  the return value is defined as follows:
      atan2(0.0, 0.0)    = 0
      atan2(0.0, -0.0)   = 3.14159
      atan2(-0.0, 0.0)   = -0
      atan2(-0.0, -0.0)  = -3.14159
  See: http://en.cppreference.com/w/cpp/numeric/math/atan2
  */

  // Predict covariance matrix
  P_.fill(0);
  for (int i=0; i < Xsig_pred_.cols(); i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = atan2( sin(x_diff(3)), cos(x_diff(3)) ); // normalize angle
    P_ += weights_(i) * x_diff * x_diff.transpose();
  }
}


/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  // Step 1:
  PredictSigmaPoints(delta_t);
  // Step 2:
  PredictMeanAndCovariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.
  */

  // Create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_laser_, n_sig_); // 2x15

  for (int i=0; i < n_sig_; i++) {
    Zsig(0, i) = Xsig_pred_(0, i); // px  
    Zsig(1, i) = Xsig_pred_(1, i); // py
  }
  
  // Predicted measurement mean and covariance
  VectorXd z_pred = VectorXd(n_z_laser_);
  for (int i=0; i < Zsig.rows(); i++)
    z_pred.row(i) = Zsig.row(i) * weights_;  // order important!

  // Define measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_laser_, n_z_laser_); // 2x2
  S.fill(0.0);

  // and a cross-correlation matrix T.
  MatrixXd T = MatrixXd(n_x_, n_z_laser_); // 5x2
  T.fill(0.0);

  // Process the columns of the sigma point matrix
  for (int i=0; i < Zsig.cols(); i++) {
    // Compute measurement covariance diff
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S += weights_(i) * z_diff * z_diff.transpose();
    
    // Compute state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // Compute T
    T = T + weights_(i) * x_diff * z_diff.transpose();
  }
  // Add the laser noise covariance
  S += R_laser_;

  // Kalman gain 
  MatrixXd K = T * S.inverse();  // 5x2   x   2x2   =   5x2

  // Difference from real measure
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;  // 2x2

  // Update state mean x_ and covariance matrix P_
  x_ = x_ + K * z_diff;          // 5x2   x   2x2   =   5x2
  P_ = P_ - K * S * K.transpose();

  // NIS for lidar
  NIS_laser_  = z_diff.transpose() * S.inverse() * z_diff;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
// The essence of my answer to quiz L7.26 and some regular Kalman math
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  // Use radar data to update the belief about the object's
  // position. Modify the state vector, x_, and covariance, P_.

  // Create Zsig matrix for measurement space sigma points
  MatrixXd Zsig = MatrixXd(n_z_radar_, n_sig_); // 3x15

  // Transform sigma points (Xsig_pred) into measurement space Zsig matrix
  for (int i=0; i < Xsig_pred_.cols(); i++) {
    const VectorXd &xCol = Xsig_pred_.col(i);
    VectorXd zT = VectorXd::Zero(3); // Initialize elements to zero
    double px = xCol(0);
    double py = xCol(1);
    double phi = atan2(py, px);
    zT(1) = phi;
    double px2py2 = (px * px) + (py * py);
    if (fabs(px2py2) > 1e-6) { // Avoid div by zero
      double v = xCol(2);
      double phi = xCol(3);
      double sqrtpx2py2 = sqrt(px2py2);

      zT(0) = sqrtpx2py2;
      zT(2) = ((px * v * cos(phi)) + (py * v * sin(phi))) / sqrtpx2py2;
    }
    Zsig.col(i) = zT;
  }
  
  // Predicted measurement mean and covariance
  VectorXd z_pred = VectorXd(n_z_radar_);
  for (int i=0; i < Zsig.rows(); i++)
    z_pred.row(i) = Zsig.row(i) * weights_;  // order important!

  // Define measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_radar_, n_z_radar_);
  S.fill(0.0);

  // and a cross-correlation matrix T.
  MatrixXd T = MatrixXd(n_x_, n_z_radar_); // 5x3
  T.fill(0.0);
  
  // Process the columns of the Zsig sigma point matrix
  for (int i=0; i < Zsig.cols(); i++) {
    // Compute measurement covariance diff w/ norm
    VectorXd z_diff = Zsig.col(i) - z_pred;
    z_diff(1) = atan2( sin(z_diff(1)), cos(z_diff(1)) ); // See NOTE line 234
    S += weights_(i) * z_diff * z_diff.transpose();
    
    // Compute state difference w/ normalized angle
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = atan2( sin(x_diff(3)), cos(x_diff(3)) );

    // Compute T
    T = T + weights_(i) * x_diff * z_diff.transpose();
  }
  // Add the radar noise covariance
  S += R_radar_;

  // Kalman gain 
  MatrixXd K = T * S.inverse();

  // Difference from real measure w/ normalized angle
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;
  z_diff(1) = atan2( sin(z_diff(1)), cos(z_diff(1)) );

  // Update state mean x_ and covariance matrix P_
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // NIS for radar (Normalized Innovations Squared)
  NIS_radar_  = z_diff.transpose() * S.inverse() * z_diff;
}

/*
Normalized Innovations Squared:
web.stanford.edu/group/arl/sites/default/files/public/publications/Robust_TRN_Framework.pdf
*/