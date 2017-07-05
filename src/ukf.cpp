#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.6;

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

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  is_initialized_ = false;

  //set state dimension
  n_x_ = 5;

  //set augmented dimension
  n_aug_ = 7;

  //define spreading parameter
  lambda_ = 3 - n_aug_;

  //create matrix with predicted sigma points as columns
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //set weights
  weights_ = VectorXd(2*n_aug_+1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  double w = 0.5 / (lambda_ + n_aug_);
  for(int i=1; i<2*n_aug_+1; i++){
      weights_(i) = w;
  }

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

      // Translate the measurments into the state vector
      float ro = meas_package.raw_measurements_[0];
      float phi = meas_package.raw_measurements_[1];
      float ro_p = meas_package.raw_measurements_[2];

      x_ << ro*cos(phi), ro*sin(phi), ro_p, 0, 0;

      // Considering high certeinity in position and velocity and high uncertainity in yaw and yawd      
      P_ << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 10, 0,
            0, 0, 0, 0, 10;

    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {

      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;

      // Considering high certeinity in position and high uncertainity in velocity yaw and yawd
      P_ << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 10, 0, 0,
            0, 0, 0, 10, 0,
            0, 0, 0, 0, 10;

    }

    is_initialized_ = true;
    previous_timestamp_ = meas_package.timestamp_;

    return;
  }
  

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
  
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
      //cout << " radar " << endl;
      if(x_(0)!=0 && x_(1)!=0){
        Prediction(dt);
        UpdateRadar(meas_package);
      }
      else{
        //This only hapen when x and y position are zero
        is_initialized_ = false;
      }

      previous_timestamp_ = meas_package.timestamp_;
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
      //cout << " laser " << endl;
      if(x_(0)!=0 && x_(1)!=0){
        Prediction(dt);
        UpdateLidar(meas_package);
      }
      else{
        //This only hapen when x and y position are zero
        is_initialized_ = false;
      }

      previous_timestamp_ = meas_package.timestamp_;
  }
  
  
  //ONLY FOR DEBUG
  //cout << "x_ = " << endl << x_ << endl;
  //cout << "P_ = " << endl << P_ << endl;
  

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  /*****************************************************************************
   *  1. Generate Sigma Points
   ****************************************************************************/
  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);
  x_aug.setZero();
  x_aug.head(n_x_) = x_;

  //create augmented covariance matrix
  MatrixXd P_aug = MatrixXd(7, 7);
  P_aug.setZero();
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;
  
  
  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();

  //Multiply by the spreading factor
  A *= sqrt(lambda_+n_aug_);

  //create augmented sigma points and set it as columns of matrix Xsig_aug
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.col(0) = x_aug;
  for(int i=0; i<n_aug_; i++){
      Xsig_aug.col(1+i)        = x_aug + A.col(i);
      Xsig_aug.col(n_aug_+1+i) = x_aug - A.col(i);
  }
  

  /*****************************************************************************
   *  2. Predict Sigma Points
   ****************************************************************************/
  
  // Process vectors
  VectorXd Xk, Vk, Ik;
  Xk = Ik = Vk = VectorXd(n_x_);
  
  // Some variables 
  double v, yaw, yawd, noise_a, noise_yawdd;

  for(int i=0; i<2*n_aug_+1; i++){

      //extract values for better readability
      v = Xsig_aug.col(i)(2);
      yaw = Xsig_aug.col(i)(3);
      yawd = Xsig_aug.col(i)(4);
      noise_a = Xsig_aug.col(i)(5);
      noise_yawdd = Xsig_aug.col(i)(6);

      //The old vector state
      Xk = Xsig_aug.col(i).head(n_x_);

      //Fill the vector Ik (the integral)
      Ik(2) = 0;
      Ik(4) = 0;
      Ik(3) = yawd * delta_t;
      //avoid division by zero
      if (fabs(yawd) < 0.001){
        Ik(0) = v * cos(yaw) * delta_t;
        Ik(1) = v * sin(yaw) * delta_t;
      }
      else{
        Ik(0) = (v/yawd) * (sin(yaw + yawd*delta_t) - sin(yaw));
        Ik(1) = (v/yawd) * (-cos(yaw + yawd*delta_t) + cos(yaw));
      }

      //Fill the noise vector Ik
      Vk(0) = cos(yaw) * noise_a *(delta_t*delta_t) / 2;
      Vk(1) = sin(yaw) * noise_a *(delta_t*delta_t) / 2;
      Vk(2) = delta_t * noise_a;
      Vk(3) = (delta_t*delta_t) * noise_yawdd / 2;
      Vk(4) = delta_t * noise_yawdd;

      //write predicted sigma points into right column
      Xsig_pred_.col(i) = Xk + Ik + Vk;


  }

  /*****************************************************************************
   *  3. Predict Mean and Covariance
   ****************************************************************************/
  //predict state mean
  x_.fill(0.0);
  for(int i=0; i<2*n_aug_+1; i++){
      x_ +=  weights_(i) * Xsig_pred_.col(i);
  }

  //predict state covariance matrix
  P_.setZero();
  VectorXd diff;
  for(int i=0; i<2*n_aug_+1; i++){
      // state difference
      diff = Xsig_pred_.col(i) - x_;

      //angle normalization
      while(diff(3) > M_PI) diff(3)-=2.*M_PI;
      while(diff(3) < -M_PI) diff(3)+=2.*M_PI;
      
      P_ += weights_(i) * diff * diff.transpose();
  }
  
  
}


/*
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  // I use the traditional kalman filter as the mesurment function in linear.
  // This is less computational expensive than use the UKF
  
  MatrixXd H = MatrixXd(2,5);
  H << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0;

  //incoming radar measurement
  VectorXd z = meas_package.raw_measurements_;

  //measurement noise covariance matrix
  MatrixXd R = MatrixXd(2,2);
  R.setZero();
  R(0,0) = std_laspx_ * std_laspx_;
  R(1,1) = std_laspy_ * std_laspy_;

  VectorXd z_pred = H * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;

  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H) * P_;


  /*****************************************************************************
   *  Compute NIS  
   ****************************************************************************/
  NIS_laser_ = y.transpose() * Si * y;

}


/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  /*****************************************************************************
   *  Predict Measurement
   ****************************************************************************/
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  double px, py, v, yaw;
  for(int i=0; i <2*n_aug_+1; i++){

    // extract values for better readibility
    px = Xsig_pred_.col(i)(0);
    py = Xsig_pred_.col(i)(1);
    v  = Xsig_pred_.col(i)(2);
    yaw= Xsig_pred_.col(i)(3);
    
    // measurement model
    Zsig.col(i)(0) = sqrt(px*px + py*py);
    Zsig.col(i)(1) = atan2(py,px);
    Zsig.col(i)(2) = (px*cos(yaw)*v + py*sin(yaw)*v) / Zsig.col(i)(0);
  }

  //calculate mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for(int i=0; i <2*n_aug_+1; i++){
    z_pred += weights_(i) * Zsig.col(i);
  }

  //calculate measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  VectorXd diff;
  for(int i=0; i <2*n_aug_+1; i++){
    //residual
    diff = Zsig.col(i) - z_pred;
    
    //angle normalization
    while (diff(1)>M_PI) diff(1)-=2.*M_PI;
    while (diff(1)<-M_PI) diff(1)+=2.*M_PI;

    S += weights_(i) * diff * diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R.setZero();
  R(0,0) = std_radr_ * std_radr_;
  R(1,1) = std_radphi_ * std_radphi_;
  R(2,2) = std_radrd_ * std_radrd_;
  S += R;


  /*****************************************************************************
   *  Update State 
   ****************************************************************************/
  //incoming radar measurement
  VectorXd z = meas_package.raw_measurements_;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  VectorXd diff_meas, diff_state;
  for (int i=0; i<2*n_aug_+1; i++) {
      //residual
      diff_state = Xsig_pred_.col(i) - x_;
      //angle normalization
      while (diff_state(3)>M_PI) diff_state(3) -= 2.*M_PI;
      while (diff_state(3)<-M_PI) diff_state(3) += 2.*M_PI;
      
      //residual
      diff_meas  = Zsig.col(i) - z_pred;
      //angle normalization
      while (diff_meas(1)>M_PI) diff_meas(1) -= 2.*M_PI;
      while (diff_meas(1)<-M_PI) diff_meas(1) += 2.*M_PI;
      
      Tc += weights_(i) * diff_state * diff_meas.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  
  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ += K * z_diff;
  P_ -= K * S * K.transpose();

  
  /*****************************************************************************
   *  Compute NIS  
   ****************************************************************************/
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;

}
