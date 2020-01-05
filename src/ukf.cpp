#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.5;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  P_ << 0.5, 0, 0, 0, 0,
        0, 0.15, 0, 0, 0,
        0, 0, 0.2, 0, 0,
        0, 0, 0, 0.02, 0, 
        0, 0, 0, 0, 0.005;
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;
  weights_ = VectorXd(2*n_aug_ + 1);
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_ + 1);
  is_initialized_ = false;
  time_us_ = 0.0;
  
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  // initialize the state
  if(!is_initialized_)
  {
    if(meas_package.sensor_type_ == MeasurementPackage::LASER) // lidar
    {
      x_ << meas_package.raw_measurements_[0],
            meas_package.raw_measurements_[1],
            0, 
            0,
            0;

      // P_ << std_laspx_*std_laspx_, 0, 0, 0, 0,
      //       0,std_laspy_*std_laspx_,0,0,0,
      //       0,0,1,0,0,
      //       0,0,0,1,0,
      //       0,0,0,0,1;
    }else if(meas_package.sensor_type_ == MeasurementPackage::RADAR)// radar
    {
      double roh = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      double roh_dot = meas_package.raw_measurements_[2];

      double px = roh * cos(phi);
      double py = roh * sin(phi);
      x_ << px,
            py,
            roh_dot*cos(phi),
            roh_dot*sin(phi),
            0;

      // P_ << std_radr_*std_radr_,0,0,0,0,
      //       0,std_radr_*std_radr_,0,0,0,
      //       0,0,std_radrd_*std_radrd_,0,0,
      //       0,0,0,std_radrd_*std_radrd_,0,
      //       0,0,0,0,1;
    }
    is_initialized_ = true;
    time_us_ = meas_package.timestamp_;
    return;  
  }
  
  double dt = (meas_package.timestamp_ - time_us_) / 1e6;
  time_us_ = meas_package.timestamp_;
  Prediction(dt);
  if(meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
    UpdateLidar(meas_package);
  }else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
    UpdateRadar(meas_package);
  }


}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
  
   // Generate the augmented sigma points
  VectorXd x_aug = VectorXd(n_aug_);
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);

  x_aug.fill(0);
  P_aug.fill(0);
  x_aug.head(5) = x_;
  
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_*std_a_;
  P_aug(6, 6) = std_yawdd_*std_yawdd_;
  MatrixXd A = P_aug.llt().matrixL();
  Xsig_aug.col(0) = x_aug;
  
  for(int i = 0; i < n_aug_; ++i)
  {
    Xsig_aug.col(i+1) = x_aug + sqrt(lambda_ + n_aug_) * A.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * A.col(i);
  }
 

  // Sigma points predictions
  for(int i = 0; i < 2*n_aug_+1; ++i)
  {
    double px = Xsig_aug(0, i);
    double py = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    double px_p, py_p;
    if(fabs(yawd) > 0.001)
    {
      px_p = px + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
      py_p = py + v/yawd * (-cos(yaw + yawd*delta_t) + cos(yaw));
    }else
    {
      px_p = px + v * cos(yaw) * delta_t;
      py_p = py + v * sin(yaw) * delta_t;
    }
    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t*cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t*sin(yaw);
    v_p = v_p + nu_a*delta_t;
    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }

  // predict the mean and covariance
  // create vector for predicted state
  VectorXd x = VectorXd(n_x_);
  // create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);
  // set weights
  weights_(0) = lambda_/(lambda_+n_aug_);
  for(int i = 1; i < 2*n_aug_+1; ++i)
  {
    weights_(i) = 0.5/(n_aug_+lambda_);
  }
  x.fill(0);
  for(int i = 0; i < 2*n_aug_+1; ++i)
  {
    x += weights_(i)*Xsig_pred_.col(i);
  }
  
  P.fill(0);
  for(int i = 0; i < 2*n_aug_+1; ++i)
  {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    P = P + weights_(i) * x_diff * x_diff.transpose() ;
  }
  x_ = x;
  P_ = P;
  //std::cout << "x_: " << x_ << std::endl;
  //std::cout << "P_: " << P_ << std::endl;
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  //Predict Radar Measurement
  //std::cout << "1. running here" << std::endl;
  int n_z = meas_package.raw_measurements_.size(); 
  //std::cout << "Radar n_z: " << n_z << std::endl;
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);
  VectorXd z_pred = VectorXd(n_z);
  MatrixXd S = MatrixXd(n_z, n_z); // measurement covariance matrix
  VectorXd z = meas_package.raw_measurements_;
  // transform sigma points Xsig_pred_ into measurement space
  for(int i = 0; i < 2*n_aug_+1; ++i)
  {
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);

    // measurement model
    Zsig(0, i) = px;       //px
    Zsig(1, i) = py;       //py
  } 
  //std::cout << "2. running here" << std::endl;

  // mean predicted measurement
  z_pred.fill(0);
  for(int i = 0; i < 2*n_aug_+1; ++i)
  {
    z_pred += weights_(i)*Zsig.col(i);
  }

  // mean covariance measurement
  S.fill(0);
  for(int i = 0; i < 2*n_aug_+1; ++i)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S += weights_(i) * z_diff * z_diff.transpose(); 
  }

  MatrixXd R = MatrixXd(2, 2);
  R << std_laspx_*std_laspx_, 0,
       0, std_laspy_*std_laspy_;
  S = S + R;

  // update
  // calculate correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0);
  for(int i = 0; i < 2*n_aug_+1; ++i)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    Tc += weights_(i) * x_diff * z_diff.transpose(); 
  }

  // Kalman gain K
  MatrixXd K = Tc * S.inverse();
  // measurement residual
  VectorXd z_diff = z - z_pred;
  // update state mean and covariance
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  double nis = (z - z_pred).transpose() * S.inverse() * (z - z_pred);
  lidarNIS.push_back(nis);
  std::cout << "Lidar-NIS: " << nis << std::endl;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  //Predict Radar Measurement
 
  int n_z = meas_package.raw_measurements_.size(); 
  // std::cout << "Radar n_z: " << n_z << std::endl;
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);
  VectorXd z_pred = VectorXd(n_z);
  MatrixXd S = MatrixXd(n_z, n_z); // measurement covariance matrix
  VectorXd z = meas_package.raw_measurements_;

  // transform sigma points Xsig_pred_ into measurement space
  for(int i = 0; i < 2*n_aug_+1; ++i)
  {
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, 0);

    // measurement model
    Zsig(0, i) = sqrt(px*px + py*py);         //phi
    Zsig(1, i) = atan2(py, px);               //yaw
    Zsig(2, i) = (px*cos(yaw)*v + py*sin(yaw)*v) / sqrt(px*px + py*py); // phi_dot
  } 
 
  // mean predicted measurement
  z_pred.fill(0);
  for(int i = 0; i < 2*n_aug_+1; ++i)
  {
    z_pred += weights_(i)*Zsig.col(i);
  }
 
  // mean covariance measurement
  S.fill(0);
  for(int i = 0; i < 2*n_aug_+1; ++i)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI; 
    S += weights_(i) * z_diff * z_diff.transpose(); 
  }
  
  MatrixXd R = MatrixXd(3, 3);
  R << std_radr_*std_radr_, 0, 0,
       0, std_radphi_*std_radphi_, 0,
       0, 0, std_radrd_*std_radrd_;
  S = S + R;

  // update
  // calculate correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0);
  for(int i = 0; i < 2*n_aug_+1; ++i)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI; 

    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    while (x_diff(3) > M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3) < -M_PI) x_diff(31) += 2.*M_PI;

    Tc += weights_(i) * x_diff * z_diff.transpose(); 
  }
  
  // Kalman gain K
  MatrixXd K = Tc * S.inverse();
  // measurement residual
  VectorXd z_diff = z - z_pred;
  while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
  while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI; 

  // update state mean and covariance
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
  double nis = (z - z_pred).transpose() * S.inverse() * (z - z_pred);
  radarNIS.push_back(nis);
  std::cout << "Radar-NIS: " << nis << std::endl;
}