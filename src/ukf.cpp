#include "ukf.h"
#include "sensor_data.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  //use_radar_ = true;
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  //std_a_ = 30;
  //std_a_ = 0.2;
  std_a_ = 0.2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  //std_yawdd_ = 30;
  //std_yawdd_ = 0.2;
  std_yawdd_ = 0.2;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  n_aug_ = 7;
  lambda_ = 3 - n_aug_;

  weights_ = VectorXd(2*n_aug_+1);
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i=1; i<2*n_aug_+1; i++) {  
    weights_(i) = 0.5/(n_aug_+lambda_);
  }

  Xsig_pred_ = MatrixXd(n_aug_, 2*n_aug_+1);
}

UKF::~UKF() {}

void UKF::Init(const MeasurementPackage &measurement_pack)
{
  /**
    TODO:
    * Initialize the state ekf_.x_ with the first measurement.
    * Create the covariance matrix.
    * Remember: you'll need to convert radar from polar to cartesian coordinates.
  */
  // first measurement
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    InitRadar(measurement_pack);
  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    InitLidar(measurement_pack);
  }
  else
  {
    // skip bad measurement pack
    cout << "WARNING: unknown sensor type: " << measurement_pack.sensor_type_ << ". Skipping.\n";
    return;
  }
}

void UKF::InitLidar(const MeasurementPackage &measurement_pack)
{
  LidarData lidarData = LidarData(measurement_pack);

  x_ << lidarData.x_measured_, lidarData.y_measured_, 0, 0, 0;

  previous_timestamp_ = measurement_pack.timestamp_;

  // done initializing, no need to predict or update
  is_initialized_ = true;  
}

void UKF::InitRadar(const MeasurementPackage &measurement_pack)
{
  x_ << 1, 1, 0, 0, 0;

  RadarData radarData = RadarData(measurement_pack);

  float x_measured, y_measured;
  radarData.GetXY(x_measured, y_measured);

  if (x_measured != 0 && y_measured != 0)
  {
    x_(0) = x_measured;
    x_(1) = y_measured;    
  }   

  previous_timestamp_ = measurement_pack.timestamp_;

  // done initializing, no need to predict or update
  is_initialized_ = true;  
}

float UKF::CalcDt(long long t0, long long t1)
{
  return (t1 - t0)/1000000.0;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {
  cout << "###ProcessMeasurement\n";
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_initialized_) {
    Init(measurement_pack);
    return;  
  }

  float dt = CalcDt(previous_timestamp_, measurement_pack.timestamp_);

  cout << "###Before prediction\n";
  Prediction(dt);
  cout << "###After prediction\n";
  DisplayData();

  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
  {
    UpdateLidar(measurement_pack);
  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    UpdateRadar(measurement_pack);
  }
  else
  {
    // skip bad measurement pack
    cout << "WARNING: unknown sensor type: " << measurement_pack.sensor_type_ << ". Skipping.\n";
    return;
  }

  cout << "###After update\n";
  DisplayData();  

  previous_timestamp_ = measurement_pack.timestamp_;
}

void UKF::MakeXSigAug(MatrixXd &Xsig_aug, const MatrixXd &P,
                      int n_aug, VectorXd &x,
                      float std_a, float std_yawdd, int lambda)
{
  //Lesson 7, section 18: Augmentation Assigment 2
  VectorXd x_aug = VectorXd(n_aug);
  MatrixXd P_aug = MatrixXd(n_aug, n_aug);

  x_aug.head(5) = x;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P;
  P_aug(5,5) = std_a*std_a;
  P_aug(6,6) = std_yawdd*std_yawdd;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for(int i =0; i<n_aug; i++)
  {
    Xsig_aug.col(i+1) = x_aug + sqrt(lambda+n_aug) * L.col(i);
    Xsig_aug.col(i+1+n_aug) = x_aug - sqrt(lambda+n_aug) * L.col(i);
  }  
}

void UKF::MakeXSigPred(const MatrixXd &Xsig_aug, MatrixXd &Xsig_pred, int n_aug, double delta_t)
{
  // Predict Sigma Points
  // Lesson 7, section 21: Sigma Point Prediction Assignment 2
  
  for(int i=0; i<2*n_aug+1; i++)
  {
    // extract values for better readability
    double p_x = Xsig_aug(0, i);
    double p_y = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001)
    {
      px_p = p_x + v/yawd * (sin(yaw+yawd*delta_t) - sin(yaw));
      py_p = p_y + v/yawd * (cos(yaw) - cos(yaw+yawd*delta_t));
    }
    else 
    {
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // write predicted sigma point into right column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;

    Tools tools;
    Xsig_pred(3,i) = tools.ConstrainAngle(yaw_p);
    Xsig_pred(4,i) = yawd_p;
  }  
}

void UKF::PredictMeanAndCovariance(MatrixXd &Xsig, int n_aug, 
                                    int lambda, VectorXd &weights,
                                    VectorXd &x, MatrixXd &P)
{
  Tools tools;

  // Lesson 7, section 24: Predicted Mean and Convariance 
  // set weights
  double weight_0 = lambda/(lambda+n_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug+1; i++) {  //2n+1 weights
    weights(i) = 0.5/(n_aug+lambda);
  }

  //predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  
    //iterate over sigma points
    x = x+ weights(i) * Xsig.col(i);
  }

  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  
    //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig.col(i) - x;
    x_diff(3) = tools.ConstrainAngle(x_diff(3));

    P = P + weights(i) * x_diff * x_diff.transpose() ;
  }
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

  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  MakeXSigAug(Xsig_aug, P_, n_aug_, x_, std_a_, std_yawdd_, lambda_);

  // predict sigma points
  MakeXSigPred(Xsig_aug, Xsig_pred_, n_aug_, delta_t);

  PredictMeanAndCovariance(Xsig_pred_, n_aug_, lambda_, weights_, x_, P_);
}

/**
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
  Tools tools;
  int n_z = 2;

  LidarData ld = LidarData(meas_package);
  VectorXd z = ld.ToMeasurementPackage().raw_measurements_;

  MatrixXd Zsig = Xsig_pred_ * MatrixXd::Identity(n_x_, 2*n_aug_+1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_x_);
  z_pred.fill(0.0);
  for(int i=0; i<2*n_aug_+1; i++)
  {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_x_, n_x_);
  S.fill(0.0);
  for(int i=0; i<2*n_aug_+1; i++)
  {
    // 2n+1 sigma points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    z_diff(1) = tools.ConstrainAngle(z_diff(1));

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_laspx_*std_laspx_, 0, 
        0, std_laspy_*std_laspy_;  
  S = S + R;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_x_);  
     
  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  
    //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    z_diff(1) = tools.ConstrainAngle(z_diff(1));

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    //angle normalization
    x_diff(3) = tools.ConstrainAngle(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  z_diff(1) = tools.ConstrainAngle(z_diff(1));

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
}

void UKF::MakeZsig(const MatrixXd &Xsig, MatrixXd &Zsig)
{
  // transform sigma points into measurement space
  for (int i=0; i<2*n_aug_*1; i++)
  {
    // 2n+1 sigma points

    // extract values for better readibility
    double p_x = Xsig(0,i);
    double p_y = Xsig(1,i);
    double v = Xsig(2,i);
    double yaw = Xsig(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    // r
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);
    // phi
    Zsig(1,i) = atan2(p_y, p_x);
    // r_dot
    Zsig(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);
  }  
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

  Tools tools;
  int n_z = 3;

  // Lesson 7, section 27: Predict Radar Measurement Assignment 2
  RadarData rd = RadarData(meas_package);
  VectorXd z = rd.ToMeasurementPackage().raw_measurements_;

  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_+ 1);

  // transform sigma points into measurement space
  MakeZsig(Xsig_pred_, Zsig);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for(int i=0; i<2*n_aug_+1; i++)
  {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for(int i=0; i<2*n_aug_+1; i++)
  {
    // 2n+1 sigma points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    z_diff(1) = tools.ConstrainAngle(z_diff(1));

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_*std_radr_, 0, 0,
        0, std_radphi_*std_radphi_, 0,
        0, 0, std_radrd_*std_radrd_;
  S = S + R;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  
    //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    z_diff(1) = tools.ConstrainAngle(z_diff(1));

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    //angle normalization
    x_diff(3) = tools.ConstrainAngle(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  z_diff(1) = tools.ConstrainAngle(z_diff(1));

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
}

void UKF::DisplayData()
{
  cout << "x_: \n" << x_ << "\n";
  cout << "P_: \n" << P_ << "\n";
}

// for unit tests
bool UKF::GetIsInitialized()
{
  return is_initialized_;
}

long long UKF::GetPreviousTimestamp()
{
  return previous_timestamp_;
}