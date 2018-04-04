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
  use_radar_ = true;

  n_aug_ = 7;
  n_x_ = 5;
  n_zlas_ = 2;
  n_zrad_ = 3;
  lambda_ = 3 - n_aug_;  

  cout << "###a1\n";
  // initial state vector
  x_ = VectorXd(n_x_);
  x_.fill(0.0);
  cout << "###a2\n";  

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_.fill(0.0);
  cout << "###a3\n";  

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

  cout << "###a4\n";
  weights_ = VectorXd(2*n_aug_+1);
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i=1; i<2*n_aug_+1; i++) {  
    weights_(i) = 0.5/(n_aug_+lambda_);
  }

  cout << "###a5\n";
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  cout << "###Xsig_pred_: " << Xsig_pred_.rows() << ", " << Xsig_pred_.cols() << "\n";
  Xsig_pred_.fill(0.0);
  cout << "###a6\n";

  previous_timestamp_ = 0;

  NIS_lidar_ = 0.0;
  NIS_radar_ = 0.0;
  mean_NIS_lidar_ = 0.0;
  mean_NIS_radar_ = 0.0;
  lidar_reading_cnt_ = 0;
  radar_reading_cnt_ = 0;
  cout << "###a7\n";
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
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    InitRadar(measurement_pack);
  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    InitLidar(measurement_pack);
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

double UKF::CalcDt(long long t0, long long t1)
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

  double dt = CalcDt(previous_timestamp_, measurement_pack.timestamp_);

  //cout << "###Before prediction: " << measurement_pack.timestamp_ << "\n";
  Prediction(dt);
  //cout << "###After prediction " << measurement_pack.timestamp_ << "\n";
  DisplayData();

  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER && use_laser_) 
  {
    UpdateLidar(measurement_pack);
  }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
  {
    UpdateRadar(measurement_pack);
  }
  else
  {
    return;
  }

  /*
  cout << "###After update\n";
  DisplayData();  
  */

  previous_timestamp_ = measurement_pack.timestamp_;
}

void UKF::MakeXSigAug(MatrixXd &Xsig_aug, const MatrixXd &P,
                      int n_aug, VectorXd &x,
                      double std_a, double std_yawdd, double lambda)
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
    float p_x = Xsig_aug(0, i);
    float p_y = Xsig_aug(1, i);
    float v = Xsig_aug(2, i);
    float yaw = Xsig_aug(3, i);
    float yawd = Xsig_aug(4, i);
    float nu_a = Xsig_aug(5, i);
    float nu_yawdd = Xsig_aug(6, i);

    // predicted state values
    float px_p, py_p;

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

    float v_p = v;
    float yaw_p = yaw + yawd*delta_t;
    float yawd_p = yawd;

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

void UKF::PredictMeanAndCovariance(const MatrixXd &Xsig_pred, int n_aug, 
                                    double lambda, VectorXd &weights,
                                    VectorXd &x, MatrixXd &P)
{
  Tools tools;

  // Lesson 7, section 24: Predicted Mean and Convariance 
  // set weights
  weights(0) = lambda/(lambda+n_aug);
  for (int i=1; i<2*n_aug+1; i++) {  
    //2n+1 weights
    weights(i) = 0.5/(n_aug+lambda);
  }

  //predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  
    //iterate over sigma points 
    /*
    cout << "###" << i << "\n";
    cout << "x: " << x.size() << "\n" << x << "\n";
    cout << "weights(" << i << "): " << weights.size() << "\n" << weights(i) << "\n";
    cout << "Xsig_pred.col(" << i << "): " << Xsig_pred.col(i).size() << "\n" 
          << Xsig_pred.col(i) << "\n";
    */
    x = x + weights(i) * Xsig_pred.col(i);
  }

  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  
    //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    x_diff(3) = tools.ConstrainAngle(x_diff(3));

    P = P + weights(i) * x_diff * x_diff.transpose();
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

void UKF::PredictMeasurementLidar(const MatrixXd &Zsig,
                                  const VectorXd &weights,
                                  int n_aug, int n_zlas,  
                                  double std_laspx, double std_laspy,
                                  VectorXd &z_pred, MatrixXd &S)
{
  assert(z_pred.rows() == Zsig.rows());
  assert(S.rows() == n_zlas_);
  assert(S.cols() == n_zlas_);

  //mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug+1; i++) 
  {
      z_pred = z_pred + weights(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) 
  {  
    //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(2, 2);
  R <<    std_laspx*std_laspx, 0, 
          0, std_laspy*std_laspy;
  S = S + R;
}

double UKF::UpdateStateLidar(const VectorXd &weights, 
                            const MatrixXd &Xsig_pred,
                            const VectorXd &z_pred, const MatrixXd &Zsig,
                            const MatrixXd &S, const VectorXd &z,
                            int n_aug, int n_x, int n_zlas, 
                            VectorXd &x, MatrixXd &P)
{
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, n_zlas);

  //calculate cross correlation matrix
  Tc.fill(0.0);

  for (int i = 0; i < 2 * n_aug + 1; i++) 
  {  
    //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //update state mean and covariance matrix
  x = x + K * z_diff;
  P = P - K*S*K.transpose();  

  // NIS
  return z_diff.transpose() * S.inverse() * z_diff;
}

void UKF::MakeZsigLidar(const MatrixXd &Xsig_pred, 
                        int n_aug, MatrixXd &Zsig)
{
  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug + 1; i++) {  
    // measurement model
    Zsig(0,i) = Xsig_pred_(0,i);          //px
    Zsig(1,i) = Xsig_pred_(1,i);          //py
  }
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
  VectorXd z_pred = VectorXd(n_zlas_);
  MatrixXd S = MatrixXd(n_zlas_, n_zlas_);

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_zlas_, 2 * n_aug_ + 1); 
  MakeZsigLidar(Xsig_pred_, n_aug_, Zsig);

  PredictMeasurementLidar(Zsig, weights_,
                          n_aug_, n_zlas_,  
                          std_laspx_, std_laspy_, 
                          z_pred, S);

  NIS_lidar_ = UpdateStateRadar(weights_, Xsig_pred_,
                                z_pred, Zsig, S, meas_package.raw_measurements_, 
                                n_aug_, n_x_, n_zlas_, 
                                x_, P_);

  mean_NIS_lidar_ = ((mean_NIS_lidar_ * lidar_reading_cnt_) + NIS_lidar_)/(lidar_reading_cnt_+1);
  lidar_reading_cnt_++;

  cout << "NIS_lidar_: " << NIS_lidar_ << "\n";
  cout << "mean_NIS_lidar_: " << NIS_lidar_ << "\n";
}

void UKF::MakeZsigRadar(const MatrixXd &Xsig_pred, int n_aug, 
                    MatrixXd &Zsig)
{
  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug + 1; i++) {  
    //2n+1 simga points

    // extract values for better readibility
    float p_x = Xsig_pred(0,i);
    float p_y = Xsig_pred(1,i);
    float v  = Xsig_pred(2,i);
    float yaw = Xsig_pred(3,i);

    float v1 = cos(yaw)*v;
    float v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }
}

void UKF::PredictMeasurementRadar(const MatrixXd &Zsig,
                                  const VectorXd &weights,
                                  int n_aug, int n_zrad,  
                                  double std_radr, double std_radphi,
                                  double std_radrd,
                                  VectorXd &z_pred, MatrixXd &S)
{
  // Lesson 7, section 27: Predict Radar Measurement Assignment 2

  //mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug+1; i++) 
  {
      z_pred = z_pred + weights(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  Tools tools;
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) 
  {  
    //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    z_diff(1) = tools.ConstrainAngle(z_diff(1));

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_zrad, n_zrad);
  R <<    std_radr*std_radr, 0, 0,
          0, std_radphi*std_radphi, 0,
          0, 0,std_radrd*std_radrd;
  S = S + R;
}

double UKF::UpdateStateRadar(const VectorXd &weights, 
                            const MatrixXd &Xsig_pred,
                            const VectorXd &z_pred, const MatrixXd &Zsig,
                            const MatrixXd &S, const VectorXd &z,
                            int n_aug, int n_x, int n_zrad, 
                            VectorXd &x, MatrixXd &P)
{
  // Lesson 7, Section 30 UKF Update
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, n_zrad);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  Tools tools;

  for (int i = 0; i < 2 * n_aug + 1; i++) 
  {  
    //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    z_diff(1) = tools.ConstrainAngle(z_diff(1));

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;

    //angle normalization
    x_diff(3) = tools.ConstrainAngle(x_diff(3));

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  z_diff(1) = tools.ConstrainAngle(z_diff(1));

  //update state mean and covariance matrix
  x = x + K * z_diff;
  P = P - K*S*K.transpose();  

  // NIS
  return z_diff.transpose() * S.inverse() * z_diff;
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
  VectorXd z_pred = VectorXd(n_zrad_);
  MatrixXd S = MatrixXd(n_zrad_, n_zrad_);

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_zrad_, 2 * n_aug_ + 1); 
  MakeZsigRadar(Xsig_pred_, n_aug_, Zsig);

  PredictMeasurementRadar(Zsig, weights_,
                          n_aug_, n_zrad_,  
                          std_radr_, std_radphi_, std_radrd_,
                          z_pred, S);

  NIS_radar_ = UpdateStateRadar(weights_, Xsig_pred_,
                                z_pred, Zsig, S, meas_package.raw_measurements_, 
                          n_aug_, n_x_, n_zrad_, 
                          x_, P_);

  mean_NIS_radar_ = ((mean_NIS_radar_ * radar_reading_cnt_) + NIS_radar_)/(radar_reading_cnt_+1);
  radar_reading_cnt_++;

  cout << "NIS_radar_: " << NIS_radar_ << "\n";

  cout << "mean_NIS_radar_: " << NIS_radar_ << "\n";
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