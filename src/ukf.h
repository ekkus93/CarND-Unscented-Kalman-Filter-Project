#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:
  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long previous_timestamp_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  int n_zlas_;
  int n_zrad_;

  ///* Sigma point spreading parameter
  double lambda_;

  double NIS_lidar_;
  double NIS_radar_;
  int lidar_reading_cnt_;
  int radar_reading_cnt_;
  int lidar_reading_under_cnt_;
  int radar_reading_under_cnt_;
  double chi_threshold;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  void Init(const MeasurementPackage &measurement_pack);
  void InitLidar(const MeasurementPackage &measurement_pack);
  void InitRadar(const MeasurementPackage &measurement_pack);
  double CalcDt(long long t0, long long t1);

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage measurement_pack);

  void MakeXSigAug(const MatrixXd &P,
                      int n_aug, VectorXd &x,
                      double std_a, double std_yawdd, double lambda, MatrixXd &Xsig_aug);

  void MakeXSigPred(const MatrixXd &Xsig_aug, int n_aug, double delta_t, 
                    MatrixXd &Xsig_pred);
  void PredictMeanAndCovariance(const MatrixXd &Xsig_pred, int n_aug, 
                                    double lambda, const VectorXd &weights,
                                    VectorXd &x, MatrixXd &P);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  void PredictMeasurementLidar(const MatrixXd &Zsig,
                                const VectorXd &weights,
                                int n_aug, int n_zlas,  
                                double std_laspx, double std_laspy,
                                VectorXd &z_pred, MatrixXd &S);  
  double UpdateStateLidar(const VectorXd &weights, 
                                const MatrixXd &Xsig_pred,
                                const VectorXd &z_pred, const MatrixXd &Zsig,
                                const MatrixXd &S, const VectorXd &z,
                                int n_aug, int n_x, int n_zlas, 
                                VectorXd &x, MatrixXd &P);

  void MakeZsigLidar(const MatrixXd &Xsig_pred, 
                        int n_aug, MatrixXd &Zsig);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  void MakeZsigRadar(const MatrixXd &Xsig_pred, int n_aug, 
                    MatrixXd &Zsig);
  void PredictMeasurementRadar(const MatrixXd &Zsig,
                                  const VectorXd &weights,
                                  int n_aug, int n_zrad,  
                                  double std_radr, double std_radphi,
                                  double std_radrd,
                                  VectorXd &z_pred, MatrixXd &S);
  double UpdateStateRadar(const VectorXd &weights, 
                            const MatrixXd &Xsig_pred,
                            const VectorXd &z_pred, const MatrixXd &Zsig,
                            const MatrixXd &S, const VectorXd &z,
                            int n_aug, int n_x, int n_zrad, 
                            VectorXd &x, MatrixXd &P);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  void DisplayData();

  // for unit tests
  bool GetIsInitialized();
  long long GetPreviousTimestamp();

  VectorXd GetX();
  MatrixXd GetP();
  MatrixXd GetXSigPred();
};

#endif /* UKF_H */
