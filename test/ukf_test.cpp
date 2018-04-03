#include <limits.h>
#include <iostream>
#include "ukf.h"
#include "sensor_data.h"
#include "util.h"
#include "gtest/gtest.h"

TEST(UFKTest, InitLidar)
{
  LaserRadarDataReader dr = LaserRadarDataReader("/Users/phillipcchin/work/carnd/CarND-Unscented-Kalman-Filter-Project/data/obj_pose-laser-radar-synthetic-input.txt");

  char lineStr[256];

  dr.GetLine(lineStr);
  while (lineStr[0] != 'L')
  {
    dr.GetLine(lineStr);
  }

  LidarData ld = LidarData(lineStr);
  MeasurementPackage mp = ld.ToMeasurementPackage();

  UKF ukf;

  ukf.InitLidar(mp);

  EXPECT_NEAR(3.122427e-01, ukf.x_(0), 0.001);
  EXPECT_NEAR(5.803398e-01, ukf.x_(1), 0.001);
  EXPECT_NEAR(0.0, ukf.x_(2), 0.001);
  EXPECT_NEAR(0.0, ukf.x_(3), 0.001);

  EXPECT_EQ(mp.timestamp_, ukf.GetPreviousTimestamp());
  EXPECT_TRUE(ukf.GetIsInitialized());  
}

TEST(UFKTest, InitRadar) {
  // TODO: change this so the path isn't hard coded.  
  LaserRadarDataReader dr = LaserRadarDataReader("/Users/phillipcchin/work/carnd/CarND-Unscented-Kalman-Filter-Project/data/obj_pose-laser-radar-synthetic-input.txt");

  char lineStr[256];
  float expected_val, actual_val;
  float err_sum_x = 0.0;
  float err_sum_y = 0.0;
  int n=100;

  dr.GetLine(lineStr);
  while (lineStr[0] != 'R')
  {
    dr.GetLine(lineStr);
  }

  RadarData rd = RadarData(lineStr);
  MeasurementPackage mp = rd.ToMeasurementPackage(); 
  for(int i=0; i<n; i++)
  {
    UKF ukf;

    // Init
    ukf.InitRadar(mp);
    expected_val = rd.x_groundtruth_;
    actual_val = ukf.x_(0);
    cout << "###x_groundtruth_: " << rd.x_groundtruth_ << "\n";
    cout << "###actual_val: " << ukf.x_(0) << "\n";
    err_sum_x += calcErr(expected_val, actual_val);

    expected_val = rd.y_groundtruth_;
    actual_val = ukf.x_(1);  
    cout << "###y_groundtruth_: " << rd.y_groundtruth_ << "\n";
    cout << "###actual_val: " << ukf.x_(1) << "\n";
    err_sum_y += calcErr(expected_val, actual_val);

    EXPECT_NEAR(0.0, ukf.x_(2), 0.001);
    EXPECT_NEAR(0.0, ukf.x_(3), 0.001);

    EXPECT_EQ(mp.timestamp_, ukf.GetPreviousTimestamp());
    EXPECT_TRUE(ukf.GetIsInitialized());

    dr.GetLine(lineStr);
    while (lineStr[0] != 'R')
    {
      dr.GetLine(lineStr);
    }

    rd = RadarData(lineStr);
    mp = rd.ToMeasurementPackage();  
  }

  EXPECT_TRUE(err_sum_x/n <= 0.10);
  EXPECT_TRUE(err_sum_y/n <= 0.10);
}

TEST(UFKTest, MakeXSigAug) 
{
  UKF ukf;

  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //Process noise standard deviation longitudinal acceleration in m/s^2
  float std_a = 0.2;

  //Process noise standard deviation yaw acceleration in rad/s^2
  float std_yawdd = 0.2;

  //define spreading parameter
  int lambda = 3 - ukf.n_aug_;

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

  VectorXd x = VectorXd(n_x);
  x << 5.7441, 1.3800, 2.2049, 0.5015, 0.3528;

  //create example covariance matrix
  MatrixXd P = MatrixXd(n_x, n_x);
  P <<    0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
          0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;    

  MatrixXd Xsig_aug_expected = MatrixXd(n_aug, 2 * n_aug + 1);
  Xsig_aug_expected <<   
    5.7441, 5.85768, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.63052, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441,
    1.38, 1.34566, 1.52806, 1.38, 1.38, 1.38, 1.38, 1.38, 1.41434, 1.23194, 1.38, 1.38, 1.38, 1.38, 1.38,
      2.2049, 2.28414, 2.24557, 2.29582, 2.2049, 2.2049, 2.2049, 2.2049, 2.12566, 2.16423, 2.11398, 2.2049, 2.2049, 2.2049, 2.2049,
    0.5015, 0.44339, 0.631886, 0.516923, 0.595227, 0.5015, 0.5015, 0.5015, 0.55961, 0.371114, 0.486077, 0.407773, 0.5015, 0.5015, 0.5015,
    0.3528, 0.299973, 0.462123, 0.376339, 0.48417, 0.418721, 0.3528, 0.3528, 0.405627, 0.243477, 0.329261,  0.22143, 0.286879, 0.3528, 0.3528,
    0, 0, 0, 0, 0, 0, 0.34641, 0, 0, 0, 0, 0, 0, -0.34641, 0,
    0, 0, 0, 0, 0, 0, 0, 0.34641, 0, 0, 0, 0, 0, 0, -0.34641;

    ukf.MakeXSigAug(Xsig_aug, P, n_aug, x, std_a, std_yawdd, lambda);

    for(int row=0; row<Xsig_aug_expected.rows(); row++)
    {
      for(int col=0; col<Xsig_aug_expected.cols(); col++)
      {
        EXPECT_NEAR(Xsig_aug(row, col), Xsig_aug_expected(row, col), 0.001);        
      }
    }
}

TEST(UFKTest, MakeXSigPred) 
{
  UKF ukf;

  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;    

  //create example sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
     Xsig_aug <<
    5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
      1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
    2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
    0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
    0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
         0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
         0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

  MatrixXd Xsig_pred_expected = MatrixXd(n_x, 2 * n_aug + 1);
  Xsig_pred_expected <<
    5.93553, 6.06251, 5.92217, 5.9415, 5.92361, 5.93516, 5.93705, 5.93553, 5.80832, 5.94481, 5.92935, 5.94553, 5.93589, 5.93401, 5.93553,
    1.48939, 1.44673, 1.66484, 1.49719, 1.508, 1.49001, 1.49022, 1.48939, 1.5308, 1.31287, 1.48182, 1.46967, 1.48876, 1.48855, 1.48939,
    2.2049, 2.28414, 2.24557, 2.29582, 2.2049, 2.2049, 2.23954, 2.2049, 2.12566, 2.16423, 2.11398, 2.2049, 2.2049, 2.17026, 2.2049,
    0.53678, 0.473387, 0.678098, 0.554557, 0.643644, 0.543372, 0.53678, 0.538512, 0.600173, 0.395462, 0.519003, 0.429916, 0.530188, 0.53678, 0.535048,
    0.3528, 0.299973, 0.462123, 0.376339, 0.48417, 0.418721, 0.3528, 0.387441, 0.405627, 0.243477, 0.329261, 0.22143, 0.286879, 0.3528, 0.318159;

  double delta_t = 0.1; //time diff in sec    

  ukf.MakeXSigPred(Xsig_aug, Xsig_pred, n_aug, delta_t);

  for(int row=0; row<Xsig_pred_expected.rows(); row++)
  {
    for(int col=0; col<Xsig_pred_expected.cols(); col++)
    { 
      /*
      if (fabs(Xsig_pred(row, col) - Xsig_pred_expected(row, col)) > 0.001)
      {
        cout << row << ", " << col << ": " << Xsig_pred(row, col) << ", " << Xsig_pred_expected(row, col) << "\n";
      }
      */
      EXPECT_NEAR(Xsig_pred(row, col), Xsig_pred_expected(row, col), 0.001);  
    }
  }
}