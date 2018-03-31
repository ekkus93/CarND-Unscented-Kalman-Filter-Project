#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  assert(estimations.size() > 0 && estimations.size() == ground_truth.size());

  VectorXd rmse(4);
  rmse << 0,0,0,0;

  for (int i = 0; i < estimations.size(); i++)
  {
    VectorXd residual = estimations[i] - ground_truth[i];

    //coefficient-wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  //calculate the mean
  rmse = rmse / estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse; 
}

float Tools::ConstrainAngle(float angle)
{
    while (angle > M_PI) 
    {
        angle -= 2.0*M_PI;    
    }
    
    while (angle < -M_PI)
    {
        angle += 2.0*M_PI;
    }
    
    assert(angle >= -M_PI && angle <= M_PI);
    return angle;
}
