#include "sensor_data.h"
#include <string>
#include <sstream>
#include <vector>
#include <assert.h>
#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::VectorXd;

// RadarData
RadarData::RadarData() 
{
  timestamp_ = 0;

  rho_measured_ = 0.0;
  phi_measured_ = 0.0;  
  rhodot_measured_ = 0.0;

  x_groundtruth_ = 0.0;
  y_groundtruth_ = 0.0;

  vx_groundtruth_ =  0.0;
  vy_groundtruth_ =  0.0;
  yaw_groundtruth_ =  0.0;
  yawrate_groundtruth_ = 0.0;     
}

RadarData::RadarData(const MeasurementPackage &measurement_pack) 
{
  timestamp_ = measurement_pack.timestamp_;

  rho_measured_ = measurement_pack.raw_measurements_(0);
  phi_measured_ = measurement_pack.raw_measurements_(1);
  rhodot_measured_ = measurement_pack.raw_measurements_(2);

  if (measurement_pack.raw_measurements_.size()>3)
  {
    x_groundtruth_ = measurement_pack.raw_measurements_(3);
    y_groundtruth_ = measurement_pack.raw_measurements_(4);

    vx_groundtruth_ =  measurement_pack.raw_measurements_(5);
    vy_groundtruth_ =  measurement_pack.raw_measurements_(6);
    yaw_groundtruth_ =  measurement_pack.raw_measurements_(7);
    yawrate_groundtruth_ =  measurement_pack.raw_measurements_(8); 
  }
  else 
  {
    x_groundtruth_ = 0.0;
    y_groundtruth_ = 0.0;

    vx_groundtruth_ =  0.0;
    vy_groundtruth_ =  0.0;
    yaw_groundtruth_ =  0.0;
    yawrate_groundtruth_ = 0.0;   
  }
}

RadarData::RadarData(float x, float y, long long timestamp)
{
  timestamp_ = timestamp;

  rho_measured_ = sqrt(x*x + y*y);
  phi_measured_ = atan2(y, x);  
  rhodot_measured_ = 0.0;

  float _x;
  float _y;
  GetXY(_x, _y);
  cout << "x,y = " << _x << ", " << _y << "\n";

  x_groundtruth_ = 0.0;
  y_groundtruth_ = 0.0;

  vx_groundtruth_ =  0.0;
  vy_groundtruth_ =  0.0;
  yaw_groundtruth_ =  0.0;
  yawrate_groundtruth_ = 0.0;   
}

RadarData::RadarData(float rho, float phi, float rhodot, long long timestamp)
{
  timestamp_ = timestamp;

  rho_measured_ = rho;
  phi_measured_ = phi; 
  rhodot_measured_ = rhodot;

  x_groundtruth_ = 0.0;
  y_groundtruth_ = 0.0;

  vx_groundtruth_ =  0.0;
  vy_groundtruth_ =  0.0;
  yaw_groundtruth_ =  0.0;
  yawrate_groundtruth_ = 0.0;   
}

RadarData::RadarData(const char* lineStr)
{
  string _lineStr = string(lineStr);
  vector<string> parsedWords;
  string buf;
  stringstream ss(_lineStr);

  while(ss >> buf)
  {
    parsedWords.push_back(buf);
  }

  rho_measured_ = strtof((parsedWords[1]).c_str(),0); 
  phi_measured_ = strtof((parsedWords[2]).c_str(),0); 
  rhodot_measured_ = strtof((parsedWords[3]).c_str(),0); 
  timestamp_= strtol((parsedWords[4]).c_str(),0,10); 
  x_groundtruth_ = strtof((parsedWords[5]).c_str(),0); 
  y_groundtruth_ = strtof((parsedWords[6]).c_str(),0); 
  vx_groundtruth_ = strtof((parsedWords[7]).c_str(),0); 
  vy_groundtruth_ = strtof((parsedWords[8]).c_str(),0); 
  yaw_groundtruth_ = strtof((parsedWords[9]).c_str(),0); 
  yawrate_groundtruth_ = strtof((parsedWords[10]).c_str(),0); 
}

RadarData::~RadarData()
{
}

MeasurementPackage RadarData::ToMeasurementPackage()
{
  MeasurementPackage measurement_pack;

  measurement_pack.sensor_type_ = MeasurementPackage::RADAR;
  measurement_pack.timestamp_ = timestamp_;

  measurement_pack.raw_measurements_ = VectorXd(3);
  measurement_pack.raw_measurements_ << rho_measured_, phi_measured_, rhodot_measured_;  

  return measurement_pack;
}

void RadarData::GetXY(float &x, float &y)
{
    Tools tools;

    float normalized_phi_measured_ = tools.ConstrainAngle(phi_measured_);
    x = rho_measured_ * cos(normalized_phi_measured_);
    y = rho_measured_ * sin(normalized_phi_measured_);

    assert(!isnan(x));
    assert(!isnan(y));
}

VectorXd RadarData::GetGroundTruth()
{
  VectorXd groundTruth = VectorXd(4);

  groundTruth << x_groundtruth_, y_groundtruth_, vx_groundtruth_, vy_groundtruth_;

  return groundTruth;
}

// LidarData

LidarData::LidarData()
{
  timestamp_ = 0;

  x_measured_ = 0.0;
  y_measured_ = 0.0;

  x_groundtruth_ = 0.0;
  y_groundtruth_ = 0.0;

  vx_groundtruth_ =  0.0;
  vy_groundtruth_ =  0.0;
  yaw_groundtruth_ =  0.0;
  yawrate_groundtruth_ = 0.0;     
}

LidarData::LidarData(const MeasurementPackage &measurement_pack)
{
  timestamp_ = measurement_pack.timestamp_;

  x_measured_ = measurement_pack.raw_measurements_(0);
  y_measured_ = measurement_pack.raw_measurements_(1);

  int mp_size = measurement_pack.raw_measurements_.size();

  if (mp_size > 2)
  {
    x_groundtruth_ = measurement_pack.raw_measurements_(2);
    y_groundtruth_ = measurement_pack.raw_measurements_(3);
    vx_groundtruth_ = measurement_pack.raw_measurements_(4);

    vy_groundtruth_ = measurement_pack.raw_measurements_(5);
    yaw_groundtruth_ = measurement_pack.raw_measurements_(6);
    yawrate_groundtruth_ = measurement_pack.raw_measurements_(7);
  }
  else 
  {
    x_groundtruth_ = 0.0;
    y_groundtruth_ = 0.0;

    vx_groundtruth_ =  0.0;
    vy_groundtruth_ =  0.0;
    yaw_groundtruth_ =  0.0;
    yawrate_groundtruth_ = 0.0;   
  }  
}

LidarData::LidarData(float x, float y, long long timestamp)
{
  timestamp_ = timestamp;

  x_measured_ = x;
  y_measured_ = y;

  x_groundtruth_ = 0.0;
  y_groundtruth_ = 0.0;

  vx_groundtruth_ =  0.0;
  vy_groundtruth_ =  0.0;
  yaw_groundtruth_ =  0.0;
  yawrate_groundtruth_ = 0.0;   
}

LidarData::LidarData(float rho, float phi, float rhodot, long long timestamp)
{
  timestamp_ = timestamp;

  Tools tools;
  float normalized_phi = tools.ConstrainAngle(phi);
  x_measured_  = rho * cos(normalized_phi);
  y_measured_ = rho * sin(normalized_phi);  

  x_groundtruth_ = 0.0;
  y_groundtruth_ = 0.0;

  vx_groundtruth_ =  0.0;
  vy_groundtruth_ =  0.0;
  yaw_groundtruth_ =  0.0;
  yawrate_groundtruth_ = 0.0;   
}

LidarData::LidarData(const char* lineStr)
{
  string _lineStr = string(lineStr);
  vector<string> parsedWords;
  string buf;
  stringstream ss(lineStr);

  while(ss >> buf)
  {
    parsedWords.push_back(buf);
  }

  x_measured_ = strtof((parsedWords[1]).c_str(),0); 
  y_measured_ = strtof((parsedWords[2]).c_str(),0); 
  timestamp_ = strtol((parsedWords[3]).c_str(),0,10); 
  x_groundtruth_ = strtof((parsedWords[4]).c_str(),0); 
  y_groundtruth_ = strtof((parsedWords[5]).c_str(),0); 
  vx_groundtruth_ = strtof((parsedWords[6]).c_str(),0); 
  vy_groundtruth_ = strtof((parsedWords[7]).c_str(),0); 
  yaw_groundtruth_ = strtof((parsedWords[8]).c_str(),0); 
  yawrate_groundtruth_ = strtof((parsedWords[9]).c_str(),0);  
}

LidarData::~LidarData()
{
}

MeasurementPackage LidarData::ToMeasurementPackage()
{
  MeasurementPackage measurement_pack;

  measurement_pack.sensor_type_ = MeasurementPackage::LASER;
  measurement_pack.timestamp_ = timestamp_;
  
  measurement_pack.raw_measurements_ = VectorXd(2);
  measurement_pack.raw_measurements_ << x_measured_, y_measured_;  

  return measurement_pack;
}

VectorXd LidarData::GetGroundTruth()
{
  VectorXd groundTruth = VectorXd(4);

  groundTruth << x_groundtruth_, y_groundtruth_, vx_groundtruth_, vy_groundtruth_;

  return groundTruth;
}
