#include "util.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>

using namespace std;

LaserRadarDataReader::LaserRadarDataReader(const char* filename) 
{
  file_ = new ifstream(filename);
}

LaserRadarDataReader::~LaserRadarDataReader()
{
}

bool LaserRadarDataReader::GetLine(char* lineStr) 
{
  if (file_->getline(lineStr, 256, '\n'))
  {
    return true;
  }
  else
  {
    return false;
  }
}

vector<string> LaserRadarDataReader::GetParsedLine()
{
  char lineStr[256];
  vector<string> results;
  string buf;

  if (!GetLine(lineStr))
  {
    return results;
  }

  string _lineStr = string(lineStr);
  stringstream ss(_lineStr);

  while(ss >> buf)
  {
    results.push_back(buf);
  }
  
  return results;
}

vector<string> LaserRadarDataReader::GetNextParsedLineByType(string sensorType)
{
  vector<string> parsedLine = this->GetParsedLine();

  while(parsedLine.size() > 0 && parsedLine[0] != sensorType)
  {
    parsedLine = this->GetParsedLine();
  }
  
  return parsedLine;
}

UTIL_LIDAR_STRUCT LaserRadarDataReader::ParseLidarData(vector<string> parsedWords)
{
  UTIL_LIDAR_STRUCT lidarData;

  lidarData.sensor_type = parsedWords[0];
  lidarData.x_measured = strtof((parsedWords[1]).c_str(),0); 
  lidarData.y_measured = strtof((parsedWords[2]).c_str(),0); 
  lidarData.timestamp = strtol((parsedWords[3]).c_str(),0,10); 
  lidarData.x_groundtruth = strtof((parsedWords[4]).c_str(),0); 
  lidarData.y_groundtruth = strtof((parsedWords[5]).c_str(),0); 
  lidarData.vx_groundtruth = strtof((parsedWords[6]).c_str(),0); 
  lidarData.vy_groundtruth = strtof((parsedWords[7]).c_str(),0); 
  lidarData.yaw_groundtruth = strtof((parsedWords[8]).c_str(),0); 
  lidarData.yawrate_groundtruth = strtof((parsedWords[9]).c_str(),0); 

  return lidarData;
}

UTIL_RADAR_STRUCT LaserRadarDataReader::ParseRadarData(vector<string> parsedWords)
{
  UTIL_RADAR_STRUCT radarData;

  radarData.sensor_type = parsedWords[0];
  radarData. rho_measured = strtof((parsedWords[1]).c_str(),0); 
  radarData. phi_measured = strtof((parsedWords[2]).c_str(),0); 
  radarData.rhodot_measured = strtof((parsedWords[3]).c_str(),0); 
  radarData.timestamp = strtol((parsedWords[4]).c_str(),0,10); 
  radarData.x_groundtruth = strtof((parsedWords[5]).c_str(),0); 
  radarData.y_groundtruth = strtof((parsedWords[6]).c_str(),0); 
  radarData.vx_groundtruth = strtof((parsedWords[7]).c_str(),0); 
  radarData.vy_groundtruth = strtof((parsedWords[8]).c_str(),0); 
  radarData.yaw_groundtruth = strtof((parsedWords[9]).c_str(),0); 
  radarData.yawrate_groundtruth = strtof((parsedWords[10]).c_str(),0); 

  return radarData;
}

void LaserRadarDataReader::OutputLidar(UTIL_LIDAR_STRUCT lidarData)
{
  cout << "sensor_type: " << lidarData.sensor_type << "\n";
  cout << "x_measured: " << lidarData.x_measured << "\n";
  cout << "y_measured: " << lidarData.y_measured << "\n";
  cout << "timestamp: " << lidarData.timestamp << "\n";
  cout << "x_groundtruth: " << lidarData.x_groundtruth << "\n";
  cout << "y_groundtruth: " << lidarData.y_groundtruth << "\n";
  cout << "vx_groundtruth: " << lidarData.vx_groundtruth << "\n";
  cout << "vy_groundtruth: " << lidarData.vy_groundtruth << "\n";
  cout << "yaw_groundtruth: " << lidarData.yaw_groundtruth << "\n";
  cout << "yawrate_groundtruth: " << lidarData.yawrate_groundtruth << "\n";
}

void LaserRadarDataReader::OutputRadar(UTIL_RADAR_STRUCT radarData)
{
  cout << "sensor_type: " << radarData.sensor_type << "\n";
  cout << "rho_measured: " << radarData.rho_measured << "\n";
  cout << "phi_measured: " << radarData.phi_measured << "\n";
  cout << "rhodot_measured: " << radarData.rhodot_measured << "\n";
  cout << "timestamp: " << radarData.timestamp << "\n";
  cout << "x_groundtruth: " << radarData.x_groundtruth << "\n";
  cout << "y_groundtruth: " << radarData.y_groundtruth << "\n";
  cout << "vx_groundtruth: " << radarData.vx_groundtruth << "\n";
  cout << "vy_groundtruth: " << radarData.vy_groundtruth << "\n";
  cout << "yaw_groundtruth: " << radarData.yaw_groundtruth << "\n";
  cout << "yawrate_groundtruth: " << radarData.yawrate_groundtruth << "\n";
}

void LaserRadarDataReader::Close()
{
  file_->close();
}

float calcErr(float expectedVal, float actualVal)
{
  return fabs(expectedVal-actualVal)/expectedVal;
}