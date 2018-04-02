#ifndef TEST_UTIL_H_
#define TEST_UTIL_H_

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

using namespace std;

typedef struct {
  string sensor_type;
  float x_measured;
  float y_measured;
  long timestamp;
  float x_groundtruth;
  float y_groundtruth;
  float vx_groundtruth;
  float vy_groundtruth;
  float yaw_groundtruth;
  float yawrate_groundtruth;
} UTIL_LIDAR_STRUCT;

typedef struct {
  string sensor_type;
  float rho_measured;
  float phi_measured;
  float rhodot_measured;
  long timestamp;
  float x_groundtruth;
  float y_groundtruth;
  float vx_groundtruth;
  float vy_groundtruth;
  float yaw_groundtruth;
  float yawrate_groundtruth;
} UTIL_RADAR_STRUCT;

class LaserRadarDataReader {
  public:
    LaserRadarDataReader(const char* filename);
    virtual ~LaserRadarDataReader();
    bool GetLine(char* lineStr);
    vector<string> GetParsedLine();
    vector<string> GetNextParsedLineByType(string sensorType);
    UTIL_LIDAR_STRUCT ParseLidarData(vector<string> parsedWords);
    UTIL_RADAR_STRUCT ParseRadarData(vector<string> parsedWords);
    void OutputLidar(UTIL_LIDAR_STRUCT lidarData);
    void OutputRadar(UTIL_RADAR_STRUCT radarData);
    void Close();
  private:
    ifstream* file_;
};

float calcErr(float expectedVal, float actualVal);

#endif /* TEST_UTIL_H_ */