#ifndef DATA_HANDLING
#define DATA_HANDLING

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <array>
#include <string>
#include <fstream>
#include <time.h>
#include <tuple>
#include <boost/make_shared.hpp>
#include <sstream>
#include <stdexcept>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Typedef for convinience
typedef pcl::PointCloud<pcl::PointXYZRGB> ptCldColor;

void test();

struct RouteData {
  ptCldColor::Ptr obj{new ptCldColor};
  ptCldColor::Ptr unexp{new ptCldColor};
  ptCldColor::Ptr env{new ptCldColor};
  std::string objType;
  std::vector<double> objPose;
  std::vector<double> kinectPose;
  bool goodInitialGrasp;
  float graspQuality;
  int stepNumber;
  int direction;
  std::string filename;
  std::vector<std::vector<double>> stepVector;
};

int getDirection(std::vector<double> &start, std::vector<double> &end, int minAngle);

void printRouteData(RouteData &in);

void saveData(RouteData &in, std::fstream &saveTo, std::string &dir);

std::string getCurTime();

void savePointCloud(ptCldColor::Ptr cloud, std::string dir, std::string prefix, int type);

void readPointCloud(ptCldColor::Ptr cloud, std::string dir, std::string prefix, int type);

std::vector<std::vector<std::string>> readCSV(std::string filename);

#endif
