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

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Typedef for convinience
typedef pcl::PointCloud<pcl::PointXYZRGB> ptCldColor;

void test();

struct RouteData {
  std::string objType;
  std::vector<double> objPose;
  std::vector<double> kinectPose;
  bool goodInitialGrasp;
  float graspQuality;
  int stepNumber;
  std::string filepath;
  std::vector<std::vector<double>> stepVector;
};

void printRouteData(RouteData &in);

void saveData(RouteData &in, std::fstream &saveTo);

std::string getCurTime();

void savePointCloud(ptCldColor::Ptr cloud, std::string prefix, int type);

#endif
