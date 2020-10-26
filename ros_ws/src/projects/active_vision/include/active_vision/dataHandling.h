#ifndef DATA_HANDLING
#define DATA_HANDLING

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <array>
#include <string>
#include <fstream>
#include <tuple>
#include <boost/make_shared.hpp>

void test();

struct RouteData {
  std::string objType;
  std::vector<double> objPose;
  std::vector<double> kinectPose;
  bool goodInitialGrasp;
  float graspQuality;
  int stepNumber;
  std::vector<std::vector<double>> stepVector;
  std::string filepath;
};

void printRouteData(RouteData &in);

#endif
