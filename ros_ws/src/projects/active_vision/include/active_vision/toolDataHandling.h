#ifndef TOOLDATAHANDLING
#define TOOLDATAHANDLING

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

struct RouteData{
  int parentID{-1};
  int childID{-1};
  ptCldColor obj,unexp,env;
  std::string objType;
  std::vector<double> objPose;
  bool goodInitialGrasp{false};
  bool success{false};
  float graspQuality{-1};
  int nSteps{0};
  std::vector<std::vector<double>> path;
  int direction{-1};
  int type{0};
  std::string filename;

  void reset(){
    parentID = -1; childID = -1;
    obj.clear();unexp.clear();env.clear();
    objType.clear(); objPose.clear();
    goodInitialGrasp = false; success = false;
    graspQuality = -1;
    nSteps = 0; path.clear();
    direction = -1;
    type = 0;
    filename.clear();
  }
};

void printRouteData(RouteData &in);

void saveData(RouteData &in, std::fstream &saveTo, std::string &dir);

std::string getCurTime();

void savePointCloud(ptCldColor::Ptr cloud, std::string dir, std::string prefix, int type);

void readPointCloud(ptCldColor::Ptr cloud, std::string dir, std::string prefix, int type);

std::vector<std::vector<std::string>> readCSV(std::string filename);

#endif
