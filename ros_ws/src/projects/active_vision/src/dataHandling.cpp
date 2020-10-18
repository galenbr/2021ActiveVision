#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <array>
#include <string>
#include <fstream>
#include <tuple>
#include <boost/make_shared.hpp>

typedef std::vector< std::tuple<int, int> > angle_path_vector;

void test(){
	std::cout << "Testing" << std::endl;
}

struct RouteData {
	std::string objType;
	std::vector<double> objPose;
	std::vector<double> kinectPose;
	bool goodInitialGrasp;
	float graspQuality;
	int stepNumber;
	angle_path_vector stepVector;
	std::string filepath;
};