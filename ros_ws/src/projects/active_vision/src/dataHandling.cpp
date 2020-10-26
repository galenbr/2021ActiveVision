#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <array>
#include <string>
#include <fstream>
#include <tuple>
#include <boost/make_shared.hpp>

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
	std::vector<std::vector<double>> stepVector;
	std::string filepath;
};

void printRouteData(RouteData &in){
	std::cout << in.objType << std::endl;
	printf("(%1.2f, %1.2f, %1.2f)\n", in.objPose[0], in.objPose[1], in.objPose[2]);
	printf("(%1.2f, %1.2f, %1.2f)\n", in.kinectPose[0], in.kinectPose[1], in.kinectPose[2]);
	printf("%1.2f\n", in.graspQuality);
	printf("Steps taken = %d\n", in.stepNumber);
	if(in.stepVector.size() > 1){
		printf("Step Path:\n");
		for(int i=0; i < in.stepVector.size(); i++){
			printf("(%1.2f, %1.2f, %1.2f)->", in.stepVector[i][0], in.stepVector[i][1], in.stepVector[i][2]);
		}
		printf("\n");
	}
};