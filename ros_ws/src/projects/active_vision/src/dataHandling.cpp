#include "active_vision/dataHandling.h"

void test(){
	std::cout << "Testing" << std::endl;
}

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
}

void saveData(RouteData &in, std::fstream &saveTo){
	saveTo  << in.objType << ", "
			<< in.objPose[0] << ", " << in.objPose[1] << ", " << in.objPose[2] << ", "
			<< in.kinectPose[0] << ", " << in.kinectPose[1] << ", " << in.kinectPose[2] << ", "
			<< in.goodInitialGrasp << ", "
			<< in.graspQuality << ", "
			<< in.stepNumber << ", "
			<< in.filepath << ", ";
	std::vector<double> cVector;
	for(int i=0; i < in.stepVector.size(); i++){
		saveTo << in.stepVector[i][0] << ", " << in.stepVector[i][1] << ", " << in.stepVector[i][2];
		if(i+1 < in.stepVector.size()){
			saveTo << ", ";
		} else {
			saveTo << "\n";
		}
	}
}

std::string getCurTime(){
	time_t now = time(0);
	tm *ltm = localtime(&now);
	char temp[50];
	sprintf(temp, "%04d_%02d_%02d_%02d%02d%02d", 1900 + ltm->tm_year,1 + ltm->tm_mon,ltm->tm_mday,ltm->tm_hour,ltm->tm_min,ltm->tm_sec);
	std::string name = temp;
	return(name);
}

// Type 1 : Object, 2 : Unexplored, 3 : Result
// Ensure there is a folder called "DataRecAV" in Home folder
void savePointCloud(ptCldColor::Ptr cloud, std::string prefix, int type){
	std::string path = "./DataRecAV/";
	std::string name;
	switch(type){
		case 1:
		  name = path + prefix + "_object.pcd";	break;
		case 2:
			name = path + prefix + "_unexp.pcd";	break;
		case 3:
			name = path + prefix + "_result.pcd";	break;
		default:
			std::cout << "Error saving point cloud." << std::endl;
			return;
	}
	pcl::io::savePCDFileASCII(name,*cloud);
}
