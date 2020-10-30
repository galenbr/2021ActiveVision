#include "active_vision/dataHandling.h"

void test(){
	std::cout << "Testing" << std::endl;
}

int getDirection(std::vector<double> &start, std::vector<double> &end, int minAngle){
	double diff[2]={};
	diff[0] = round((end[1]-start[1])*180/M_PI);
	diff[1] = round((end[2]-start[2])*180/M_PI);
	if     (diff[0]==0 					&& diff[1]==-minAngle) 	return(1);		//N
	else if(diff[0]==minAngle 	&& diff[1]==-minAngle) 	return(2);		//NE
	else if(diff[0]==minAngle 	&& diff[1]==0)	 				return(3);		//E
	else if(diff[0]==minAngle 	&& diff[1]==minAngle) 	return(4);		//SE
	else if(diff[0]==0 					&& diff[1]==minAngle) 	return(5);		//S
	else if(diff[0]==-minAngle 	&& diff[1]==minAngle) 	return(6);		//SW
	else if(diff[0]==-minAngle 	&& diff[1]==0) 					return(7);		//W
	else if(diff[0]==-minAngle 	&& diff[1]==-minAngle) 	return(8);		//NW
	else{
		std::cout << "ERROR : getDirection function" << std::endl;
		return(0);
	}
}

void printRouteData(RouteData &in){
	std::cout << in.objType << std::endl;
	printf("(%1.2f, %1.2f, %1.2f)\n", in.objPose[0], in.objPose[1], in.objPose[2]);
	printf("(%1.2f, %1.2f, %1.2f)\n", in.kinectPose[0], in.kinectPose[1], in.kinectPose[2]);
	printf("%1.2f\n", in.graspQuality);
	printf("Steps taken = %d\n", in.stepNumber);
	if(in.stepVector.size() > 1){
		printf("Direction = %d\n", in.direction);
		printf("Step Path:\n");
		for(int i=0; i < in.stepVector.size(); i++){
			printf("(%1.2f, %1.2f, %1.2f)->", in.stepVector[i][0], in.stepVector[i][1], in.stepVector[i][2]);
		}
		printf("\n");
	}
}

void saveData(RouteData &in, std::fstream &saveTo, std::string &dir){
	saveTo  << in.objType << ","
			<< in.objPose[0] << "," << in.objPose[1] << "," << in.objPose[2] << ","
			<< in.kinectPose[0] << "," << in.kinectPose[1] << "," << in.kinectPose[2] << ","
			<< in.goodInitialGrasp << ","
			<< in.graspQuality << ","
			<< in.stepNumber << ","
			<< in.direction << ","
			<< in.filename << ",";

	for(int i=0; i < in.stepVector.size(); i++){
		saveTo << in.stepVector[i][0] << "," << in.stepVector[i][1] << "," << in.stepVector[i][2];
		if(i+1 < in.stepVector.size()){
			saveTo << ",";
		} else {
			saveTo << "\n";
		}
	}
	savePointCloud(in.obj,dir,in.filename,1);
	savePointCloud(in.unexp,dir,in.filename,2);
	savePointCloud(in.env,dir,in.filename,3);
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
void savePointCloud(ptCldColor::Ptr cloud, std::string dir, std::string prefix, int type){

	std::string name;
	switch(type){
		case 1:
		  name = dir + prefix + "_object.pcd";	break;
		case 2:
			name = dir + prefix + "_unexp.pcd";	break;
		case 3:
			name = dir + prefix + "_result.pcd";	break;
		default:
			std::cout << "Error saving point cloud." << std::endl;
			return;
	}
	pcl::io::savePCDFileASCII(name,*cloud);
}

// Type 1 : Object, 2 : Unexplored, 3 : Result
void readPointCloud(ptCldColor::Ptr cloud, std::string dir, std::string prefix, int type){

	std::string name;
	switch(type){
		case 1:
		  name = dir + prefix + "_object.pcd";	break;
		case 2:
			name = dir + prefix + "_unexp.pcd";	break;
		case 3:
			name = dir + prefix + "_result.pcd";	break;
		default:
			std::cout << "Error saving point cloud." << std::endl;
			return;
	}
	if(pcl::io::loadPCDFile(name,*cloud) == -1) throw std::runtime_error("Could not read PCD file");
}

std::vector<std::vector<std::string>> readCSV(std::string filename){
	std::vector<std::vector<std::string>> data;
	std::vector<std::string> row;

	// Create an input filestream
	std::ifstream myFile(filename);
	// Make sure the file is open
  if(!myFile.is_open()) throw std::runtime_error("Could not open CSV file");

	std::string line, element;

	// Read data, line by line
  while(std::getline(myFile, line)){
    std::stringstream ss(line);
		row.clear();
		while(std::getline(ss, element, ',')){
    	row.push_back(element);
    }
		data.push_back(row);
  }

	myFile.close();

	return(data);
}
