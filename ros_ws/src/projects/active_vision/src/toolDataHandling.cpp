#include "active_vision/toolDataHandling.h"

void printRouteData(RouteData &in){
	std::cout << in.objType << std::endl;
	printf("(%1.2f, %1.2f, %1.2f)\n", in.objPose[0], in.objPose[1], in.objPose[2]);
	printf("%1.2f\n", in.graspQuality);
	printf("Steps taken = %d\n", in.nSteps);
	if(in.path.size() > 1){
		printf("Direction = %d\n", in.direction);
		printf("Step Path:\n");
		for(int i=0; i < in.path.size(); i++){
			printf("(%1.2f, %1.2f, %1.2f)->", in.path[i][0], in.path[i][1], in.path[i][2]);
		}
		printf("\n");
	}
}

void saveData(RouteData &in, std::fstream &saveTo, std::string &dir){
	// Saving with some blank spaces to add information later if needed
	saveTo  << in.objType << ","
			<< in.objPose[0] << "," << in.objPose[1] << "," << in.objPose[2] << ","
			<< in.goodInitialGrasp << ","
			<< in.graspQuality << ","
			<< "dummy"<< ","
			<< "dummy"<< ","
			<< "dummy"<< ","
			<< "dummy"<< ","
			<< in.type << ","
			<< in.filename << ","
			<< in.direction << ","
			<< in.nSteps << ",";

	for(int i=0; i < in.path.size(); i++){
		saveTo << in.path[i][0] << "," << in.path[i][1] << "," << in.path[i][2];
		if(i+1 < in.path.size()){
			saveTo << ",";
		} else {
			saveTo << "\n";
		}
	}

	ptCldColor::Ptr temp{new ptCldColor};
	*temp = in.obj; 	savePointCloud(temp,dir,in.filename,1);
	*temp = in.unexp; savePointCloud(temp,dir,in.filename,2);
	*temp = in.env; 	savePointCloud(temp,dir,in.filename,3);
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

std::vector<nodeData> convertToTree(std::vector<std::vector<std::string>> &data){
  std::vector<nodeData> result; result.clear();
  nodeData temp;
  std::size_t found;
  int childID;
  // storing all the nodes first
  for(int i = 0; i < data.size(); i++){
    temp.reset();
    found = data[i][0].find(":");
    temp.nodeID = std::atoi(data[i][0].substr(0,found).c_str());
    temp.dirs = data[i][0].substr(found+1);
    result.push_back(temp);
  }
  for(int i = 0; i < data.size(); i++){
    for(int j = 1; j < data[i].size(); j++){
      if(data[i][j].length() > 0){
        childID = std::atoi(data[i][j].c_str());
        result[i].childIDs.push_back(childID);
        result[childID].parentID = i;
      }
    }
  }
  return result;
}
