#include <active_vision/dataHandling.h>
#include <pcl/common/common.h>

// Function to print the HAF state vector
void printHAFStVec(std::vector<float> &stateVec, int dim){
  std::string red = "\x1b[31m";
  std::string green = "\x1b[32m";
  std::string off = "\x1b[0m";
  if (stateVec.size() < 2*dim*dim+2){
    std::cout << "Invalid State Vector" << std::endl;
    return;
  }
  std::cout << "\t Object\t\t\t\tUnexplored" << std::endl;
  for(int i = 0; i < dim; i++){
    // Object State Vector
    for(int j = 0; j < dim; j++){
      if (stateVec[i*dim+j] != 0) std::cout << red;
      std::cout << std::fixed << std::setfill('0') << std::setw(5) << std::setprecision(2) << stateVec[i*dim+j] << " ";
      if (stateVec[i*dim+j] != 0) std::cout << off;
    }
    std::cout << "\t";
    // unexplored State Vector
    for(int j = 0; j < dim; j++) {
      if (stateVec[dim*dim+i*dim+j] != 0) std::cout << red;
      std::cout << std::fixed << std::setfill('0') << std::setw(5) << std::setprecision(2) << stateVec[dim*dim+i*dim+j] << " ";
      if (stateVec[dim*dim+i*dim+j] != 0) std::cout << off;
    }
    std::cout << std::endl;
  }
  // Kinect Position
  std::cout << "Polar Angle : " << stateVec[2*dim*dim]*180/M_PI
            << ", Azhimuthal Angle : " << stateVec[2*dim*dim+1]*180/M_PI << std::endl << std::endl;
}

// Function to save the HAF state vector
void saveHAFStVec(std::fstream &saveTo, std::vector<float> &stateVec, std::string path, std::string direction){
  saveTo << path << ",,";
  for(int i = 0; i < stateVec.size(); i++){
    saveTo << stateVec[i] << ",";
  }
  saveTo << "," << direction << "\n" ;
}

// Creating the State vector
std::vector<float> HAFStVec(ptCldColor::Ptr obj, ptCldColor::Ptr unexp, std::vector<double> &kinViewsphere, int gridDim){
  Eigen::Vector3f pt;
  float gridSize[2]={};
  int gridLoc[2]={};

  pcl::PointXYZRGB minPtObj, maxPtObj;
  pcl::getMinMax3D(*obj, minPtObj, maxPtObj);

  float stateObj[gridDim][gridDim] = {};
  gridSize[0] = (maxPtObj.x-minPtObj.x)/gridDim;
  gridSize[1] = (maxPtObj.y-minPtObj.y)/gridDim;
  for (int i = 0; i < obj->size(); i++){
    pt = obj->points[i].getVector3fMap();
    pt[0] = pt[0]-minPtObj.x; pt[1] = pt[1]-minPtObj.y; pt[2] = pt[2]-minPtObj.z;

    gridLoc[0] = (int)(pt[0]/gridSize[0]);
    gridLoc[0] = std::min(gridLoc[0],gridDim-1);        // Ensuring that the grid loc for the extreme point is inside the grid
    gridLoc[0] = (gridDim-1)-gridLoc[0];                // Reversing so that the array and grid indices match
    gridLoc[1] = (int)(pt[1]/gridSize[1]);
    gridLoc[1] = std::min(gridLoc[1],gridDim-1);        // Ensuring that the grid loc for the extreme point is inside the grid

    stateObj[gridLoc[0]][gridLoc[1]] = std::max(stateObj[gridLoc[0]][gridLoc[1]],pt[2]);
  }

  float scale = 3;
  pcl::PointXYZRGB minPtUnexp, maxPtUnexp;
  minPtUnexp.x = minPtObj.x - (scale-1)*(maxPtObj.x-minPtObj.x)/2;
  minPtUnexp.y = minPtObj.y - (scale-1)*(maxPtObj.y-minPtObj.y)/2;
  minPtUnexp.z = minPtObj.z;
  maxPtUnexp.x = maxPtObj.x + (scale-1)*(maxPtObj.x-minPtObj.x)/2;
  maxPtUnexp.y = maxPtObj.y + (scale-1)*(maxPtObj.y-minPtObj.y)/2;
  maxPtUnexp.z = maxPtObj.z + 0.05;

  float stateUnexp[gridDim][gridDim] = {};
  gridSize[0] = (maxPtUnexp.x-minPtUnexp.x)/gridDim;
  gridSize[1] = (maxPtUnexp.y-minPtUnexp.y)/gridDim;
  for (int i = 0; i < unexp->size(); i++){
    pt = unexp->points[i].getVector3fMap();
    pt[0] = pt[0]-minPtUnexp.x; pt[1] = pt[1]-minPtUnexp.y; pt[2] = pt[2]-minPtUnexp.z;

    // Ignoring the unexplored point cloud which dont affect the object
    if (pt[2] <= maxPtUnexp.z-minPtUnexp.z){
      gridLoc[0] = (int)(pt[0]/gridSize[0]);
      gridLoc[0] = std::min(gridLoc[0],gridDim-1);        // Ensuring that the grid loc for the extreme point is inside the grid
      gridLoc[0] = (gridDim-1)-gridLoc[0];                // Reversing so that the array and grid indices match
      gridLoc[1] = (int)(pt[1]/gridSize[1]);
      gridLoc[1] = std::min(gridLoc[1],gridDim-1);        // Ensuring that the grid loc for the extreme point is inside the grid
      stateUnexp[gridLoc[0]][gridLoc[1]] = std::max(stateUnexp[gridLoc[0]][gridLoc[1]],pt[2]);
    }
  }

  std::vector<float> stateVec;
  // Storing both in the state vector
  for(int i = 0; i < gridDim; i++){
    for(int j = 0; j < gridDim; j++){
      stateVec.push_back(stateObj[i][j]*100);
    }
  }
  for(int i = 0; i < gridDim; i++){
    for(int j = 0; j < gridDim; j++){
      stateVec.push_back(stateUnexp[i][j]*100);
    }
  }
  // Adding kinect position
  stateVec.push_back(kinViewsphere[1]);
  stateVec.push_back(kinViewsphere[2]);

  return(stateVec);
}

void help(){
  std::cout << "******* State Vector Generator Help *******" << std::endl;
  std::cout << "Arguments : [Directory] [CSV filename] [Type]" << std::endl;
  std::cout << "Directory : Directory where csv file is there (./DataRecAV/)" << std::endl;
  std::cout << "CSV filename : CSV file name (Data.csv)" << std::endl;
  std::cout << "Type : 1 (HSV Based)" << std::endl;
  std::cout << "*******" << std::endl;
}

int main(int argc, char** argv){
  if(argc != 4){
    std::cout << "ERROR. Incorrect number of arguments." << std::endl;
    help(); return(-1);
  }
  int type;
  type = std::atoi(argv[3]);
  if(type != 1){
    std::cout << "ERROR. Incorrect type." << std::endl;
    help(); return(-1);
  }

  std::string directory(argv[1]);
  std::string csvFile(argv[2]);
  std::string newCsvFile;
  newCsvFile = csvFile.substr(0,18)+"stateVec.csv";
  // std::cout << csvFile << "," << newCsvFile << std::endl;
  std::fstream fout;
 	fout.open(directory+newCsvFile, std::ios::out);


  std::vector<std::vector<std::string>> data;
  data = readCSV(directory+csvFile);

  int kinColID = 5;
  int dirColID = 11;
  int pathColID = 12;

  int gridDim = 5;
  ptCldColor::Ptr ptrPtCldObj{new ptCldColor};
  ptCldColor::Ptr ptrPtCldUnexp{new ptCldColor};
  std::vector<double> kinViewsphere = {0,0,0};
  std::vector<float> stateVec;

  for(int i = 0; i < data.size(); i++){
    // Generate State Vector only if direction is not 0
    if(std::atoi(data[i][dirColID-1].c_str()) != 0){
      readPointCloud(ptrPtCldObj,directory,data[i][pathColID-1],1);
      readPointCloud(ptrPtCldUnexp,directory,data[i][pathColID-1],2);
      kinViewsphere[0] = std::atof(data[i][kinColID-1].c_str());
      kinViewsphere[1] = std::atof(data[i][kinColID-1+1].c_str());
      kinViewsphere[2] = std::atof(data[i][kinColID-1+2].c_str());

      if(type == 1){
        stateVec = HAFStVec(ptrPtCldObj,ptrPtCldUnexp,kinViewsphere,gridDim);
        // printHAFStVec(stateVec,gridDim);
        saveHAFStVec(fout,stateVec,data[i][pathColID-1],data[i][dirColID-1]);
      }
    }
  }
  fout.close();
  std::cout << "State Vector saved to " << directory+newCsvFile << ".\n";
}
