#include <active_vision/toolDataHandling.h>
#include <active_vision/toolStateVector.h>
#include <pcl/common/common.h>

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
  newCsvFile = csvFile.substr(0,csvFile.size()-4) + "_stateVec.csv";
  // std::cout << csvFile << "," << newCsvFile << std::endl;
  std::fstream fout;
 	fout.open(directory+newCsvFile, std::ios::out);

  std::vector<std::vector<std::string>> data = readCSV(directory+csvFile);
  printf("Number of rows in the input file = %d\n",int(data.size()));

  if(type == 1){
    int stepTypeColID = 11;
    int pathColID     = 12;
    int dirColID      = 13;
    int kinColID      = 15;
    int stepType      = 0;

    ptCldColor::Ptr ptrPtCldObj{new ptCldColor};
    ptCldColor::Ptr ptrPtCldUnexp{new ptCldColor};
    std::vector<double> kinViewsphere = {0,0,0};
    HAFStVec1 stVecCreator;
    stVecCreator.setGridDim(5);
    stVecCreator.setMaintainScale(true);

    for(int i = 0; i < data.size(); i++){
      if(i%100 == 0) std::cout << i+1 << "/" << data.size() <<" completed." << std::endl;
      // Generate State Vector only if direction is not -1
      if(std::atoi(data[i][dirColID-1].c_str()) != -1){
        stepType = std::atoi(data[i][stepTypeColID-1].c_str());
        readPointCloud(ptrPtCldObj,directory,data[i][pathColID-1],1);
        readPointCloud(ptrPtCldUnexp,directory,data[i][pathColID-1],2);
        kinViewsphere[0] = std::atof(data[i][kinColID-1+(stepType-1)*3].c_str());
        kinViewsphere[1] = std::atof(data[i][kinColID-1+1+(stepType-1)*3].c_str());
        kinViewsphere[2] = std::atof(data[i][kinColID-1+2+(stepType-1)*3].c_str());

        stVecCreator.setInput(ptrPtCldObj,ptrPtCldUnexp,kinViewsphere);
        stVecCreator.calculate();
        // stVecCreator.print();
        stVecCreator.saveToCSV(fout,data[i][pathColID-1],data[i][dirColID-1]);
      }
    }
  }

  fout.close();
  std::cout << "State Vectors saved to " << directory+newCsvFile << ".\n";
}
