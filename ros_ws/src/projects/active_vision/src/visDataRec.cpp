#include <active_vision/dataHandling.h>
#include <active_vision/testingModel_v1.h>

bool called = false;
int counter = 0;
bool ok = true;
bool skipTo = false;

std::map<int, std::string> dirLookup{{0, "Nil"},
                                     {1, "N"}, {2, "NE"},
																		 {3, "E"}, {4, "SE"},
																		 {5, "S"}, {6, "SW"},
																	 	 {7, "W"}, {8, "NW"}};

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event){
  if(event.keyUp()){
    // std::cout << event.getKeySym() << std::endl;
    ::counter = 0;
    if(event.getKeySym() == "Right") ::counter = 1;
    else if(event.getKeySym() == "Left") ::counter = -1;
    else if(event.getKeySym() == "Escape") ::ok = false;
    else if(event.getKeySym() == "z") ::skipTo = true;
    ::called = true;
  }
}

void help(){
  std::cout << "******* Recorded Data Visualizer Help *******" << std::endl;
  std::cout << "Arguments : [Directory] [CSV filename] [Type]" << std::endl;
  std::cout << "Directory : Directory where csv and pcd files are there (./DataRecAV/)" << std::endl;
  std::cout << "CSV filename : CSV file name (Data.csv)" << std::endl;
  std::cout << "Type : 1 (result.pcd only), 2 (obj,unexp,result.pcd)" << std::endl;
  std::cout << "*******" << std::endl;
}

int main(int argc, char** argv){
  if(argc != 4){
    std::cout << "ERROR. Incorrect number of arguments." << std::endl;
    help(); return(-1);
  }
  int type;
  type = std::atoi(argv[3]);
  if(type != 1 && type != 2){
    std::cout << "ERROR. Incorrect type." << std::endl;
    help(); return(-1);
  }

  std::string directory(argv[1]);
  std::string csvFile(argv[2]);
  std::vector<std::vector<std::string>> data;
  data = readCSV(directory+csvFile);

  int pathColID = 12;
  int dirColID = 13;
  int nStepColID = 14;
  int stepColID = 15;
  int nSteps;

  std::vector<double> pose={0,0,0};
  pcl::PointXYZ sphereCentre;
  pcl::PointXYZ table,a1,a2;
  table.x = 1.5; table.y = 0,1; table.z = 1;

  ptCldColor::Ptr ptrPtCldTemp{new ptCldColor};

  // Setting up the point cloud visualizer
  ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
  int vp[3] = {};
  if(type == 1){
    viewer->createViewPort(0.0,0.0,1.0,1.0,vp[2]);
  }else if(type == 2){
    viewer->createViewPort(0.0,0.5,0.5,1.0,vp[0]);
    viewer->createViewPort(0.5,0.5,1.0,1.0,vp[1]);
    viewer->createViewPort(0.25,0.0,0.75,0.5,vp[2]);
  }
  viewer->addCoordinateSystem(1.0);
  viewer->setCameraPosition(-2,0,7,2,0,-1,1,0,2);
  viewer->registerKeyboardCallback(keyboardEventOccurred);

  std::cout << "***Key Bindings***\n \u2192 , \u2190 : Move between data \n z : Goto a data number \n Esc : Exit." << std::endl;

  int i = 0;
  while(::ok){
    if (pathColID <= data[i].size()){
      if(type==2){
        readPointCloud(ptrPtCldTemp,directory,data[i][pathColID-1],1);
        rbgVis(viewer,ptrPtCldTemp,"Obj",vp[0]);
        readPointCloud(ptrPtCldTemp,directory,data[i][pathColID-1],2);
        rbgVis(viewer,ptrPtCldTemp,"Unexp",vp[1]);
      }
      readPointCloud(ptrPtCldTemp,directory,data[i][pathColID-1],3);
      rbgVis(viewer,ptrPtCldTemp,"Env",vp[2]);
      viewer->addText("Data No : "+std::to_string(i+1),5,5,25,1,0,0,"Data",vp[2]);
      viewer->addText("Direction : "+dirLookup[std::atoi(data[i][dirColID-1].c_str())],5,35,25,1,0,0,"Dir",vp[2]);

      nSteps = std::atoi(data[i][nStepColID-1].c_str());
      viewer->addText("nSteps : "+std::to_string(nSteps),5,65,25,1,0,0,"nSteps",vp[2]);
      for (int j = stepColID; j < stepColID+(nSteps-1)*3; j+=3){
        pose[0] = std::atof(data[i][j-1].c_str());
        pose[1] = std::atof(data[i][j-1+1].c_str());
        pose[2] = std::atof(data[i][j-1+2].c_str());
        a1.x = table.x+pose[0]*sin(pose[2])*cos(pose[1]);
        a1.y = table.y+pose[0]*sin(pose[2])*sin(pose[1]);
        a1.z = table.z+pose[0]*cos(pose[2]);

        pose[0] = std::atof(data[i][j-1+3].c_str());
        pose[1] = std::atof(data[i][j-1+4].c_str());
        pose[2] = std::atof(data[i][j-1+5].c_str());
        a2.x = table.x+pose[0]*sin(pose[2])*cos(pose[1]);
        a2.y = table.y+pose[0]*sin(pose[2])*sin(pose[1]);
        a2.z = table.z+pose[0]*cos(pose[2]);

        viewer->addArrow(a2,a1,0,1,0,false,std::to_string(j),vp[2]);
      }

      pose[0] = std::atof(data[i][stepColID-1].c_str());
      pose[1] = std::atof(data[i][stepColID-1+1].c_str());
      pose[2] = std::atof(data[i][stepColID-1+2].c_str());
      a1.x = table.x+pose[0]*sin(pose[2])*cos(pose[1]);
      a1.y = table.y+pose[0]*sin(pose[2])*sin(pose[1]);
      a1.z = table.z+pose[0]*cos(pose[2]);

      viewer->addSphere(a1,0.03,0,1,0,"Cam",vp[2]);

      int vCtr = 0;
      for (int azimuthalAngle = 0; azimuthalAngle < 360; azimuthalAngle+=10){
        for (int polarAngle = 5; polarAngle <= 90; polarAngle+=10){
          a1.x = table.x+pose[0]*sin(polarAngle*(M_PI/180.0))*cos(azimuthalAngle*(M_PI/180.0));
          a1.y = table.y+pose[0]*sin(polarAngle*(M_PI/180.0))*sin(azimuthalAngle*(M_PI/180.0));
          a1.z = table.z+pose[0]*cos(polarAngle*(M_PI/180.0));
          vCtr++;
          viewer->addSphere(a1,0.02,0,0,1,"Sph_"+std::to_string(vCtr),vp[2]);
        }
      }
      viewer->addSphere(table,pose[0],0,0,1,"Viewsphere",vp[2]);
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,"Viewsphere");

      ::called = false;
      while(!viewer->wasStopped() && ::called==false){
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds(100000));
      }

      viewer->resetStoppedFlag();
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();
    }else{
      std::cout << "No data found for i = " << i+1 << std::endl;
      ::ok = false;
    }
    if(::skipTo == true){
      ::skipTo = false;
      std::cout << "Enter the Data Number you want to go : "; std::cin >> i;
    }
    else{
      i += ::counter;
    }
    if(i < 0) i = 0;
    if(i > data.size()-1) i = data.size()-1;
  }
}
