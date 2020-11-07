#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include <fstream>
#include <chrono>
#include <boost/make_shared.hpp>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>

typedef pcl::visualization::PCLVisualizer ptCldVis;

#define MIN_ANGLE 20
#define MIN_ANGLE_RAD MIN_ANGLE*(M_PI/180.0)

int dir = 0;
bool called = false;
bool ok = true;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event){
  if(event.keyUp()){
    // std::cout << event.getKeySym() << std::endl;
    if      (event.getKeySym() == "KP_5") ::dir = 0;
    else if (event.getKeySym() == "KP_1") ::dir = 1;
    else if (event.getKeySym() == "KP_2") ::dir = 2;
    else if (event.getKeySym() == "KP_3") ::dir = 3;
    else if (event.getKeySym() == "KP_4") ::dir = 4;
    else if (event.getKeySym() == "KP_6") ::dir = 6;
    else if (event.getKeySym() == "KP_7") ::dir = 7;
    else if (event.getKeySym() == "KP_8") ::dir = 8;
    else if (event.getKeySym() == "KP_9") ::dir = 9;
    else if (event.getKeySym() == "5") ::dir = 0;
    else if (event.getKeySym() == "1") ::dir = 1;
    else if (event.getKeySym() == "2") ::dir = 2;
    else if (event.getKeySym() == "3") ::dir = 3;
    else if (event.getKeySym() == "4") ::dir = 4;
    else if (event.getKeySym() == "6") ::dir = 6;
    else if (event.getKeySym() == "7") ::dir = 7;
    else if (event.getKeySym() == "8") ::dir = 8;
    else if (event.getKeySym() == "9") ::dir = 9;
    else if (event.getKeySym() == "Escape") ::ok = false;
    ::called = true;
  }
}

pcl::PointXYZ sphericalToCartesian(pcl::PointXYZ &centre, std::vector<double> &pose){
  pcl::PointXYZ res;
  res.x = centre.x+pose[0]*sin(pose[2])*cos(pose[1]);
  res.y = centre.y+pose[0]*sin(pose[2])*sin(pose[1]);
  res.z = centre.z+pose[0]*cos(pose[2]);
  return(res);
}

//Check that the azimuthal angle is less than 90.
bool checkValidPose(std::vector<double> pose){
	return (pose[2] < (M_PI/2+.1));
}

std::vector<double> getOffset(int direction){
	std::vector<double> offset={0,0};
  switch (direction) {
    case 8: offset[0]=0;              offset[1]=-MIN_ANGLE_RAD;  break;
    case 9: offset[0]=MIN_ANGLE_RAD;  offset[1]=-MIN_ANGLE_RAD;  break;
    case 6: offset[0]=MIN_ANGLE_RAD;  offset[1]=0;               break;
    case 3: offset[0]=MIN_ANGLE_RAD;  offset[1]=MIN_ANGLE_RAD;   break;
    case 2: offset[0]=0;              offset[1]=MIN_ANGLE_RAD;   break;
    case 1: offset[0]=-MIN_ANGLE_RAD; offset[1]=MIN_ANGLE_RAD;   break;
    case 4: offset[0]=-MIN_ANGLE_RAD; offset[1]=0;               break;
    case 7: offset[0]=-MIN_ANGLE_RAD; offset[1]=-MIN_ANGLE_RAD;  break;
  }
	return offset;
}

std::vector<double> moveDirection(std::vector<double> start, int direction){
  std::vector<double> offset = getOffset(direction);
	std::vector<double> end = {start[0], start[1]+offset[0], start[2]+offset[1]};

  // Addressing the NW & NE scenario when polar angle goes from *ve to -ve
  if(end[2] < 0 && start[2] > 0) end[1] = start[1]-offset[0];

  // Polar angle 0 to 90 degree
  if(end[2] < 0){
    end[2] = -1*end[2];
    end[1] = end[1] + M_PI;
  }

  // Azhimuthal angle 0 to 360 degree
  end[1] = fmod(end[1],2*M_PI);
  if(end[1] < 0) end[1] += 2*M_PI;

	if(checkValidPose(end)) return end;
  else                    return start;
}

void help(){
  std::cout << "******* Viewsphere Visualizer Help *******" << std::endl;
  std::cout << "Use numpad/normal keys to go to a direction N(8),NE(9),E(6),SE(3),S(2),SW(1),W(4),NW(7)" << std::endl;
  std::cout << "Use numpad/normal key 5 to reset." << std::endl;
  std::cout << "*******" << std::endl;
}

int main(int argc, char** argv){
  help();

  std::vector<double> pose={1.4,M_PI,30*M_PI/180};
  std::vector<double> temp=pose;
  std::vector<double> currPose=pose;
  pcl::PointXYZ table,a1;
  table.x = 1.5; table.y = 0,1; table.z = 1;

  // Setting up the point cloud visualizer
  ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
  int vp=0;
  viewer->createViewPort(0.0,0.0,1.0,1.0,vp);
  viewer->addCoordinateSystem(1.0);
  viewer->setBackgroundColor(0.5,0.5,0.5,vp);
  viewer->registerKeyboardCallback(keyboardEventOccurred);

  // Adding possible viewspheres
  int vCtr = 0;
  for (int azimuthalAngle = 0; azimuthalAngle < 360; azimuthalAngle+=20){
    for (int polarAngle = 10; polarAngle <= 90; polarAngle+=20){
      temp[1] = azimuthalAngle*M_PI/180; temp[2] = polarAngle*M_PI/180;
      a1 = sphericalToCartesian(table,temp);
      viewer->addSphere(a1,0.02,0,0,1,"Sph_"+std::to_string(vCtr),vp); vCtr++;
    }
  }
  viewer->addSphere(table,pose[0],0,0,1,"Viewsphere",vp);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,"Viewsphere");

  viewer->addCube(table.x-0.25,table.x+0.25,
                  table.y-0.50,table.y+0.50,
                  table.z-0.01,table.z,0.9,0.8,0.4,"Table",vp);

  viewer->addText("7(NW)\n4(W)\n1(SW)",5,70,20,1,0,0,"Key1",vp);
  viewer->addText("8(N)\n5(Reset)\n2(S)",70,70,20,1,0,0,"Key2",vp);
  viewer->addText("9(NE)\n6(E)\n3(SE)",160,70,20,1,0,0,"Key3",vp);
  // viewer->addText("Azhimuthal : "+std::to_string(round(currPose[2]*180/M_PI)),5,35,25,1,0,0,"Azhi",vp);

  viewer->initCameraParameters();
  viewer->setCameraPosition(0,0,5,2,0,-1,1,0,2);

  while(::ok){
    // Adding the Kinect position
    if (::dir >=1 && ::dir <= 9 && ::dir!=5)
      currPose = moveDirection(currPose,::dir);
    else
      currPose = pose;
    a1 = sphericalToCartesian(table,currPose);
    viewer->addSphere(a1,0.04,0,1,0,"Sphere",vp);

    viewer->addText("Polar : "+std::to_string(round(currPose[1]*180/M_PI)),5,5,25,1,0,0,"Polar",vp);
    viewer->addText("Azhimuthal : "+std::to_string(round(currPose[2]*180/M_PI)),5,35,25,1,0,0,"Azhi",vp);
    ::called = false;
    while(!viewer->wasStopped() && ::called==false){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
    viewer->resetStoppedFlag();
    viewer->removeShape("Sphere",vp);
    viewer->removeShape("Polar",vp);
    viewer->removeShape("Azhi",vp);
  }
}
