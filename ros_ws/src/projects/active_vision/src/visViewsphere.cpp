#include <active_vision/toolVisualization.h>
#include <active_vision/toolViewPointCalc.h>
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <chrono>
#include <boost/make_shared.hpp>

void help(){
  std::cout << "******* Viewsphere Visualizer Help *******" << std::endl;
  std::cout << "Use numpad/normal keys to go to a direction N(1),NE(2),E(3),SE(4),S(5),SW(6),W(7),NW(8)" << std::endl;
  std::cout << "Use numpad/normal key 0 to reset." << std::endl;
  std::cout << "Use space to switch between modes." << std::endl;
  std::cout << "*******" << std::endl;
}

int main(int argc, char** argv){
  help();

  std::vector<double> pose={1.4,M_PI,45*M_PI/180};
  std::vector<double> temp=pose;
  std::vector<double> currPose=pose;
  pcl::PointXYZ table,a1;
  table.x = 1.5; table.y = 0; table.z = 1;

  // Setting up the point cloud visualizer
  ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer")); std::vector<int> vp;
  setupViewer(viewer, 1, vp);
  keyboardEvent keyPress(viewer,2); keyPress.help();

  addViewsphere(viewer,vp[0],table,pose[0],true);

  viewer->addCube(table.x-0.25,table.x+0.25,
                  table.y-0.50,table.y+0.50,
                  table.z-0.01,table.z,0.9,0.8,0.4,"Table",vp[0]);

  viewer->addText("8(NW)\n7(W)\n6(SW)",5,70,20,1,0,0,"Key1",vp[0]);
  viewer->addText("1(N)\n0(Reset)\n5(S)",70,70,20,1,0,0,"Key2",vp[0]);
  viewer->addText("2(NE)\n3(E)\n4(SE)",160,70,20,1,0,0,"Key3",vp[0]);

  viewer->initCameraParameters();
  viewer->setCameraPosition(0,0,5,table.x,table.y,table.z,0,0,1);

  while(keyPress.ok){
    // Adding the Kinect position
    if(keyPress.dir >=1 && keyPress.dir <= 8)
      currPose = calcExplorationPose(currPose,keyPress.dir,keyPress.mode);
    else if(keyPress.dir == 0)
      currPose = pose;

    a1 = sphericalToCartesian(currPose,table);
    viewer->addSphere(a1,0.04,0,1,0,"Sphere",vp[0]);

    viewer->addText("Polar : "+std::to_string(round(currPose[1]*180/M_PI)),5,5,25,1,0,0,"Polar",vp[0]);
    viewer->addText("Azhimuthal : "+std::to_string(round(currPose[2]*180/M_PI)),5,35,25,1,0,0,"Azhi",vp[0]);
    viewer->addText("Mode " +std::to_string(keyPress.mode)+" (Space to switch)",5,140,20,1,0,0,"Mode",vp[0]);

    keyPress.called = false;
    while(!viewer->wasStopped() && keyPress.called==false){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
    viewer->resetStoppedFlag();
    viewer->removeShape("Mode",vp[0]);
    viewer->removeShape("Sphere",vp[0]);
    viewer->removeShape("Polar",vp[0]);
    viewer->removeShape("Azhi",vp[0]);
  }
}
