#include <active_vision/environment.h>
#include <active_vision/toolVisualization.h>

// ******************** SET OF FUNCTIONS FOR TESTING ENVIRONMENT ********************

// 1: A test function spawn and delete objects in gazebo
void testSpawnDeleteObj(environment &av){
  std::cout << "*** In object spawn and delete testing function ***" << std::endl;
  int flag = 0;
  for(auto data: av.objectDict){
    if(data.second.fileName.substr(0,3) != "YCB") continue;
    for(int j = 0; j < data.second.nPoses; j++) {
      av.spawnObject(data.second.ID,j,0);
      printf("Object %d/%d with configuration %d/%d spawned. Enter any key to continue. ",
             data.second.ID,int(av.objectDict.size()),
             j+1,int(data.second.nPoses));
      std::cin >> flag;
      av.deleteObject(data.second.ID);
      boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    }
  }
  std::cout << "*** End ***" << std::endl;
}

// 2: A test function to load and update gripper
void testGripper(ros::NodeHandle *nh){
  std::cout << "*** In gripper testing function ***" << std::endl;

  ros::ServiceClient gripperVisClient;
  gripperVisClient = nh->serviceClient<std_srvs::Empty> ("/graspSynthesis/viewGripper");
  std_srvs::Empty emptyMsg;
  gripperVisClient.call(emptyMsg);
  
  std::cout << "*** End ***" << std::endl;
}

// 3: A test function to check if the realsense / franka movement functions are working
void testMovement(environment &av){
  std::cout << "*** In camera movement testing function ***" << std::endl;
  int flag = 0;
  av.spawnObject(2,1,0);
  bool res;
  do {
    std::cout << "Enter your choice 1:Franka cartesian, 2:Camera viewsphere, 0:Exit : "; std::cin >> flag;
    av.moveFrankaHome();
    if (flag == 1) {
      std::vector<double> pose(6);
      double addnlPitch;
      std::cout << "Enter gripper pose data (Grasp Pose Data Type)" << std::endl;
      std::cout << "X : ";      std::cin >> pose[0];
      std::cout << "Y : ";      std::cin >> pose[1];
      std::cout << "Z : ";      std::cin >> pose[2];
      std::cout << "Roll : ";   std::cin >> pose[3];
      std::cout << "Pitch : ";  std::cin >> pose[4];
      std::cout << "Yaw : ";    std::cin >> pose[5];
      std::cout << "Addnl Pitch: "; std::cin >> addnlPitch;

      Eigen::Affine3f tfGrasp = pcl::getTransformation(pose[0],pose[1],
                                                       pose[2],pose[3],
                                                       pose[4],pose[5])*
                                pcl::getTransformation(0,0,0,0,addnlPitch,0)*
                                pcl::getTransformation(0.0,0,-0.0447-av.fingerZOffset,0,0,0)*
                                pcl::getTransformation(0,0,0,0,-M_PI/2,-M_PI);
      // std::cout << tfGrasp(0,0) << "," << tfGrasp(0,1) << "," <<tfGrasp(0,2) << std::endl;
      // std::cout << tfGrasp(1,0) << "," << tfGrasp(1,1) << "," <<tfGrasp(1,2) << std::endl;
      // std::cout << tfGrasp(2,0) << "," << tfGrasp(2,1) << "," <<tfGrasp(2,2) << std::endl;
      // if(tfGrasp(2,2) < 0)      tfGrasp = tfGrasp*pcl::getTransformation(0,0,0,M_PI,0,0);

      geometry_msgs::Pose pDummy;
      res = av.moveFranka(tfGrasp.matrix(),"JOINT",false,true,pDummy);
    } else if(flag == 2){
      std::vector<double> pose(3);
      std::cout << "Enter viewsphere co-ordinates with centre at (" <<
                    av.tableCentre[0] << "," <<
                    av.tableCentre[1] << "," <<
                    av.tableCentre[2] << ")" << std::endl;
      std::cout << "R (Radius) : ";                             std::cin >> pose[0];
      std::cout << "Phi (Azhimuthal Angle) (0->2*PI) : ";       std::cin >> pose[1];
      std::cout << "Theta (Polar Angle) (0->PI/2): ";           std::cin >> pose[2];

      res = av.moveCameraViewsphere(pose);
    }
    std::cout << "Camera moved : " << std::boolalpha << res << std::endl;
  } while(flag != 0);
  av.deleteObject(2);
  std::cout << "*** End ***" << std::endl;
}

// 4: A test function to move the camera in a viewsphere continuously
void testMoveCameraInViewsphere(environment &av){
  std::cout << "*** In Camera move in viewsphere testing function ***" << std::endl;
  float rad;
  std::cout << "Viewsphere radius : "; std::cin >> rad;
  av.spawnObject(2,1,0);
  std::vector<int> data;
  for (double j = 0; j < 2*M_PI;) {
    av.moveCameraViewsphere({rad,M_PI,0},false);
    for (double i = 0; i <= 75.0/180.0*M_PI;) {
      if(av.simulationMode == "SIMULATION"){
        av.moveCameraViewsphere({rad,j,i});
      }
      else if(av.simulationMode == "FRANKASIMULATION"){
        std::cout << "Azhimuthal\t" << round(j/M_PI*180) << "\tPolar\t" << round(i/M_PI*180);
        bool res = av.moveCameraViewsphere({rad,j,i},false);
        std::cout << "\t" << res << std::endl;
        data.push_back(res);
        if(res == false) av.moveCameraViewsphere({rad,M_PI,0},false);
      }

      i += M_PI/18; // 10 degree increment
    }
    j += 2*M_PI/16; // 22.5 degree increment
  }
  if(av.simulationMode == "FRANKASIMULATION"){
    for(auto i : data) std::cout << i << ",";
    std::cout << std::endl;
  }
  av.deleteObject(2);
  std::cout << "*** End ***" << std::endl;
}

// 5: A test function to check if the "readCamera" function is working
void testCameraRead(environment &av, int objID, int flag){
  std::cout << "*** In camera data read testing function ***" << std::endl;
  av.spawnObject(objID,0,0);

  int row; int col;
  std::cout << "Enter pixel value to print data for" << std::endl;
  std::cout << "Row (0-479) : "; std::cin >> row;
  std::cout << "Col (0-639) : "; std::cin >> col;
  av.readCamera();

  std::cout << "Printing values for pixel ( " << row << " , " << col << " )"<< std::endl;
  std::cout << "PCD (XYZRGB) : " << av.ptrPtCldLast->points.at(row*(av.ptrPtCldLast->width)+col) << std::endl;
  // NOT USED (JUST FOR REFERENCE)
  /*std::cout << "Color (BGR) : " << av.ptrRgbLast->image.at<cv::Vec3b>(row,col) << std::endl;
  std::cout << "Depth (Z) : " << av.ptrDepthLast->image.at<float>(row,col) << std::endl;*/

  if (flag==1) {
    // Setting up the point cloud visualizer
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer")); std::vector<int> vp;
    setupViewer(viewer, 1, vp);
    viewer->setCameraPosition(0,0,-1,0,0,1,0,-1,0);

    // Adding the point cloud
    addRGB(viewer,av.cPtrPtCldLast,"Raw Data",vp[0]);

    std::cout << "Close window to continue." << std::endl;
    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
    // NOT USED (JUST FOR REFERENCE)
    /*cv::imshow("Color Feed", av.ptrRgbLast->image);
    cv::imshow("Depth Feed", av.ptrDepthLast->image);
    cv::waitKey(0);*/
  }
  av.deleteObject(objID);
  std::cout << "*** End ***" << std::endl;
}

// 6: A test function to check fusing of data
void testPtCldFuse(environment &av, int objID, int flag){
  std::cout << "*** In point cloud data fusion testing function ***" << std::endl;
  av.spawnObject(objID,0,0);

  // Setting up the point cloud visualizer
  ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer")); std::vector<int> vp;
  setupViewer(viewer, 4, vp);
  keyboardEvent keyPress(viewer,1);
  viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

  // Camera positions
  std::vector<std::vector<double>> camPoses;
  camPoses = {{av.viewsphereRad,-M_PI,M_PI/7},
              {av.viewsphereRad,-M_PI-M_PI/4,M_PI/4},
              {av.viewsphereRad,-M_PI+M_PI/4,M_PI/4}};
  for (int i = 0; i < 3; i++) {
    if(av.simulationMode == "FRANKA") av.moveFrankaHome();
    av.moveCameraViewsphere(camPoses[i]);
    av.readCamera();
    av.fuseLastData();
    av.dataExtract();
    if (flag == 1){
      addRGB(viewer,av.cPtrPtCldEnv,"Fuse "+std::to_string(i),vp[i]);
    }
  }
  if(av.simulationMode == "FRANKA") av.moveFrankaHome();
  if (flag == 1){
    std::cout << "Close viewer to continue." << std::endl;
    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
  av.deleteObject(objID);
  std::cout << "*** End ***" << std::endl;
}

// 7: A test function to extract table and object data
void testDataExtract(environment &av, int objID, int flag){
  std::cout << "*** In table and object extraction testing function ***" << std::endl;
  av.spawnObject(objID,0,0);

  // Camera positions
  std::vector<std::vector<double>> camPoses;
  camPoses = {{av.viewsphereRad,-M_PI,M_PI/7},
              {av.viewsphereRad,-M_PI-M_PI/4,M_PI/4},
              {av.viewsphereRad,-M_PI+M_PI/4,M_PI/4}};
  for (int i = 0; i < 3; i++){
    if(av.simulationMode == "FRANKA") av.moveFrankaHome();
    av.moveCameraViewsphere(camPoses[i]);
    av.readCamera();
    av.fuseLastData();
    av.dataExtract();
  }
  if(av.simulationMode == "FRANKA") av.moveFrankaHome();
  av.editMoveItCollisions("OBJECT","ADD");
  if(flag==1){
    // Setting up the point cloud visualizer
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer")); std::vector<int> vp;
    setupViewer(viewer, 2, vp);
    viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

    // addRGBN(viewer,av.cPtrPtCldObject,normal,"Object",vp[1]);
    std::cout << "Showing the table and object extacted. Close viewer to continue" << std::endl;

    // ADding the point clouds
    addRGB(viewer,av.cPtrPtCldTable,"Table",vp[0]);
    addRGB(viewer,av.cPtrPtCldObject,"Object",vp[1]);

    while(!viewer->wasStopped()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
  av.editMoveItCollisions("OBJECT","REMOVE");
  av.deleteObject(objID);
  std::cout << "*** End ***" << std::endl;
}

// 8: A test function to generate unexplored point cloud
void testGenUnexpPtCld(environment &av, int objID, int flag){
  std::cout << "*** In unexplored point cloud generation testing function ***" << std::endl;
  av.spawnObject(objID,0,0);

  std::vector<std::vector<double>> camPoses;
  camPoses = {{av.viewsphereRad,-M_PI,M_PI/6},
              {av.viewsphereRad,-M_PI-M_PI/4,M_PI/7},
              {av.viewsphereRad,-M_PI+M_PI/4,M_PI/7}};
  for (int i = 0; i < 3; i++){
    av.moveCameraViewsphere(camPoses[i]);
    av.readCamera();
    av.fuseLastData();
  }
  av.dataExtract();
  av.genUnexploredPtCld();
  if(av.simulationMode == "FRANKA") av.moveFrankaHome();
  if(flag==1){
    // Setting up the point cloud visualizer
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer")); std::vector<int> vp;
    setupViewer(viewer, 1, vp);
    viewer->setCameraPosition(2,1,3,-1,-1,-1,-1,-1,1);

    // Adding the point clouds
    addRGB(viewer,av.cPtrPtCldObject,"Object",vp[0]);
    addRGB(viewer,av.cPtrPtCldUnexp,"Unexplored pointcloud",vp[0]);
    std::cout << "Showing the object extacted and unexplored point cloud generated. Close viewer to continue" << std::endl;
    while (!viewer->wasStopped()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
  av.deleteObject(objID);
  std::cout << "*** End ***" << std::endl;
}

// 9: A test function to update unexplored point cloud
void testUpdateUnexpPtCld(environment &av, int objID, int flag){
  std::cout << "*** In unexplored point cloud update testing function ***" << std::endl;
  av.spawnObject(objID,0,0);

  // Setting up the point cloud visualizer
  ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer")); std::vector<int> vp;
  setupViewer(viewer, 4, vp);
  viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

  // Camera positions
  std::vector<std::vector<double>> camPoses;
  camPoses = {{av.viewsphereRad,-M_PI,M_PI/7},
              {av.viewsphereRad,-M_PI-M_PI/4,M_PI/4},
              {av.viewsphereRad,-M_PI+M_PI/4,M_PI/4}};
  for (int i = 0; i < 3; i++){
    if(av.simulationMode == "FRANKA") av.moveFrankaHome();
    av.moveCameraViewsphere(camPoses[i]);
    av.readCamera();
    av.fuseLastData();
    av.dataExtract();
    if (i==0){
      av.genUnexploredPtCld();
    }
    av.updateUnexploredPtCld();
    if (flag == 1){
      addRGB(viewer,av.cPtrPtCldObject,"Env "+std::to_string(i),vp[i]);
      addRGB(viewer,av.ptrPtCldUnexp,"Unexp "+std::to_string(i),vp[i]);
    }
  }
  if(av.simulationMode == "FRANKA") av.moveFrankaHome();
  if (flag == 1){
    std::cout << "Close viewer to continue." << std::endl;
    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
  av.deleteObject(objID);
  std::cout << "*** End ***" << std::endl;
}

// 10: A test function to check the grasp and collision check algorithm with object and grasp points
void testComplete(environment &av, int objID, int flag, int write){
  std::cout << "*** In overall testing function ***" << std::endl;
  int ctrGrasp;

  std::ofstream outfile;
  if(write == 1){
    outfile.open("resultsAV.txt", std::ios_base::app); // append instead of overwrite
    outfile << std::endl;
  }

  std::chrono::high_resolution_clock::time_point start[3], end[3];
  double elapsed[3];

  av.spawnObject(objID,0,0);  

  // camera poses
  std::vector<std::vector<double>> camPoses;
  camPoses = {{av.viewsphereRad,-M_PI,M_PI/7},
              {av.viewsphereRad,-M_PI-M_PI/4,M_PI/4},
              {av.viewsphereRad,-M_PI+M_PI/4,M_PI/4}};

  start[0] = std::chrono::high_resolution_clock::now();
  start[1] = std::chrono::high_resolution_clock::now();
  // Read nViepoints and fuse them and update unexplord point cloud
  for (int i = 0; i < 3; i++){
    if(av.simulationMode == "FRANKA") av.moveFrankaHome();
    av.moveCameraViewsphere(camPoses[i]);
    av.readCamera();
    av.fuseLastData();
    av.dataExtract();
    if(i==0) av.genUnexploredPtCld();
    av.updateUnexploredPtCld();
  }
  if(av.simulationMode == "FRANKA") av.moveFrankaHome();
  end[1] = std::chrono::high_resolution_clock::now();

  std::cout << "Number of points in object point cloud : " << av.ptrPtCldObject->points.size() << std::endl;
  std::cout << "Number of points in unexplored cloud : " << av.ptrPtCldUnexp->points.size() << std::endl;

  start[2] = std::chrono::high_resolution_clock::now();

  // Grasp synthesis
  av.graspsynthesis();
  ctrGrasp = av.nGrasps;
  std::cout << "Number of grasps found : " << ctrGrasp << std::endl;
  end[2] = std::chrono::high_resolution_clock::now();

  if(av.graspID != -1){
    av.updateGripper(av.graspID,0);    // Only for visulization purpose
    std::cout << "Selected Grasp ID : " << av.graspID << std::endl;
    std::cout << "Selected Grasp Quality : " << av.graspData.quality << std::endl;
    std::cout << "graspData.pose = {" << av.graspData.pose[0] << "," <<
                                         av.graspData.pose[1] << "," <<
                                         av.graspData.pose[2] << "," <<
                                         av.graspData.pose[3] << "," <<
                                         av.graspData.pose[4] << "," <<
                                         av.graspData.pose[5] << "};" << std::endl;
    std::cout << "graspData.addnlPitch = " << av.graspData.addnlPitch << ";" << std::endl;
    std::cout << "graspData.gripperWidth = " << av.graspData.gripperWidth << ";" << std::endl;
  }else{
    std::cout << "No grasp found" << std::endl;
  }

  end[0] = std::chrono::high_resolution_clock::now();

  elapsed[0] = (std::chrono::duration_cast<std::chrono::milliseconds>(end[0] - start[0])).count();
  elapsed[1] = (std::chrono::duration_cast<std::chrono::milliseconds>(end[1] - start[1])).count();
  elapsed[2] = (std::chrono::duration_cast<std::chrono::milliseconds>(end[2] - start[2])).count();

  std::cout << std::endl << "Printing out the timings for each section (sec) :" << std::endl;
  std::cout << "Overall Timing = " << elapsed[0]/1000 << std::endl;
  std::cout << "(Move Camera + Read Camera +" << std::endl <<
               "Fuse Data + Data Extract +" << std::endl <<
               "Update Enexp PtCld) x 4 = " << elapsed[1]/1000 << std::endl;
  std::cout << "Grasp Synthesis = " << elapsed[2]/1000 << std::endl;

  if(write == 1){
    outfile << objID+1 << ","
            << camPoses.size() << ","
            << av.voxelGridSizeUnexp << ","
            << av.ptrPtCldObject->points.size() << ","
            << av.ptrPtCldUnexp->points.size() << ","
            << ctrGrasp << ","
            << av.graspID << ","
            << av.graspData.quality << ","
            << elapsed[0]/1000 << ","
            << elapsed[1]/1000 << ","
            << elapsed[2]/1000;
  }

  if(flag == 1){
    // Setting up the point cloud visualizer
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer")); std::vector<int> vp;
    setupViewer(viewer, 2, vp);
    keyboardEvent keyPress(viewer,1);
    viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

    addRGB(viewer,av.ptrPtCldEnv,"Environment",vp[0]);
    addRGB(viewer,av.ptrPtCldObject,"Object",vp[1]);

    pcl::PointXYZRGB centroidObj; Eigen::Vector4f temp1;
    pcl::compute3DCentroid(*av.ptrPtCldObject, temp1);
    centroidObj.x = temp1[0]; centroidObj.y = temp1[1]; centroidObj.z = temp1[2];
    viewer->addSphere<pcl::PointXYZRGB>(centroidObj,0.0050,0.0,1.0,0.0,"Centroid0",vp[0]);
    viewer->addSphere<pcl::PointXYZRGB>(centroidObj,0.0050,0.0,1.0,0.0,"Centroid1",vp[1]);

    for (int i = 0; i < av.ptrPtCldUnexp->size(); i++) {
      av.ptrPtCldUnexp->points[i].r = 0;
      av.ptrPtCldUnexp->points[i].b = 200;
      av.ptrPtCldUnexp->points[i].g = 0;
    }
    addRGB(viewer,av.ptrPtCldUnexp,"Unexplored",vp[1]);

    if(av.graspID == -1){
      std::cout << "No grasp orientation for the grasp points found." << std::endl;
      std::cout << "Showing the object (red), collision check points (blue). Close viewer to continue" << std::endl;
    }else{
      addRGB(viewer,av.ptrPtCldGripper,"Gripper",vp[0]);
      addRGB(viewer,av.ptrPtCldGripper,"Gripper1",vp[1]);
      std::cout << "Showing the object (red), collision check points (blue), selected gripper position (black). Close viewer to continue" << std::endl;
    }

    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }

  if(av.simulationMode == "FRANKA" && av.graspID != -1) av.graspObject(av.graspData);

  av.deleteObject(objID);
  if(write == 1) outfile.close();
  std::cout << "*** End ***" << std::endl;
}

// 11: A test function to check rollback feature
void testSaveRollback(environment &av, int objID, int flag){
  std::cout << "*** In save and rollback testing function ***" << std::endl;
  av.spawnObject(objID,0,0);  

  double step = 20*M_PI/180;
  std::vector<std::vector<double>> directions = {{ 00,-step},{ step,-step},
                                                 { step, 00},{ step, step},
                                                 { 00, step},{-step, step},
                                                 {-step, 00},{-step,-step}};

  std::vector<double> camPose = {av.viewsphereRad,-M_PI,M_PI/8};
  av.moveCameraViewsphere(camPose);
  av.readCamera();
  av.fuseLastData();
  av.dataExtract();
  av.genUnexploredPtCld();
  av.updateUnexploredPtCld();
  av.graspsynthesis();
  av.saveConfiguration("Base");

  if(flag == 1){
    // Setting up the point cloud visualizer
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer")); std::vector<int> vp;
    setupViewer(viewer, 2, vp);
    viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

    addRGB(viewer,av.ptrPtCldEnv,"Environment",vp[0]);
    addRGB(viewer,av.ptrPtCldObject,"Object",vp[1]);
    // rbgNormalVis(viewer,av.ptrPtCldObject,av.ptrObjNormal,"Object",vp[1]);

    for (int i = 0; i < av.ptrPtCldUnexp->size(); i++) {
      av.ptrPtCldUnexp->points[i].r = 0;
      av.ptrPtCldUnexp->points[i].b = 200;
      av.ptrPtCldUnexp->points[i].g = 0;
    }
    addRGB(viewer,av.ptrPtCldUnexp,"Unexplored",vp[1]);

    if(av.graspID != -1){
      av.updateGripper(av.graspID,0);    // Only for visulization purpose
      addRGB(viewer,av.ptrPtCldGripper,"Gripper",vp[0]);
      addRGB(viewer,av.ptrPtCldGripper,"Gripper1",vp[1]);
    }
    std::cout << "Close viewer to goto next direction." << std::endl;
    while(!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }

  for(int i = 0; i < 8; i++){
    av.rollbackConfiguration(0);

    camPose = av.lastCameraPoseViewsphere;
    camPose[1] += directions[i][0];
    camPose[2] += directions[i][1];
    av.moveCameraViewsphere(camPose);

    av.readCamera();
    av.fuseLastData();
    av.dataExtract();
    av.updateUnexploredPtCld();
    av.graspsynthesis();

    av.saveConfiguration("Direction "+std::to_string(i+1));

    if(flag == 1){
      // Setting up the point cloud visualizer
      ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
      int vp[2] = {};
      viewer->initCameraParameters();
      viewer->createViewPort(0.0,0.0,0.5,1.0,vp[0]);
      viewer->createViewPort(0.5,0.0,1.0,1.0,vp[1]);
      viewer->addCoordinateSystem(1.0);
      viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);
      // viewer->removeAllPointClouds(vp[0]);
      // viewer->removeAllPointClouds(vp[1]);

      addRGB(viewer,av.ptrPtCldEnv,"Environment",vp[0]);
      addRGB(viewer,av.ptrPtCldObject,"Object",vp[1]);
      // rbgNormalVis(viewer,av.ptrPtCldObject,av.ptrObjNormal,"Object",vp[1]);

      for (int i = 0; i < av.ptrPtCldUnexp->size(); i++) {
        av.ptrPtCldUnexp->points[i].r = 0;
        av.ptrPtCldUnexp->points[i].b = 200;
        av.ptrPtCldUnexp->points[i].g = 0;
      }
      addRGB(viewer,av.ptrPtCldUnexp,"Unexplored",vp[1]);

      if(av.graspID != -1){
        av.updateGripper(av.graspID,0);    // Only for visulization purpose
        addRGB(viewer,av.ptrPtCldGripper,"Gripper",vp[0]);
        addRGB(viewer,av.ptrPtCldGripper,"Gripper1",vp[1]);
      }
      std::cout << "Close viewer to goto next direction." << std::endl;
      while (!viewer->wasStopped ()){
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds(100000));
      }
    }
  }

  av.deleteObject(objID);
  std::cout << "*** End ***" << std::endl;
}

// 12: Testing gripper open close
void testGripperOpenClose(environment &av){
  std::cout << "*** In gripper open close testing function ***" << std::endl;
  if(av.simulationMode != "FRANKASIMULATION"){
    std::cout << "Incorrect simulation mode... Closing" << std::endl;
    std::cout << "*** End ***" << std::endl;
    return;
  }

  av.spawnObject(2,1,0);
  av.moveCameraViewsphere({0.5,M_PI,0});
  franka_pos_grasping_gazebo::GripPos grasp;

  float width = 0;
  do {
    std::cout << "Enter gripper width (cms) (-1 to exit) : "; std::cin >> width;
    if (width != -1) {
      grasp.request.finger_pos = width/100.0;
      av.gripperPosClient.call(grasp);
      ros::Duration(2).sleep();
    }
  } while(width != -1);

  av.deleteObject(2);

  std::cout << "*** End ***" << std::endl;
}

// 13: Testing object pickup
void testObjectPickup(environment &av, int objID){
  std::cout << "*** In object pickup function ***" << std::endl;
  if(av.simulationMode == "SIMULATION"){
    std::cout << "Incorrect simulation mode... Closing" << std::endl;
    std::cout << "*** End ***" << std::endl;
    return;
  }

  graspPoint graspData;
  if(objID == 2){
    {graspData.pose = {0.451718,7.93952e-06,0.0479308,-1.5708,1.57031,1.5708};}
    graspData.addnlPitch = 1.5708;
    graspData.gripperWidth = 0.0475006;
  }else if(objID == 7){
    {graspData.pose = {0.466138,0.0687767,0.0409107,-1.5708,1.5529,1.60483};}
    graspData.addnlPitch = 1.5708;
    graspData.gripperWidth = 0.0207021;
  }else if(objID == 8){
    {graspData.pose = {0.446149,-0.0264734,0.142481,-1.5708,1.37201,3.12775};}
    graspData.addnlPitch = 3.14159;
    graspData.gripperWidth = 0.0460051;
  }else if(objID == 9){
    {graspData.pose = {0.474497,-0.027417,0.0487696,1.5708,1.52962,2.15774};}
    graspData.addnlPitch = 1.5708;
    graspData.gripperWidth = 0.0075;
  }else if(objID == 10){
    {graspData.pose = {0.450668,0.00685909,0.115708,1.5708,1.5586,0.6982};}
    graspData.addnlPitch = 0;
    graspData.gripperWidth = 0.0606716;
  }else if(objID == 11){
    {graspData.pose = {0.447039,0.0451949,0.123947,1.5708,-1.56269,3.02644};}
    graspData.addnlPitch = 3.14159;
    graspData.gripperWidth = 0.0703883;
  }else if(objID == 12){
    {graspData.pose = {0.449776,0.000872621,0.073784,-1.5708,-1.56848,-1.68471};}
    graspData.addnlPitch = -1.5708;
    graspData.gripperWidth = 0.0726486;
  }else{
    {graspData.pose = {0.453253,-0.0020611,0.184009,-1.5708,-1.55776,-1.10351};}
    graspData.addnlPitch = -1.5708;
    graspData.gripperWidth = 0.0488689;

    if(av.simulationMode != "FRANKA") return;
  }
  graspData.pose[2] += av.tableCentre[2];

  av.spawnObject(objID,0,0);
  std::vector<double> camPose = {av.viewsphereRad,-M_PI,M_PI/8};
  av.moveCameraViewsphere(camPose);
  av.readCamera();
  av.fuseLastData();
  av.dataExtract();
  av.graspObject(graspData);
  av.deleteObject(objID);

  std::cout << "*** End ***" << std::endl;
}

// 14: Testing moveit collision
void testMoveItCollision(environment &av, int objID){
  std::cout << "*** In moveit collision testing function ***" << std::endl;

  if(av.simulationMode != "FRANKASIMULATION"){
    std::cout << "Incorrect simulation mode... Closing" << std::endl;
    std::cout << "*** End ***" << std::endl;
    return;
  }

  av.spawnObject(objID,0,0);

  // Camera positions to capture and fuse
  std::vector<std::vector<double>> camPoses;
  camPoses = {{av.viewsphereRad,-M_PI,M_PI/7},
              {av.viewsphereRad,-M_PI-M_PI/4,M_PI/4},
              {av.viewsphereRad,-M_PI+M_PI/4,M_PI/4}};
  for(int i = 0; i < 3; i++){
    av.moveCameraViewsphere(camPoses[i]);
    av.readCamera();
    av.fuseLastData();
    av.dataExtract();
  }

  int mode = 0;
  int object = 0;
  do{
    std::cout << "Enter your choice 1:ADD, 2:REMOVE, 0:Exit : "; std::cin >> mode;
    if(mode == 1){
      std::cout << "Choose object 1:TABLE, 2:OBJECT : "; std::cin >> object;
      if(object == 1)       av.editMoveItCollisions("TABLE","ADD");
      else if(object == 2)  av.editMoveItCollisions("OBJECT","ADD");
    }else if(mode == 2){
      std::cout << "Choose object 1:TABLE, 2:OBJECT : "; std::cin >> object;
      if(object == 1)       av.editMoveItCollisions("TABLE","REMOVE");
      else if(object == 2)  av.editMoveItCollisions("OBJECT","REMOVE");
    }
  }while(mode != 0);

  av.deleteObject(objID);
  std::cout << "*** End ***" << std::endl;
}

// 15: Testing moveit path constraints
void testMoveitPathConstraint(environment &av){
  std::cout << "*** In moveit path constraint testing function ***" << std::endl;

  if(av.simulationMode != "FRANKASIMULATION"){
    std::cout << "Incorrect simulation mode... Closing" << std::endl;
    std::cout << "*** End ***" << std::endl;
    return;
  }

  av.clearAllConstraints();
  geometry_msgs::Pose pTemp;
  int flag;
  bool res;
  std::cout << "Choices Available :\n"<<
               "\t1:Visibility Constraint\n"<<
               "\t2:Orientataion Constraint"<< std::endl;
  std::cin >> flag;
  if(flag == 1){
    av.addVisibilityConstraint();
    std::vector<double> pose(3);
    do{
      std::cout << "Enter your choice 1:Viewsphere, 0:Exit : "; std::cin >> flag;
      if(flag == 1){
        std::cout << "Enter viewsphere co-ordinates with centre at (" <<
                      av.tableCentre[0] << "," <<
                      av.tableCentre[1] << "," <<
                      av.tableCentre[2] << ")" << std::endl;
        std::cout << "R (Radius) : ";                         std::cin >> pose[0];
        std::cout << "Phi (Azhimuthal Angle) (0->2*PI) : ";   std::cin >> pose[1];
        std::cout << "Theta (Polar Angle) (0->PI/2): ";       std::cin >> pose[2];
        res = av.moveCameraViewsphere(pose);
      }
      std::cout << "Franka moved : " << std::boolalpha << res << std::endl;
    }while(flag != 0);
    av.clearAllConstraints();
  }else if(flag == 2){
    std::vector<double> pose(6);
    std::cout << "Enter orientation pose data" << std::endl;
    std::cout << "Roll : ";   std::cin >> pose[3];
    std::cout << "Pitch : ";  std::cin >> pose[4];
    std::cout << "Yaw : ";    std::cin >> pose[5];
    Eigen::Affine3f tf = pcl::getTransformation(0.5,0,0.5,pose[3],pose[4],pose[5]);
    av.moveFranka(tf.matrix(),"JOINT",false,true,pTemp);
    av.addOrientationConstraint(tf);
    do{
      std::cout << "Enter your choice 1:Cartesian, 0:Exit : "; std::cin >> flag;
      if(flag == 1){
        std::cout << "Enter pose data" << std::endl;
        std::cout << "X : ";      std::cin >> pose[0];
        std::cout << "Y : ";      std::cin >> pose[1];
        std::cout << "Z : ";      std::cin >> pose[2];
        tf = pcl::getTransformation(pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]);
        res = av.moveFranka(tf.matrix(),"JOINT",false,true,pTemp);
      }
      std::cout << "Franka moved : " << std::boolalpha << res << std::endl;
    }while(flag != 0);
    av.clearAllConstraints();
  }

  // std::vector<double> cartPoseStart = {0.5,0,0.1,0,0,0};
  // std::vector<double> cartPoseEnd = {0.5,0,0.5,0,0,0};
  // geometry_msgs::Pose pTemp;
  // Eigen::Affine3f tfStart = pcl::getTransformation(cartPoseStart[0],cartPoseStart[1],cartPoseStart[2],
  //                                                  cartPoseStart[3],cartPoseStart[4],cartPoseStart[5]);
  //
  // Eigen::Affine3f tfEnd = pcl::getTransformation(cartPoseEnd[0],cartPoseEnd[1],cartPoseEnd[2],
  //                                                cartPoseEnd[3],cartPoseEnd[4],cartPoseEnd[5]);
  //
  // pcl::Normal direction;
  // direction.normal_x = cartPoseEnd[0] - cartPoseStart[0];
  // direction.normal_y = cartPoseEnd[1] - cartPoseStart[1];
  // direction.normal_z = cartPoseEnd[2] - cartPoseStart[2];
  //
  // pcl::PointXYZRGB midPoint;
  // midPoint.x = (cartPoseEnd[0] + cartPoseStart[0])/2;
  // midPoint.y = (cartPoseEnd[1] + cartPoseStart[1])/2;
  // midPoint.z = (cartPoseEnd[2] + cartPoseStart[2])/2;
  //
  // float distance = sqrt(pow(direction.normal_x,2)+pow(direction.normal_y,2)+pow(direction.normal_z,2));
  //
  // Eigen::Affine3f tf = calcTfFromNormal(direction,midPoint);
  // Eigen::Matrix4f tfMat = tf.matrix();
  // Eigen::Quaternionf quat(tfMat.block<3,3>(0,0));
  //
  // moveit_planner::AddCollision collisionObjMsg;
  // collisionObjMsg.request.collObject.header.frame_id = "/world";
  // collisionObjMsg.request.collObject.id = "object";
  //

  // moveit_msgs::PositionConstraint posConstraint;
  //
  // posConstraint.header.frame_id = "/world";
  // posConstraint.link_name = "panda_hand";
  // posConstraint.weight = 1.0;
  //
  // shape_msgs::SolidPrimitive primitive;
  // primitive.type = primitive.BOX;
  // primitive.dimensions.resize(3);
  // primitive.dimensions[0] = distance+0.1;
  // primitive.dimensions[1] = 0.1;
  // primitive.dimensions[2] = 0.1;
  // posConstraint.constraint_region.primitives.push_back(primitive);
  //
  // geometry_msgs::Pose pose;
  // pose.position.x = midPoint.x;
  // pose.position.y = midPoint.y;
  // pose.position.z = midPoint.z;
  // pose.orientation.x = quat.x();
  // pose.orientation.y = quat.y();
  // pose.orientation.z = quat.z();
  // pose.orientation.w = quat.w();
  // posConstraint.constraint_region.primitive_poses.push_back(pose);
  //
  // constraintsMsg.request.constraints.position_constraints.push_back(posConstraint);
  //
  // collisionObjMsg.request.collObject.primitives.push_back(primitive);
  // collisionObjMsg.request.collObject.primitive_poses.push_back(pose);
  // collisionObjMsg.request.collObject.operation = collisionObjMsg.request.collObject.ADD;
  // av.collisionClient.call(collisionObjMsg);
  // int flag;
  // std::cout << "xxxx " ; std::cin >> flag;
  //
  // collisionObjMsg.request.collObject.operation = collisionObjMsg.request.collObject.REMOVE;
  // av.collisionClient.call(collisionObjMsg);
  //
  // av.moveFranka(tfStart.matrix(),"JOINT",false,true,pTemp);
  // ros::Duration(2).sleep();
  // std::cout << "xxxx " ; std::cin >> flag;
  // // av.addOrientationConstraint(tfEnd.matrix());
  // // av.setConstClient.call(constraintsMsg);
  //
  // av.moveFranka(tfEnd.matrix(),"JOINT",false,true,pTemp);
  // ros::Duration(2).sleep();
  // std::cout << "xxxx " ; std::cin >> flag;
  // av.clearAllConstraints();

  std::cout << "*** End ***" << std::endl;
}

// 16: Testing rotation and shaking test
void testRotAndShake(environment &av){
  std::cout << "*** In rotation and shaking test function ***" << std::endl;
  if(av.simulationMode == "SIMULATION"){
    std::cout << "Incorrect simulation mode... Closing" << std::endl;
    std::cout << "*** End ***" << std::endl;
    return;
  }

  av.moveFrankaHome();

  moveit_planner::SetVelocity velscale;
  velscale.request.velScaling = 1.0;
  av.velScalingClient.call(velscale);

  moveit_planner::SetJointWithTime setJointWithTimeMsg;
  setJointWithTimeMsg.request.joint_name = "panda_joint7";

  std::cout << "Rotation Test" << std::endl;
  for(int i = 0; i < 2; i++){
    setJointWithTimeMsg.request.joint_angle = 0.000; av.oneJointWithTimeClient.call(setJointWithTimeMsg);
    setJointWithTimeMsg.request.joint_angle = 1.570; av.oneJointWithTimeClient.call(setJointWithTimeMsg);
  }
  setJointWithTimeMsg.request.joint_angle = 0.785; av.oneJointWithTimeClient.call(setJointWithTimeMsg);

  std::cout << "Shaking Test" << std::endl;
  setJointWithTimeMsg.request.joint_name = "panda_joint5";
  for(int i = 0; i < 4; i++){
    setJointWithTimeMsg.request.joint_angle = -0.35; av.oneJointWithTimeClient.call(setJointWithTimeMsg);
    setJointWithTimeMsg.request.joint_angle =  0.35; av.oneJointWithTimeClient.call(setJointWithTimeMsg);
  }
  setJointWithTimeMsg.request.joint_angle =  0.00; av.oneJointWithTimeClient.call(setJointWithTimeMsg);

  std::cout << "*** End ***" << std::endl;

}

int main (int argc, char** argv){

  // Initialize ROS
  ros::init(argc, argv, "Environment_Testing");
  ros::NodeHandle nh;

  environment activeVision(&nh);
  // Delay to ensure all publishers and subscribers are connected
  boost::this_thread::sleep(boost::posix_time::milliseconds(500));

  int choice, objID;
  std::cout << "Available choices for test functions : " << std::endl;
  std::cout << "1  : Spawn and delete objects and its configurations on the table." << std::endl;
  std::cout << "2  : Load and view the gripper model." << std::endl;
  std::cout << "3  : Move the camera/franka to a custom position." << std::endl;
  std::cout << "4  : Continuously move the camera in a viewsphere with centre on the table." << std::endl;
  std::cout << "5  : Read and view the data from camera." << std::endl;
  std::cout << "6  : Read and fuse the data from different viewpoints." << std::endl;
  std::cout << "7  : Extract the table and object from point cloud." << std::endl;
  std::cout << "8  : Generate the initial unexplored pointcloud based on the object." << std::endl;
  std::cout << "9  : Update the unexplored pointcloud based on different viewpoints." << std::endl;
  std::cout << "10 : Grasp synthesis based on different viewpoints." << std::endl;
  std::cout << "11 : Store and rollback configurations." << std::endl;
  std::cout << "12 : Surface patch testing." << std::endl;
  std::cout << "13 : Gripper open close testing." << std::endl;
  std::cout << "14 : Testing object pickup." << std::endl;
  std::cout << "15 : Testing moveit collision add/remove." << std::endl;
  std::cout << "16 : Testing moveit constraint." << std::endl;
  std::cout << "17 : Rotation and shaking test." << std::endl;
  std::cout << "Enter your choice : "; cin >> choice;

  if(choice >= 5 && choice <= 14 && choice != 12){
    if(activeVision.simulationMode != "FRANKA"){
      std::cout << "YCB Objects available :" << std::endl;
      for(auto data: activeVision.objectDict){
        if(data.second.fileName.substr(0,3) == "YCB") data.second.printObjectInfo();
      }
      std::cout << "Enter your choice : "; std::cin>>objID;
    }else{
      objID = 1;
    }
  }

  switch(choice){
    case 1:
      testSpawnDeleteObj(activeVision);               break;
    case 2:
      testGripper(&nh);                               break;
    case 3:
      testMovement(activeVision);                     break;
    case 4:
      testMoveCameraInViewsphere(activeVision);       break;
    case 5:
      testCameraRead(activeVision,objID,1);           break;
    case 6:
      testPtCldFuse(activeVision,objID,1);            break;
    case 7:
      testDataExtract(activeVision,objID,1);          break;
    case 8:
      testGenUnexpPtCld(activeVision,objID,1);        break;
    case 9:
      testUpdateUnexpPtCld(activeVision,objID,1);     break;
    case 10:
      testComplete(activeVision,objID,1,0);           break;
    case 11:
      testSaveRollback(activeVision,objID,1);         break;
    case 12:
      testGripperOpenClose(activeVision);             break;
    case 13:
      testObjectPickup(activeVision,objID);           break;
    case 14:
      testMoveItCollision(activeVision,objID);        break;
    case 15:
      testMoveitPathConstraint(activeVision);         break;
    case 16:
      testRotAndShake(activeVision);                  break;
    default:
      std::cout << "Invalid choice." << std::endl;
  }
  // testCollisionDummy(activeVision,false,1);
}

/*
Notes:
-> POint cloud XYZRGB data type : std::vector< pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >
-> 640 elements in rach row of the matrix.
-> Transformation of CameraOpticalFrame wrt CameraGazeboFrame (RPY) - (-90 0 -90)
-> Working combo : Voxel Obj 0.01, Unexp voxel 0.02
*/
