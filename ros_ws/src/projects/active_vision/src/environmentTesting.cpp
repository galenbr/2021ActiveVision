#include <active_vision/environment.h>
#include <active_vision/toolVisualization.h>

// ******************** SET OF FUNCTIONS FOR TESTING ENVIRONMENT ********************

// 1: A test function spawn and delete objects in gazebo
void testSpawnDeleteObj(environment &av){
  std::cout << "*** In object spawn and delete testing function ***" << std::endl;
  int flag = 0;
  for(auto data: av.objectDict){
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

// 3: A test function to check if the "moveKinect" functions are working
void testKinectMovement(environment &av){
  std::cout << "*** In kinect movement testing function ***" << std::endl;
  int flag = 0;
  do {
    std::cout << "Enter your choice 1:Cartesian, 2:Viewsphere, 0:Exit : "; std::cin >> flag;
    if (flag == 1) {
      std::vector<double> pose(7);
      std::cout << "Enter kinect pose data" << std::endl;
      std::cout << "X : ";      std::cin >> pose[0];
      std::cout << "Y : ";      std::cin >> pose[1];
      std::cout << "Z : ";      std::cin >> pose[2];
      std::cout << "qX : ";    std::cin >> pose[3];
      std::cout << "qY : ";    std::cin >> pose[4];
      std::cout << "qZ : ";    std::cin >> pose[5];
      std::cout << "qW : ";    std::cin >> pose[6];

      av.moveKinectCartesian(pose);
      std::cout << "Kinect moved" << std::endl;

    } else if(flag == 2){
      std::vector<double> pose(3);
      std::cout << "Enter viewsphere co-ordinates with centre at (" <<
                    av.tableCentre[0] << "," <<
                    av.tableCentre[1] << "," <<
                    av.tableCentre[2] << ")" << std::endl;
      std::cout << "R (Radius) : ";                         std::cin >> pose[0];
      std::cout << "Phi (Azhimuthal Angle) (0->2*PI) : ";      std::cin >> pose[1];
      std::cout << "Theta (Polar Angle) (0->PI/2): ";    std::cin >> pose[2];

      av.moveKinectViewsphere(pose);
      std::cout << "Kinect moved" << std::endl;
    }
  } while(flag != 0);
  std::cout << "*** End ***" << std::endl;
}

// 4: A test function to move the kinect in a viewsphere continuously
void testMoveKinectInViewsphere(environment &av){
  std::cout << "*** In Kinect move in viewsphere testing function ***" << std::endl;
  for (double i = M_PI/2; i > 0;) {
    for (double j = 0; j < 2*M_PI;) {
      av.moveKinectViewsphere({1.0,j,i});
      j += 2*M_PI/10;
    }
    std::cout << (int)((M_PI/2-i)/(M_PI/2/10)+1)<< " out of 10 completed." << std::endl;
    i -= M_PI/2/10;
  }
  std::cout << "*** End ***" << std::endl;
}

// 5: A test function to check if the "readKinect" function is working
void testKinectRead(environment &av, int objID, int flag){
  std::cout << "*** In kinect data read testing function ***" << std::endl;
  av.spawnObject(objID,0,0);

  int row; int col;
  std::cout << "Enter pixel value to print data for" << std::endl;
  std::cout << "Row (0-479) : "; std::cin >> row;
  std::cout << "Col (0-639) : "; std::cin >> col;
  av.readKinect();

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
  viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

  // 4 kinect position to capture and fuse
  std::vector<std::vector<double>> kinectPoses = {{1.0,-M_PI,M_PI/4},
                                                  {1.0,-M_PI/2,M_PI/4},
                                                  {1.0,0,M_PI/4},
                                                  {1.0,M_PI/2,M_PI/4}};

  for (int i = 0; i < 4; i++) {
    av.moveKinectViewsphere(kinectPoses[i]);
    av.readKinect();
    av.fuseLastData();
    if (flag == 1){
      addRGB(viewer,av.cPtrPtCldEnv,"Fuse "+std::to_string(i),vp[i]);
    }
  }
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

  // 4 kinect position to capture and fuse
  std::vector<std::vector<double>> kinectPoses = {{1.0,-M_PI,M_PI/4},
                                                  {1.0,-M_PI/2,M_PI/4},
                                                  {1.0,0,M_PI/4},
                                                  {1.0,M_PI/2,M_PI/4}};
  for (int i = 0; i < 4; i++) {
    av.moveKinectViewsphere(kinectPoses[i]);
    av.readKinect();
    av.fuseLastData();
  }
  av.dataExtract();

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
  av.deleteObject(objID);
  std::cout << "*** End ***" << std::endl;
}

// 8: A test function to generate unexplored point cloud
void testGenUnexpPtCld(environment &av, int objID, int flag){
  std::cout << "*** In unexplored point cloud generation testing function ***" << std::endl;
  av.spawnObject(objID,0,0);

  std::vector<double> kinectPose = {1.0,-M_PI,M_PI/4};
  av.moveKinectViewsphere(kinectPose);
  av.readKinect();
  av.fuseLastData();
  av.dataExtract();
  av.genUnexploredPtCld();

  if(flag==1){
    // Setting up the point cloud visualizer
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer")); std::vector<int> vp;
    setupViewer(viewer, 1, vp);
    viewer->setCameraPosition(2,1,3,-1,-1,-1,-1,-1,1);

    // Adding the point clouds
    addRGB(viewer,av.cPtrPtCldObject,"Object",vp[0]);
    //addRGB(viewer,av.cPtrPtCldUnexp,"Unexplored pointcloud",vp[0]);
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

  // 4 kinect position to capture and fuse
  std::vector<std::vector<double>> kinectPoses = {{1.0,-M_PI,M_PI/4},
                                                  {1.0,-M_PI/2,M_PI/4},
                                                  {1.0,0,M_PI/4},
                                                  {1.0,M_PI/2,M_PI/4}};
  for (int i = 0; i < 4; i++) {
    av.moveKinectViewsphere(kinectPoses[i]);
    av.readKinect();
    av.fuseLastData();
    av.dataExtract();
    if (i==0){
      av.genUnexploredPtCld();
    }
    av.updateUnexploredPtCld();
    if (flag == 1){
      addRGB(viewer,av.cPtrPtCldEnv,"Env "+std::to_string(i),vp[i]);
      addRGB(viewer,av.ptrPtCldUnexp,"Unexp "+std::to_string(i),vp[i]);
    }
  }
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

// 2: A test function to load and update gripper
void testGripper(environment &av, int flag, float width){
  std::cout << "*** In gripper testing function ***" << std::endl;

  graspPoint graspTemp; // Creating a dummy grasp
  graspTemp.pose[2] = 0.0447+0.0584;
  av.graspsPossible.push_back(graspTemp);

  av.loadGripper();
  av.updateGripper(0,0);
  av.updateGripper(0,1);
  av.updateGripper(0,2);

  if (flag == 1) {
    // Setting up the point cloud visualizer
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer")); std::vector<int> vp;
    setupViewer(viewer, 1, vp);
    viewer->removeCoordinateSystem();
    viewer->setCameraPosition(0.5,0,0,-1,0,0,0,0,1);

    // Adding the point cloud
    addRGB(viewer,av.cPtrPtCldGripper,"Gripper",vp[0]);
    for (int i = 0; i < 3; i++) {
      viewer->addCube(av.minPtCol[i].x,av.maxPtCol[i].x,
                      av.minPtCol[i].y,av.maxPtCol[i].y,
                      av.minPtCol[i].z,av.maxPtCol[i].z,0.0,1.0,0.0,"Cube"+std::to_string(i),vp[0]);
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "Cube"+std::to_string(i));
    }
    for (int i = 3; i < 5; i++) {
      viewer->addCube(av.minPtCol[i].x,av.maxPtCol[i].x,
                      av.minPtCol[i].y,av.maxPtCol[i].y,
                      av.minPtCol[i].z,av.maxPtCol[i].z,1.0,0.0,0.0,"Cube"+std::to_string(i),vp[0]);
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "Cube"+std::to_string(i));
    }

    std::cout << "Showing the gripper and the bounding boxes for collision check. Close viewer to continue" << std::endl;

    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
  std::cout << "*** End ***" << std::endl;
}

// 10: Grasp synthesis test function
void testGraspsynthesis(environment &av, int objID, int flag){
  std::cout << "*** In grasp synthesis testing function ***" << std::endl;
  av.spawnObject(objID,0,0);

  // 4 kinect position
  std::vector<std::vector<double>> kinectPoses = {{1.0,-M_PI,M_PI/4},
                                                  {1.0,-M_PI/2,M_PI/4},
                                                  {1.0,0,M_PI/4},
                                                  {1.0,M_PI/2,M_PI/4}};

  for (int i = 0; i < 4; i++) {
    av.moveKinectViewsphere(kinectPoses[i]);
    av.readKinect();
    av.fuseLastData();
    av.dataExtract();
  }

  std::cout << "Min grasp quality threshold is " << av.minGraspQuality << std::endl;
  av.graspsynthesis();

  std::cout << "No. of grasp pairs found : " << av.graspsPossible.size() << std::endl;
  if (av.graspsPossible.size() > 5){
    std::cout << "Top 5 grasp pairs are : " << std::endl;
    for (int i = 0; i < 5; i++){
      std::cout << i + 1 << " " <<
                   av.graspsPossible[i].p1 << " " <<
                   av.graspsPossible[i].p2 << " " <<
                   av.graspsPossible[i].quality << " " <<
                   av.graspsPossible[i].gripperWidth << std::endl;
    }
  }

  if(flag==1){
    // Setting up the point cloud visualizer
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer")); std::vector<int> vp;
    setupViewer(viewer, 1, vp);
    viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

    // Adding the point clouds
    addRGB(viewer,av.cPtrPtCldEnv,"Environment",vp[0]);
    if (av.graspsPossible.size() > 3){
      for (int i = 0; i < 3; i++){
        viewer->addSphere<pcl::PointXYZRGB>(av.graspsPossible[i].p1,0.0050,0.0,0.0,(i+1.0)/3.0,"GP_"+std::to_string(i)+"_A",vp[0]);
        viewer->addSphere<pcl::PointXYZRGB>(av.graspsPossible[i].p2,0.0050,0.0,0.0,(i+1.0)/3.0,"GP_"+std::to_string(i)+"_B",vp[0]);
      }
    }
    std::cout << "Showing the object and top 3 grasp pairs. Close viewer to continue" << std::endl;
    while (!viewer->wasStopped()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
  av.deleteObject(objID);
  std::cout << "*** End ***" << std::endl;
}

// A test function to check the collision check algorithm with dummy data
void testCollisionDummy(environment &av, bool result, int flag){
  std::cout << "*** In dummy collision testing function ***" << std::endl;

  graspPoint graspTemp; // Creating a dummy grasp
  av.graspsPossible.push_back(graspTemp);

  av.loadGripper();

  // Creating a dummy unexplored point cloud
  av.ptrPtCldUnexp->clear();
  std::vector<double> min{0,0,0}, max{0,0,0};
  min[0] = -0.03; max[0] = 0.03;
  min[1] = -0.15; max[1] = 0.15;
  if(result == true) {min[2] = -0.15; max[2] = -0.02;}
  else {min[2] = -0.15; max[2] = 0.15;}

  pcl::PointXYZRGB ptTemp; ptTemp.b = 200;
  for(float x = min[0]; x < max[0]; x+=0.01){
    for(float y = min[1]; y < max[1]; y+=0.01){
      for(float z = min[2]; z < max[2]; z+=0.01){
        ptTemp.x = x; ptTemp.y = y; ptTemp.z = z;
        av.ptrPtCldUnexp->points.push_back(ptTemp);
      }
    }
  }
  av.ptrPtCldUnexp->width = av.ptrPtCldUnexp->points.size();
  av.ptrPtCldUnexp->height = 1;

  av.collisionCheck();
  if (av.selectedGrasp == -1) {
    std::cout << "No grasp orientation for the grasp points found. Showing the last tested grasp." << std::endl;
  }

  // Setting color to red
  for (int i = 0; i < av.ptrPtCldCollided->size(); i++) {
    av.ptrPtCldCollided->points[i].b = 0;
    av.ptrPtCldCollided->points[i].r = 200;
  }

  if (flag == 1) {
    // Setting up the point cloud visualizer
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer")); std::vector<int> vp;
    setupViewer(viewer, 1, vp);
    viewer->removeCoordinateSystem();
    viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

    addRGB(viewer,av.ptrPtCldUnexp,"Unexplored",vp[0]);
    addRGB(viewer,av.ptrPtCldCollided,"Collision",vp[0]);
    av.updateGripper(0,0);    // Only for visulization purpose
    addRGB(viewer,av.ptrPtCldGripper,"Gripper",vp[0]);
    std::cout << "Showing the Gripper(Black), Unexplored(Blue), Collision(Red) points. Close viewer to continue" << std::endl;

    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
  std::cout << "*** End ***" << std::endl;
}

// 11: A test function to check the collision check algorithm with object and grasp points
void testComplete(environment &av, int objID, int nVp, int graspMode, int flag, int write){
  std::cout << "*** In overall testing function ***" << std::endl;
  int ctrGrasp;

  std::ofstream outfile;
  if(write == 1){
    outfile.open("resultsAV.txt", std::ios_base::app); // append instead of overwrite
    outfile << std::endl;
  }

  std::chrono::high_resolution_clock::time_point start[4], end[4];
  double elapsed[4];

  av.spawnObject(objID,0,0);
  av.loadGripper();

  // 4 kinect poses
  std::vector<std::vector<double>> kinectPoses = {{1,-M_PI,M_PI/4},
                                                  {1,-M_PI/2,M_PI/4},
                                                  {1,0,M_PI/4},
                                                  {1,M_PI/2,M_PI/4}};

  start[0] = std::chrono::high_resolution_clock::now();

  start[1] = std::chrono::high_resolution_clock::now();
  // Read nViepoints and fuse them and update unexplord point cloud
  for (int i = 0; i < nVp; i++){
    av.moveKinectViewsphere(kinectPoses[i]);
    av.readKinect();
    av.fuseLastData();
    av.dataExtract();
    if (i==0){
      av.genUnexploredPtCld();
    }
    av.updateUnexploredPtCld();
  }
  end[1] = std::chrono::high_resolution_clock::now();

  std::cout << "Number of points in object point cloud : " << av.ptrPtCldObject->points.size() << std::endl;
  std::cout << "Number of points in unexplored cloud : " << av.ptrPtCldUnexp->points.size() << std::endl;

  start[2] = std::chrono::high_resolution_clock::now();

  // Grasp synthesis and collision check
  if(graspMode == 1){
    av.graspsynthesis();
    ctrGrasp = av.graspsPossible.size();
    std::cout << "Number of grasps found : " << ctrGrasp << std::endl;
  }else if(graspMode == 2){
    ctrGrasp = av.graspAndCollisionCheck();
    std::cout << "Number of grasps searched : " << ctrGrasp << std::endl;
  }
  end[2] = std::chrono::high_resolution_clock::now();

  if(graspMode == 1){
    start[3] = std::chrono::high_resolution_clock::now();
    av.collisionCheck();
    end[3] = std::chrono::high_resolution_clock::now();
  }

  if(av.selectedGrasp != -1){
    std::cout << "Selected Grasp ID : " << av.selectedGrasp << std::endl;
    std::cout << "Selected Grasp Quality : " << av.graspsPossible[av.selectedGrasp].quality << std::endl;
  }else{
    std::cout << "No grasp found" << std::endl;
  }

  end[0] = std::chrono::high_resolution_clock::now();

  elapsed[0] = (std::chrono::duration_cast<std::chrono::milliseconds>(end[0] - start[0])).count();
  elapsed[1] = (std::chrono::duration_cast<std::chrono::milliseconds>(end[1] - start[1])).count();
  elapsed[2] = (std::chrono::duration_cast<std::chrono::milliseconds>(end[2] - start[2])).count();
  elapsed[3] = (std::chrono::duration_cast<std::chrono::milliseconds>(end[3] - start[3])).count();

  std::cout << std::endl << "Printing out the timings for each section (sec) :" << std::endl;
  std::cout << "Overall Timing = " << elapsed[0]/1000 << std::endl;
  std::cout << "(Move Kinect + Read Kinect +" << std::endl <<
               "Fuse Data + Data Extract +" << std::endl <<
               "Update Enexp PtCld) x 4 = " << elapsed[1]/1000 << std::endl;
  if(graspMode == 1){
    std::cout << "Grasp Synthesis = " << elapsed[2]/1000 << std::endl;
    std::cout << "Collision Check = " << elapsed[3]/1000 << std::endl << std::endl;
  }else if(graspMode == 2){
    std::cout << "Grasp Synthesis + Collision Check = " << elapsed[2]/1000 << std::endl;
  }

  if(write == 1){
    outfile << objID+1 << ","
            << nVp << ","
            << av.voxelGridSizeUnexp << ","
            << av.ptrPtCldObject->points.size() << ","
            << av.ptrPtCldUnexp->points.size() << ","
            << ctrGrasp << ","
            << av.selectedGrasp << ","
            << av.graspsPossible[av.selectedGrasp].quality << ","
            << elapsed[0]/1000 << ","
            << elapsed[1]/1000 << ","
            << elapsed[2]/1000;
    if(graspMode == 1){
      outfile << "," << elapsed[3]/1000;
    }
  }

  if (flag == 1) {
    // Setting up the point cloud visualizer
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer")); std::vector<int> vp;
    setupViewer(viewer, 2, vp);
    viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

    addRGB(viewer,av.ptrPtCldEnv,"Environment",vp[0]);
    addRGB(viewer,av.ptrPtCldObject,"Object",vp[1]);

    pcl::PointXYZRGB centroidObj; Eigen::Vector4f temp1;
    pcl::compute3DCentroid(*av.ptrPtCldObject, temp1);
    centroidObj.x = temp1[0]; centroidObj.y = temp1[1]; centroidObj.z = temp1[2];
    viewer->addSphere<pcl::PointXYZRGB>(centroidObj,0.0050,0.0,1.0,0.0,"Centroid0",vp[0]);
    viewer->addSphere<pcl::PointXYZRGB>(centroidObj,0.0050,0.0,1.0,0.0,"Centroid1",vp[1]);
    // rbgNormalVis(viewer,av.ptrPtCldObject,av.ptrObjNormal,"Object",vp[1]);

    for (int i = 0; i < av.ptrPtCldUnexp->size(); i++) {
      av.ptrPtCldUnexp->points[i].r = 0;
      av.ptrPtCldUnexp->points[i].b = 200;
      av.ptrPtCldUnexp->points[i].g = 0;
    }
    addRGB(viewer,av.ptrPtCldUnexp,"Unexplored",vp[1]);

    if(av.selectedGrasp == -1){
      std::cout << "No grasp orientation for the grasp points found." << std::endl;
      std::cout << "Showing the object (red), collision check points (blue). Close viewer to continue" << std::endl;
    }else{
      av.updateGripper(av.selectedGrasp,0);    // Only for visulization purpose
      addRGB(viewer,av.ptrPtCldGripper,"Gripper",vp[0]);
      addRGB(viewer,av.ptrPtCldGripper,"Gripper1",vp[1]);
      std::cout << "Showing the object (red), collision check points (blue), selected gripper position (black). Close viewer to continue" << std::endl;
    }

    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }

  av.deleteObject(objID);
  if(write == 1) outfile.close();
  std::cout << "*** End ***" << std::endl;
}

// 12: A test function to check rollback feature
void testSaveRollback(environment &av, int objID, int flag){
  std::cout << "*** In save and rollback testing function ***" << std::endl;
  // av.reset();
  av.spawnObject(objID,0,0);
  av.loadGripper();

  double step = 20*M_PI/180;
  std::vector<std::vector<double>> directions = {{ 00,-step},{ step,-step},
                                                 { step, 00},{ step, step},
                                                 { 00, step},{-step, step},
                                                 {-step, 00},{-step,-step}};

  std::vector<double> kinectPose = {1.0,-M_PI,M_PI/4};
  av.moveKinectViewsphere(kinectPose);
  av.readKinect();
  av.fuseLastData();
  av.dataExtract();
  av.genUnexploredPtCld();
  av.updateUnexploredPtCld();
  av.graspsynthesis();
  av.collisionCheck();
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

    if(av.selectedGrasp != -1){
      av.updateGripper(av.selectedGrasp,0);    // Only for visulization purpose
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

    kinectPose = av.lastKinectPoseViewsphere;
    kinectPose[1] += directions[i][0];
    kinectPose[2] += directions[i][1];
    av.moveKinectViewsphere(kinectPose);

    av.readKinect();
    av.fuseLastData();
    av.dataExtract();
    av.updateUnexploredPtCld();
    av.graspsynthesis();
    av.collisionCheck();

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

      if(av.selectedGrasp != -1){
        av.updateGripper(av.selectedGrasp,0);    // Only for visulization purpose
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

// 13: A function to test saving pointcloud
void testSavePCD(environment &av, int objID){
  std::cout << "*** In point cloud save testing function ***" << std::endl;
  std::string dir = "./DataRecAV/";
  std::string time;
  ptCldColor::Ptr ptrPtCldTemp{new ptCldColor};

  av.spawnObject(objID,0,0);
  av.loadGripper();

  // 4 kinect poses
  std::vector<std::vector<double>> kinectPoses = {{1.0,-M_PI,M_PI/4},
                                                  {1.0,-M_PI/2,M_PI/4},
                                                  {1.0,0,M_PI/4},
                                                  {1.0,M_PI/2,M_PI/4}};

  for(int i = 0; i < 4; i++){
    singlePass(av, kinectPoses[i], i==0, true);
    time = getCurTime();
    savePointCloud(av.ptrPtCldObject,dir,time,1);
    savePointCloud(av.ptrPtCldUnexp,dir,time,2);
    if(av.selectedGrasp!=-1){
      av.updateGripper(av.selectedGrasp,0);
      *ptrPtCldTemp = *av.ptrPtCldEnv + *av.ptrPtCldGripper;
      savePointCloud(ptrPtCldTemp,dir,time,3);
      ptrPtCldTemp->clear();
    }else{
      savePointCloud(av.ptrPtCldEnv,dir,time,3);
    }
  }

  std::cout << "*** END ***" << std::endl;
}

// 14: A function to test reading csv file
void testReadCSV(std::string filename, int colID){
  std::cout << "*** In CSV read testing function ***" << std::endl;

  std::vector<std::vector<std::string>> data;
  data = readCSV(filename);

  std::cout << "Printing " << colID << " column of the csv." << std::endl;
  for(int i = 0; i < data.size(); i++){
    if (colID <= data[i].size()){
      std::cout << data[i][colID-1] << std::endl;
    }else{
      std::cout << "****" << std::endl;
    }
  }
  std::cout << "*** END ***" << std::endl;
}

// 15: A function to test reading PCD based on CSV file data
void testReadPCD(std::string filename){
  std::cout << "*** In PCD read testing function ***" << std::endl;
  std::string dir = "./DataRecAV/";
  int colID = 12;
  ptCldColor::Ptr ptrPtCldTemp{new ptCldColor};
  std::vector<std::vector<std::string>> data;
  data = readCSV(filename);

  // Setting up the point cloud visualizer
  ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer")); std::vector<int> vp;
  setupViewer(viewer, 3, vp);
  viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

  std::cout << "Viewing first 4 data recerded." << std::endl;
  for(int i = 0; i < 4; i++){
    if (colID <= data[i].size()){
      readPointCloud(ptrPtCldTemp,dir,data[i][colID-1],1);
      addRGB(viewer,ptrPtCldTemp,"Obj",vp[0]);
      readPointCloud(ptrPtCldTemp,dir,data[i][colID-1],2);
      addRGB(viewer,ptrPtCldTemp,"Unexp",vp[1]);
      readPointCloud(ptrPtCldTemp,dir,data[i][colID-1],3);
      addRGB(viewer,ptrPtCldTemp,"Env",vp[2]);

      std::cout << "Close viewer to view next." << std::endl;
      while (!viewer->wasStopped()){
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds(100000));
      }
      viewer->resetStoppedFlag();
      viewer->removeAllPointClouds();
    }
  }
  std::cout << "*** END ***" << std::endl;
}

// 16: Testing SUrface Patch and Curvature
void testSurfacePatchAndCurvature(environment &av, int objID, int flag){
  std::cout << "*** In surface patch and curvature testing function ***" << std::endl;
  av.spawnObject(objID,0,0.0/180.0*M_PI);

  // 4 kinect position to capture and fuse
  std::vector<std::vector<double>> kinectPoses = {{1.0,-M_PI,M_PI/4},
                                                  {1.0,-M_PI/2,M_PI/4},
                                                  {1.0,0,M_PI/4},
                                                  {1.0,M_PI/2,M_PI/4}};

  for (int i = 0; i < 4; i++){
    av.moveKinectViewsphere(kinectPoses[i]);
    av.readKinect();
    av.fuseLastData();
    av.dataExtract();
    if (i==0){
      av.genUnexploredPtCld();
    }
    av.updateUnexploredPtCld();
  }

  for(int i = 0; i < av.ptrPtCldObject->points.size(); i++){
    if(av.useForGrasp[i]){
      av.ptrPtCldObject->points[i].r = 0;
      av.ptrPtCldObject->points[i].g = 255;
      av.ptrPtCldObject->points[i].b = 0;
    }
  }

  // float avgCurvature;
  // for(int i = 0; i < av.cPtrPtCldObject->points.size(); i++) avgCurvature += av.ptrObjNormal->points[i].curvature;
  // avgCurvature /= av.cPtrPtCldObject->points.size();
  // for(int i = 0; i < av.ptrPtCldObject->points.size(); i++){
  //   if(av.ptrObjNormal->points[i].curvature <= 2*avgCurvature && isContactPatchOk(av.ptrPtCldObject,av.ptrObjNormal,i,av.voxelGridSize)){
  //     av.ptrPtCldObject->points[i].r = 0;
  //     av.ptrPtCldObject->points[i].g = 255;
  //     av.ptrPtCldObject->points[i].b = 0;
  //   }
  // }

  av.deleteObject(objID);

  if (flag == 1) {
    // Setting up the point cloud visualizer
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer")); std::vector<int> vp;
    setupViewer(viewer, 1, vp);
    viewer->setCameraPosition(-2,0,7,1.5,0,1,0,0,1);

    addRGB(viewer,av.ptrPtCldUnexp,"Unexplored",vp[0]);
    addRGB(viewer,av.ptrPtCldObject,"Object",vp[0]);
    std::cout << "Showing the graspable points in green. Close viewer to continue" << std::endl;

    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }

  std::cout << "*** End ***" << std::endl;
}

int main (int argc, char** argv){

  // Initialize ROS
  ros::init(argc, argv, "Environment_Testing");
  ros::NodeHandle nh;

  environment activeVision(&nh);
  // Delay to ensure all publishers and subscribers are connected
  boost::this_thread::sleep(boost::posix_time::milliseconds(500));

  int choice, objID, graspMode;
  std::cout << "Available choices for test functions : " << std::endl;
  std::cout << "1  : Spawn and delete objects and its configurations on the table." << std::endl;
  std::cout << "2  : Load and view the gripper model." << std::endl;
  std::cout << "3  : Move the kinect to a custom position." << std::endl;
  std::cout << "4  : Continuously move the kinect in a viewsphere with centre on the table." << std::endl;
  std::cout << "5  : Read and view the data from kinect." << std::endl;
  std::cout << "6  : Read and fuse the data from 4 different viewpoints." << std::endl;
  std::cout << "7  : Extract the table and object from point cloud." << std::endl;
  std::cout << "8  : Generate the initial unexplored pointcloud based on the object." << std::endl;
  std::cout << "9  : Update the unexplored pointcloud based on 4 different viewpoints." << std::endl;
  std::cout << "10 : Grasp synthesis after fusing 4 viewpoints." << std::endl;
  std::cout << "11 : Selecting a grasp after grasp synthesis and collision check for a object." << std::endl;
  std::cout << "12 : Store and rollback configurations." << std::endl;
  std::cout << "13 : Save point clouds." << std::endl;
  std::cout << "14 : Read CSV and print a column." << std::endl;
  std::cout << "15 : Read PCD based on CSV data." << std::endl;
  std::cout << "16 : Surface patch testing." << std::endl;
  std::cout << "Enter your choice : "; cin >> choice;

  if (choice >= 5 && choice <= 13 || choice == 16) {
    std::cout << "Objects available :" << std::endl;
    for(auto data: activeVision.objectDict){
        data.second.printObjectInfo();
    }
    std::cout << "Enter your choice : "; std::cin>>objID;
    if(choice == 11){
      std::cout << "Grasp Modes available :" << std::endl;
      std::cout << "1: Find all grasps -> Sort -> Collision check" << std::endl;
      std::cout << "2: Find 1 grasp -> Collision Check -> Repeat" << std::endl;
      std::cout << "Enter your grasp Mode : "; std::cin>>graspMode;
    }
    // std::cout << "Enter your voxel grid size (0.005 to 0.01) : "; std::cin >> activeVision.voxelGridSize;
    // activeVision.voxelGridSize = std::max(activeVision.voxelGridSize,0.005);
    // activeVision.voxelGridSize = std::min(activeVision.voxelGridSize,0.01);
  }
  std::string filename;
  int colID;
  if(choice == 14 || choice == 15){
    std::cin.ignore();
    std::cout << "Enter csv filename with path : "; std::getline(std::cin,filename);
  }
  if(choice == 14){
    std::cout << "Enter your column ID (>0) : "; std::cin>>colID;
  }

  switch(choice){
    case 1:
      testSpawnDeleteObj(activeVision);               break;
    case 2:
      testGripper(activeVision,1,0.05);               break;
    case 3:
      testKinectMovement(activeVision);               break;
    case 4:
      testMoveKinectInViewsphere(activeVision);       break;
    case 5:
      testKinectRead(activeVision,objID,1);           break;
    case 6:
      testPtCldFuse(activeVision,objID,1);            break;
    case 7:
      testDataExtract(activeVision,objID,1);          break;
    case 8:
      testGenUnexpPtCld(activeVision,objID,1);        break;
    case 9:
      testUpdateUnexpPtCld(activeVision,objID,1);     break;
    case 10:
      testGraspsynthesis(activeVision,objID,1);       break;
    case 11:
      // for (int i = 1; i <= 4; i++) {
      //   for (int j = 0; j <= 5; j++) {
      //     testComplete(activeVision,j,i,graspMode,0,1);
      //     activeVision.reset();
      //   }
      // }
      testComplete(activeVision,objID,4,graspMode,1,0);
      break;
    case 12:
      testSaveRollback(activeVision,objID,1);         break;
    case 13:
      testSavePCD(activeVision,objID);                break;
    case 14:
      testReadCSV(filename,colID);                    break;
    case 15:
      testReadPCD(filename);                          break;
    case 16:
      testSurfacePatchAndCurvature(activeVision,objID,1);         break;
    default:
      std::cout << "Invalid choice." << std::endl;
  }
  // testCollisionDummy(activeVision,false,1);
}

/*
Notes:
-> POint cloud XYZRGB data type : std::vector< pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >
-> 640 elements in rach row of the matrix.
-> Transformation of KinectOpticalFrame wrt KinectGazeboFrame (RPY) - (-90 0 -90)
-> Working combo : Voxel Obj 0.01, Unexp voxel 0.02
*/
