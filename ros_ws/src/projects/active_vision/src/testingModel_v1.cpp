#include <active_vision/testingModel_v1.h>
#include "active_vision/dataHandling.h"

// Struct graspPoint constructor
graspPoint::graspPoint(){
  quality = 0;
  gripperWidth = 0.05;
  pose = {0,0,0,0,0,0};
  addnlPitch = 0;
}

// Function to compare grasp point for sorting
bool compareGrasp(graspPoint A, graspPoint B){
  return(A.quality > B.quality);
}

// Fuction to view a rgb point cloud
void rbgVis(ptCldVis::Ptr viewer, ptCldColor::ConstPtr cloud, std::string name,int vp){
  viewer->setBackgroundColor(0.5,0.5,0.5,vp);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud,rgb,name,vp);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,name,vp);
}

// Fuction to view a rgb point cloud with normals
void rbgNormalVis(ptCldVis::Ptr viewer, ptCldColor::ConstPtr cloud, ptCldNormal::ConstPtr normal, std::string name,int vp){
  viewer->setBackgroundColor(0.5,0.5,0.5,vp);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud,rgb,name,vp);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,name,vp);
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud,normal,1,0.01,name+"_normal",vp);
}

// Funstion to transpose a homogenous matrix
Eigen::Affine3f homoMatTranspose(const Eigen::Affine3f& tf){
  Eigen::Affine3f tfTranspose;
  tfTranspose.setIdentity();
  tfTranspose.matrix().block<3,3>(0,0) = tf.rotation().transpose();
  tfTranspose.matrix().block<3,1>(0,3) = -1*tf.rotation().transpose()*tf.translation();
  return(tfTranspose);
}

// Get Rotation Part of a Affine3f
Eigen::Vector3f getEuler(const Eigen::Affine3f& tf){
  return Eigen::Vector3f(atan2f(tf(2,1), tf(2,2)),
                         asinf(-tf(2,0)),
                         atan2f(tf(1,0), tf(0,0)));
}

// Get Translational Part of a Affine3f
Eigen::Vector3f getTranslation(const Eigen::Affine3f& tf){
  return Eigen::Vector3f(tf(0,3), tf(1,3), tf(2,3));
}

// ******************** ENVIRONMENT CLASS FUNCTIONS START ********************
// Environment class constructor
environment::environment(ros::NodeHandle *nh){

  pubObjPose = nh->advertise<gazebo_msgs::ModelState> ("/gazebo/set_model_state", 1);
  subKinectPtCld = nh->subscribe ("/camera/depth/points", 1, &environment::cbPtCld, this);
  // NOT USED (JUST FOR REFERENCE)
  /*subKinectRGB = nh->subscribe ("/camera/color/image_raw", 1, &environment::cbImgRgb, this);
  subKinectDepth = nh->subscribe ("/camera/depth/image_raw", 1, &environment::cbImgDepth, this);*/
  gazeboSpawnModel = nh->serviceClient< gazebo_msgs::SpawnModel> ("/gazebo/spawn_sdf_model");
  gazeboDeleteModel = nh->serviceClient< gazebo_msgs::DeleteModel> ("/gazebo/delete_model");

  readFlag[3] = {};           // Flag used to read data from kinect only when needed
  fingerZOffset = 0.0584;     // Z axis offset between gripper hand and finger

  // Transform : Kinect Optical Frame to Kinect Gazebo frame
  tfKinOptGaz = pcl::getTransformation(0,0,0,-M_PI/2,0,-M_PI/2);

  // Camera projection matrix
  projectionMat.resize(3,4);
  projectionMat << 554.254691191187, 0.0, 320.5, -0.0,
                   0.0, 554.254691191187, 240.5, 0.0,
                   0.0, 0.0, 1.0, 0.0;

  path = ros::package::getPath("active_vision");  // Path to the active_vision package folder

  // Dictionary of objects to be spawned
  objectDict = {{"drillAV","Cordless Drill"},
                {"squarePrismAV","Square Prism"},
                {"rectPrismAV","Rectangular Prism"},
                {"bowlAV","Bowl"},
                {"bowl2AV","Bowl2"},
                {"cupAV","Cup"}};

  // Stores stable poses for the objects (Z(m), Roll(Rad), Pitch(Rad))
  objectPosesDict = {{{0.012,0.000,0.000},  {0.084,1.459,-0.751},
                      {0.082,1.398,-0.014}, {0.040,0.280,-1.040},
                      {0.249,2.860,-0.400}, {0.068,-1.287,-0.477},
                      {0.048,-1.570,1.480}},
                     {{0.015,0.000,0.000}},
                     {{0.015,0.000,0.000},{0.015,0.000,1.570}},
                     {{0.015,0.000,0.000}},
                     {{0.015,0.000,0.000}},
                     {{0.022,0.000,0.000}}};

  objectPosesYawLimits = {{0,359},{0,89},{0,89},{0,1},{0,1},{0,1},{0,1}};

  voxelGridSize = 0.01;          // Voxel Grid size for environment
  voxelGridSizeUnexp = 0.015;    // Voxel Grid size for unexplored point cloud
  tableCentre = {1.5,0,1};       // Co-ordinates of table centre
  minUnexp = {0,0,0};
  maxUnexp = {0,0,0};
  scale = 3;                     // Scale value for unexplored point cloud generation
  maxGripperWidth = 0.075;       // Gripper max width (Actual is 8 cm)
  minGraspQuality = 150;         // Min grasp quality threshold
  selectedGrasp = -1;            // Index of the selected grasp
}

// Function to reset the environment
void environment::reset(){
  ptrPtCldEnv->clear();
  ptrPtCldUnexp->clear();
  configurations.clear();
}

// Store the configuration
int environment::saveConfiguration(std::string name){
  stateConfig configTemp;
  configTemp.env = *ptrPtCldEnv;
  configTemp.unexp = *ptrPtCldUnexp;
  configTemp.kinectPose = lastKinectPoseViewsphere;
  configTemp.description = name;
  configurations.push_back(configTemp);
  //std::cout << "State saved : " << name << std::endl;
  return configurations.size()-1;
}

// Rollback to a configuration
void environment::rollbackConfiguration(int index){
  *ptrPtCldEnv = configurations[index].env;
  *ptrPtCldUnexp = configurations[index].unexp;
  lastKinectPoseViewsphere = configurations[index].kinectPose;
  //std::cout << "Rolled back to state : " << configurations[index].description << std::endl;
}

// 1A: Callback function to point cloud subscriber
void environment::cbPtCld(const ptCldColor::ConstPtr& msg){
  if (readFlag[0]==1) {
    *ptrPtCldLast = *msg;
    readFlag[0] = 0;
  }
}

// NOT USED (JUST FOR REFERENCE)
/*// 1B: Callback function to RGB image subscriber
void cbImgRgb (const sensor_msgs::ImageConstPtr& msg){
  if (readFlag[1]==1) {
    ptrRgbLast = cv_bridge::toCvShare(msg);
    readFlag[1] = 0;
  }
}

// 1C: Callback function to RGB image subscriber
void cbImgDepth (const sensor_msgs::ImageConstPtr& msg){
  if (readFlag[2]==1) {
    ptrDepthLast = cv_bridge::toCvShare(msg);
    readFlag[2] = 0;
  }
}*/

// 2A: Spawning objects in gazebo on the table centre for a given pose option and yaw
void environment::spawnObject(int objectID, int choice, float yaw){
  gazebo_msgs::SpawnModel spawnObj;
  geometry_msgs::Pose pose;

  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = objectPosesDict[objectID][0][0];
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  spawnObj.request.model_name = objectDict[objectID][1];

  std::ifstream ifs(path+"/models/"+objectDict[objectID][0]+"/model.sdf");
  std::string sdfFile( (std::istreambuf_iterator<char>(ifs)),
                       (std::istreambuf_iterator<char>()));
  spawnObj.request.model_xml = sdfFile;

  spawnObj.request.reference_frame = "world";
  spawnObj.request.initial_pose = pose;

  gazeboSpawnModel.call(spawnObj);
  boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
  moveObject(objectID,choice,yaw);
}

// 2B: Function to move the object. Same args as spawnObject
void environment::moveObject(int objectID, int choice, float yaw){

  if(choice >= objectPosesDict[objectID].size()){
    choice = 0;
    printf("WARNING moveObject: Pose choice invalid. Setting choice to 0.\n");
  }

  //Create Matrix3x3 from Euler Angles
  tf::Matrix3x3 m_rot;
  m_rot.setEulerYPR(yaw, objectPosesDict[objectID][choice][2], objectPosesDict[objectID][choice][1]);

  // Convert into quaternion
  tf::Quaternion quat;
  m_rot.getRotation(quat);

  // Converting it to the required gazebo format
  gazebo_msgs::ModelState ModelState;
  ModelState.model_name = objectDict[objectID][1];
  ModelState.reference_frame = "world";
  ModelState.pose.position.x = tableCentre[0];
  ModelState.pose.position.y = tableCentre[1];
  ModelState.pose.position.z = tableCentre[2]+objectPosesDict[objectID][choice][0];
  ModelState.pose.orientation.x = quat.x();
  ModelState.pose.orientation.y = quat.y();
  ModelState.pose.orientation.z = quat.z();
  ModelState.pose.orientation.w = quat.w();

  // Publishing it to gazebo
  pubObjPose.publish(ModelState);
  ros::spinOnce();
  boost::this_thread::sleep(boost::posix_time::milliseconds(500));
}

// 3: Deleting objects in gazebo
void environment::deleteObject(int objectID){
  gazebo_msgs::DeleteModel deleteObj;
  deleteObj.request.model_name = objectDict[objectID][1];

  gazeboDeleteModel.call(deleteObj);
}

// 4: Load Gripper Hand and Finger file
void environment::loadGripper(){
  std::string pathToHand = path+"/models/gripperAV/hand1.ply";
  std::string pathToFinger = path+"/models/gripperAV/finger1.ply";
  // Gripper Hand
  if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(pathToHand, *ptrPtCldGrpHnd) == -1){
    PCL_ERROR ("Couldn't read file hand.ply \n");
  }
  // Gripper Left Finger
  if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(pathToFinger, *ptrPtCldGrpLfgr) == -1){
    PCL_ERROR ("Couldn't read file finger.ply \n");
  }
  // Gripper Right Finger
  pcl::transformPointCloud(*ptrPtCldGrpLfgr, *ptrPtCldGrpRfgr, pcl::getTransformation(0,0,0,0,0,M_PI));

  pcl::getMinMax3D(*ptrPtCldGrpHnd,minPtGrp[0],maxPtGrp[0]);
  pcl::getMinMax3D(*ptrPtCldGrpLfgr,minPtGrp[1],maxPtGrp[1]);
  pcl::getMinMax3D(*ptrPtCldGrpRfgr,minPtGrp[2],maxPtGrp[2]);
  std::cout << "Ignore the PLY reader error on 'face' and 'rgb'." << std::endl;
}

// 5: Update gripper
// 0 -> Visualization
// 1 -> Axis Collision Check
// 2 -> Gripper Collision Check
void environment::updateGripper(int index ,int choice){

  tfGripper = pcl::getTransformation(graspsPossible[index].pose[0],graspsPossible[index].pose[1],
                                     graspsPossible[index].pose[2],graspsPossible[index].pose[3],
                                     graspsPossible[index].pose[4],graspsPossible[index].pose[5])*
              pcl::getTransformation(0,0,0,0,graspsPossible[index].addnlPitch,0)*
              pcl::getTransformation(0,0,-0.0447-fingerZOffset,0,0,0);

  if (choice == 0) {
    // Adding the gripper hand
    *ptrPtCldGripper=*ptrPtCldGrpHnd;

    // Translating the left finger and adding
    pcl::transformPointCloud(*ptrPtCldGrpLfgr, *ptrPtCldTemp,
                            pcl::getTransformation(0,graspsPossible[index].gripperWidth/2,fingerZOffset,0,0,0));
    *ptrPtCldGripper += *ptrPtCldTemp;

    // Translating the right finger and adding
    pcl::transformPointCloud(*ptrPtCldGrpRfgr, *ptrPtCldTemp,
                            pcl::getTransformation(0,-graspsPossible[index].gripperWidth/2,fingerZOffset,0,0,0));
    *ptrPtCldGripper += *ptrPtCldTemp;

    pcl::transformPointCloud(*ptrPtCldGripper, *ptrPtCldGripper, tfGripper);
    ptrPtCldTemp->clear();

  } else if (choice == 1) {
    // Left Finger Basic Check
    minPtCol[3].x = -0.0125; maxPtCol[3].x = 0.0125;
    minPtCol[3].y =  0.0;    maxPtCol[3].y = 0.0250;
    minPtCol[3].z =  0.0322; maxPtCol[3].z = 0.0572;
    minPtCol[3] = pcl::transformPoint(minPtCol[3],pcl::getTransformation(0,graspsPossible[index].gripperWidth/2,fingerZOffset,0,0,0));
    maxPtCol[3] = pcl::transformPoint(maxPtCol[3],pcl::getTransformation(0,graspsPossible[index].gripperWidth/2,fingerZOffset,0,0,0));

    // Right Finger Basic Check
    minPtCol[4].x = -0.0125; maxPtCol[4].x = 0.0125;
    minPtCol[4].y = -0.0250; maxPtCol[4].y = 0.0;
    minPtCol[4].z =  0.0322; maxPtCol[4].z = 0.0572;
    minPtCol[4] = pcl::transformPoint(minPtCol[4],pcl::getTransformation(0,-graspsPossible[index].gripperWidth/2,fingerZOffset,0,0,0));
    maxPtCol[4] = pcl::transformPoint(maxPtCol[4],pcl::getTransformation(0,-graspsPossible[index].gripperWidth/2,fingerZOffset,0,0,0));

  } else if (choice == 2) {
    // Hand
    minPtCol[0] = minPtGrp[0];
    maxPtCol[0] = maxPtGrp[0];

    // Left Finger
    minPtCol[1] = pcl::transformPoint(minPtGrp[1],pcl::getTransformation(0,graspsPossible[index].gripperWidth/2,fingerZOffset,0,0,0));
    maxPtCol[1] = pcl::transformPoint(maxPtGrp[1],pcl::getTransformation(0,graspsPossible[index].gripperWidth/2,fingerZOffset,0,0,0));

    // Right Finger
    minPtCol[2] = pcl::transformPoint(minPtGrp[2],pcl::getTransformation(0,-graspsPossible[index].gripperWidth/2,fingerZOffset,0,0,0));
    maxPtCol[2] = pcl::transformPoint(maxPtGrp[2],pcl::getTransformation(0,-graspsPossible[index].gripperWidth/2,fingerZOffset,0,0,0));
  }
}

// 6A: Function to move the kinect. Args: Array of X,Y,Z,Roll,Pitch,Yaw
void environment::moveKinectCartesian(std::vector<double> pose){
  //Create Matrix3x3 from Euler Angles
  tf::Matrix3x3 rotMat;
  rotMat.setEulerYPR(pose[5], pose[4], pose[3]);

  // Convert into quaternion
  tf::Quaternion quat;
  rotMat.getRotation(quat);

  // Converting it to the required gazebo format
  gazebo_msgs::ModelState ModelState;
  ModelState.model_name = "Kinect";           // This should be the name of kinect in gazebo
  ModelState.reference_frame = "world";
  ModelState.pose.position.x = pose[0];
  ModelState.pose.position.y = pose[1];
  ModelState.pose.position.z = pose[2];
  ModelState.pose.orientation.x = quat.x();
  ModelState.pose.orientation.y = quat.y();
  ModelState.pose.orientation.z = quat.z();
  ModelState.pose.orientation.w = quat.w();

  // Publishing it to gazebo
  pubObjPose.publish(ModelState);
  ros::spinOnce();
  boost::this_thread::sleep(boost::posix_time::milliseconds(250));

  // Storing the kinect pose
  lastKinectPoseCartesian = pose;
}

// 6B: Funtion to move the Kinect in a viewsphere which has the table cente as its centre
// R (Radius)
// Phi (Azhimuthal angle) -> 0 to 2*PI
// Theta (Polar Angle)) -> 0 to PI/2
void environment::moveKinectViewsphere(std::vector<double> pose){
  //Create Matrix3x3 from Euler Angles
  tf::Matrix3x3 rotMat;
  rotMat.setEulerYPR(M_PI+pose[1], M_PI/2-pose[2], 0);

  // Convert into quaternion
  tf::Quaternion quat;
  rotMat.getRotation(quat);

  // Converting it to the required gazebo format
  gazebo_msgs::ModelState ModelState;
  ModelState.model_name = "Kinect";           // This should be the name of kinect in gazebo
  ModelState.reference_frame = "world";
  ModelState.pose.position.x = tableCentre[0]+pose[0]*sin(pose[2])*cos(pose[1]);
  ModelState.pose.position.y = tableCentre[1]+pose[0]*sin(pose[2])*sin(pose[1]);
  ModelState.pose.position.z = tableCentre[2]+pose[0]*cos(pose[2]);
  ModelState.pose.orientation.x = quat.x();
  ModelState.pose.orientation.y = quat.y();
  ModelState.pose.orientation.z = quat.z();
  ModelState.pose.orientation.w = quat.w();

  // Publishing it to gazebo
  pubObjPose.publish(ModelState);
  ros::spinOnce();
  boost::this_thread::sleep(boost::posix_time::milliseconds(250));

  // Storing the kinect pose
  lastKinectPoseViewsphere = pose;
  lastKinectPoseCartesian = {ModelState.pose.position.x,
                             ModelState.pose.position.y,
                             ModelState.pose.position.z,
                             0,M_PI/2-pose[2],M_PI+pose[1]};
}

// 7: Function to read the kinect data.
void environment::readKinect(){
  readFlag[0] = 1; // readFlag[1] = 1; readFlag[2] = 1;
  while (readFlag[0]==1) {
    ros::spinOnce();
    r.sleep();
  }
}

// 8: Function to Fuse last data with existing data
void environment::fuseLastData(){
  ptrPtCldTemp->clear();
  // Transform : Kinect Gazebo Frame to Gazebo World frame
  tfGazWorld = pcl::getTransformation(lastKinectPoseCartesian[0],lastKinectPoseCartesian[1],lastKinectPoseCartesian[2],\
                                      lastKinectPoseCartesian[3],lastKinectPoseCartesian[4],lastKinectPoseCartesian[5]);

  // Apply transformation
  Eigen::Affine3f tf = tfGazWorld * tfKinOptGaz;
  pcl::transformPointCloud(*ptrPtCldLast, *ptrPtCldTemp, tf);

  // Downsample using voxel grid
  voxelGrid.setInputCloud(cPtrPtCldTemp);
  voxelGrid.setLeafSize(voxelGridSize, voxelGridSize, voxelGridSize);
  voxelGrid.filter(*ptrPtCldTemp);

  // Use registration to further align the point pointclouds
  // Skipping this for now as using simulation

  // Fuse the two pointclouds (except for the first time) and downsample again
  if (ptrPtCldEnv->width == 0) {
    *ptrPtCldEnv = *ptrPtCldTemp;
  }else{
    *ptrPtCldEnv += *ptrPtCldTemp;
    voxelGrid.setInputCloud(cPtrPtCldEnv);
    voxelGrid.setLeafSize(voxelGridSize, voxelGridSize, voxelGridSize);
    voxelGrid.filter(*ptrPtCldEnv);
  }

  // Using pass through filter to remove ground plane
  pass.setInputCloud(cPtrPtCldEnv);
  pass.setFilterFieldName("z"); pass.setFilterLimits(0.2,10);
  pass.filter(*ptrPtCldEnv);

  ptrPtCldTemp->clear();
}

// 9: Extracting the major plane (Table) and object
void environment::dataExtract(){
  // Find the major plane and get its coefficients and indices
  seg.setInputCloud(cPtrPtCldEnv);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.01);
  seg.segment(*tableIndices,*tableCoeff);

  if (tableIndices->indices.size () == 0){
    std::cerr << "No table found in the environment" << std::endl;
    return;
  }

  // Seperating the table and storing its point
  extract.setInputCloud(cPtrPtCldEnv);
  extract.setIndices(tableIndices);
  extract.setNegative(false);
  extract.filter(*ptrPtCldTable);

  // Using convex hull to get the table boundary which would be like a rectangle
  cvHull.setInputCloud(cPtrPtCldTable);
  cvHull.setDimension(2);
  cvHull.reconstruct(*ptrPtCldHull);

  // Double checking the hull dimensions
  if (cvHull.getDimension() != 2){
    std::cerr << "Convex hull dimension != 2" << std::endl;
    return;
  }

  // Using polygonal prism and hull the extract object above the table
  prism.setInputCloud(cPtrPtCldEnv);
  prism.setInputPlanarHull(cPtrPtCldHull);
  if (tableCoeff->values[3] < 0) {
    prism.setHeightLimits(-1.5f,-0.005f-voxelGridSize);         // Z height (min, max) in m
  }else{
    prism.setHeightLimits(0.005f+voxelGridSize,1.5f);           // Z height (min, max) in m
  }
  prism.segment(*objectIndices);

  // Using extract to get the point cloud
  extract.setInputCloud(cPtrPtCldEnv);
  extract.setNegative(false);
  extract.setIndices(objectIndices);
  extract.filter(*ptrPtCldObject);

  // Getting the min and max co-ordinates of the object
  pcl::getMinMax3D(*ptrPtCldObject, minPtObj, maxPtObj);
}

// 10: Generating unexplored point cloud
void environment::genUnexploredPtCld(){
  if(ptrPtCldUnexp->width != 0){
    std::cerr << "Unexplored point cloud already created. Not creating new one." << std::endl;
    return;
  }

  // Setting the min and max limits based on the object dimension and scale.
  // Min of 0.25m on each side
  // Note: Z scale is only used on +z axis
  minUnexp[0] = (minPtObj.x-std::max((scale-1)*(maxPtObj.x-minPtObj.x)/2,0.40f));
  minUnexp[1] = (minPtObj.y-std::max((scale-1)*(maxPtObj.y-minPtObj.y)/2,0.40f));
  minUnexp[2] = (minPtObj.z-0.02);
  maxUnexp[0] = (maxPtObj.x+std::max((scale-1)*(maxPtObj.x-minPtObj.x)/2,0.40f));
  maxUnexp[1] = (maxPtObj.y+std::max((scale-1)*(maxPtObj.y-minPtObj.y)/2,0.40f));
  maxUnexp[2] = (maxPtObj.z+std::max((scale-1)*(maxPtObj.z-minPtObj.z)/2,0.25f));

  pcl::PointXYZRGB ptTemp;
  for(float x = minUnexp[0]; x < maxUnexp[0]; x+=voxelGridSizeUnexp){
    for(float y = minUnexp[1]; y < maxUnexp[1]; y+=voxelGridSizeUnexp){
      for(float z = minUnexp[2]; z < maxUnexp[2]; z+=voxelGridSizeUnexp){
        ptTemp.x = x; ptTemp.y = y; ptTemp.z = z;
        ptrPtCldUnexp->points.push_back(ptTemp);
      }
    }
  }
  ptrPtCldUnexp->width = ptrPtCldUnexp->points.size();
  ptrPtCldUnexp->height = 1;
}

// 11: Updating the unexplored point cloud
void environment::updateUnexploredPtCld(){
  // Transforming the point cloud to Kinect frame from world frame
  Eigen::Affine3f tf = tfGazWorld*tfKinOptGaz;
  Eigen::Affine3f tfTranspose = homoMatTranspose(tf);
  pcl::transformPointCloud(*ptrPtCldUnexp, *ptrPtCldTemp, tfTranspose);

  Eigen::Vector4f ptTemp;
  Eigen::Vector3f proj;
  pcl::PointIndices::Ptr occludedIndices(new pcl::PointIndices());
  int projIndex;

  // Looping through all the points and finding occluded ones.
  // Using the camera projection matrix to project 3D point to camera plane
  for (int i = 0; i < ptrPtCldTemp->width; i++){
    ptTemp = ptrPtCldTemp->points[i].getVector4fMap();
    proj = projectionMat*ptTemp;
    proj = proj/proj[2];
    proj[0] = round(proj[0])-1;
    proj[1] = round(proj[1])-1;
    projIndex = proj[1]*(ptrPtCldLast->width)+proj[0];
    // If the z value of unexplored pt is greater than the corresponding
    // projected point in Kinect Raw data then that point is occluded.
    if (ptrPtCldLast->points[projIndex].z <= ptTemp[2]){
      occludedIndices->indices.push_back(i);
    }
  }

  // Only keeping the occluded points
  extract.setInputCloud(cPtrPtCldUnexp);
  extract.setIndices(occludedIndices);
  extract.setNegative(false);
  extract.filter(*ptrPtCldUnexp);

  if(minUnexp[0] > minPtObj.x || minUnexp[1] > minPtObj.y ||
     maxUnexp[0] < maxPtObj.x || maxUnexp[1] < maxPtObj.y || maxUnexp[2] < maxPtObj.z){
    std::cout << "WARNING : Unexplored point cloud initially generated smaller than the object. " <<
                 "Increase unexplored point cloud size for better accuracy during collision check." << std::endl;
  }

  ptrPtCldTemp->clear();
}

// 12: Finding normals and pairs of grasp points from object point cloud
void environment::graspsynthesis(){
  // Generating the normals for the object point cloud
  pcl::search::Search<pcl::PointXYZRGB>::Ptr KdTree{new pcl::search::KdTree<pcl::PointXYZRGB>};
  ne.setInputCloud(cPtrPtCldObject);
  ne.setSearchMethod(KdTree);
  ne.setKSearch(10);
  ne.compute(*ptrObjNormal);

  graspsPossible.clear();   // Clear the vector
  selectedGrasp = -1;

  graspPoint graspTemp;
  Eigen::Vector3f vectA, vectB;
  double A,B;

  for (int i = 0; i < ptrPtCldObject->size()-1; i++){
    for (int j = i+1; j < ptrPtCldObject->size(); j++){
      graspTemp.p1 = ptrPtCldObject->points[i];
      graspTemp.p2 = ptrPtCldObject->points[j];

      // Ignoring point closer to table
      if(graspTemp.p1.z <= tableCentre[2]+0.03 || graspTemp.p2.z <= tableCentre[2]+0.03) continue;

      // Vector connecting the two grasp points and its distance
      vectA = graspTemp.p1.getVector3fMap() - graspTemp.p2.getVector3fMap();
      vectB = graspTemp.p2.getVector3fMap() - graspTemp.p1.getVector3fMap();
      graspTemp.gripperWidth = vectA.norm() + voxelGridSize; // Giving a tolerance based on voxel grid size

      // If grasp width is greater than the limit then skip the rest
      if(graspTemp.gripperWidth > maxGripperWidth) continue;

      // Using normals to find the angle
      A = std::min(pcl::getAngle3D(vectA,ptrObjNormal->points[i].getNormalVector3fMap()),
                   pcl::getAngle3D(vectB,ptrObjNormal->points[i].getNormalVector3fMap()))*180/M_PI;
      B = std::min(pcl::getAngle3D(vectA,ptrObjNormal->points[j].getNormalVector3fMap()),
                   pcl::getAngle3D(vectB,ptrObjNormal->points[j].getNormalVector3fMap()))*180/M_PI;

      graspTemp.quality = 180 - ( A + B );
      // If grasp quality is less than the min requirement then skip the rest
      if(graspTemp.quality < minGraspQuality) continue;

      // Push this into the vector
      graspsPossible.push_back(graspTemp);
    }
  }
  std::sort(graspsPossible.begin(),graspsPossible.end(),compareGrasp);

  // For thin objects grasp pair would not be feasible, so each point is considered as a grasp pair
  // Adding these grasps in the end
  Eigen::Vector3f xyPlaneA(0,0,1);
  Eigen::Vector3f xyPlaneB(0,0,-1);
  for(int i = 0; i < ptrPtCldObject->size(); i++){

    A = std::min(pcl::getAngle3D(xyPlaneA,ptrObjNormal->points[i].getNormalVector3fMap()),
                 pcl::getAngle3D(xyPlaneB,ptrObjNormal->points[i].getNormalVector3fMap()))*180/M_PI;

    // If the point is too close to table and its normal vector is along z axis this skip it
    if (A > 45 && ptrPtCldObject->points[i].z < tableCentre[2]+0.02){
      continue;
    }
    graspTemp.p1 = ptrPtCldObject->points[i];
    // Translating it along the +ve normal vector
    graspTemp.p1.x += (voxelGridSize)/2*ptrObjNormal->points[i].normal_x;
    graspTemp.p1.y += (voxelGridSize)/2*ptrObjNormal->points[i].normal_y;
    graspTemp.p1.z += (voxelGridSize)/2*ptrObjNormal->points[i].normal_z;

    graspTemp.p2 = ptrPtCldObject->points[i];
    // Translating it along the -ve normal vector
    graspTemp.p2.x -= (voxelGridSize)/2*ptrObjNormal->points[i].normal_x;
    graspTemp.p2.y -= (voxelGridSize)/2*ptrObjNormal->points[i].normal_y;
    graspTemp.p2.z -= (voxelGridSize)/2*ptrObjNormal->points[i].normal_z;

    graspTemp.gripperWidth = voxelGridSize;
    graspTemp.quality = 180;
    graspsPossible.push_back(graspTemp);
  }
}

// 13: Given a grasp point pair find the gripper orientation
void environment::findGripperPose(int index){

  Eigen::Vector3f xAxis,yAxis,zAxis;
  Eigen::Vector3f xyPlane(0,0,1);

  yAxis = graspsPossible[index].p1.getVector3fMap() - graspsPossible[index].p2.getVector3fMap(); yAxis.normalize();
  zAxis = yAxis.cross(xyPlane); zAxis.normalize();
  xAxis = yAxis.cross(zAxis);

  tf::Matrix3x3 rotMat;
  double Roll,Pitch,Yaw;
  rotMat.setValue(xAxis[0],yAxis[0],zAxis[0],
                  xAxis[1],yAxis[1],zAxis[1],
                  xAxis[2],yAxis[2],zAxis[2]);
  rotMat.getRPY(Roll,Pitch,Yaw);

  std::vector<float> pose = {0,0,0,0,0,0};
  pose[0] = (graspsPossible[index].p1.x + graspsPossible[index].p2.x)/2;
  pose[1] = (graspsPossible[index].p1.y + graspsPossible[index].p2.y)/2;
  pose[2] = (graspsPossible[index].p1.z + graspsPossible[index].p2.z)/2;
  pose[3] = Roll; pose[4] = Pitch; pose[5] = Yaw;

  graspsPossible[index].pose = pose;
}

// 14: Collision check for gripper and unexplored point cloud
void environment::collisionCheck(){
  ptrPtCldCollided->clear();    // Reset the collision cloud
  *ptrPtCldCollCheck = *ptrPtCldUnexp + *ptrPtCldObject;
  cpBox.setInputCloud(ptrPtCldCollCheck);

  bool stop = false;
  int nOrientations = 8;
  // Loop through all the possible grasps available
  for(int i = 0; (i < graspsPossible.size()) && (stop == false); i++){
    findGripperPose(i);

    // Do axis collision check
    updateGripper(i,1);
    for (int j = 3; j < 5; j++) {
      cpBox.setMin(minPtCol[j].getVector4fMap());
      cpBox.setMax(maxPtCol[j].getVector4fMap());
      cpBox.setRotation(getEuler(tfGripper));
      cpBox.setTranslation(getTranslation(tfGripper));
      cpBox.filter(*ptrPtCldCollided);
      // If collision detected then exit this loop and check next grasp pair
      if(ptrPtCldCollided->size() > 0) break;
    }
    // Move to next grasp is collision found
    if(ptrPtCldCollided->size() > 0) continue;

    // Do gripper collision check for each orientation
    for(int j = 0; (j < nOrientations) && (stop == false); j++){
      graspsPossible[i].addnlPitch = j*(2*M_PI)/nOrientations;
      updateGripper(i,2);
      // Get concave hull of the gripper fingers and hand in sequence and check
      for(int k = 2; k >= 0; k--){
        // ptrPtCldCollided->clear();    // Reset the collision cloud
        cpBox.setMin(minPtCol[k].getVector4fMap());
        cpBox.setMax(maxPtCol[k].getVector4fMap());
        cpBox.setRotation(getEuler(tfGripper));
        cpBox.setTranslation(getTranslation(tfGripper));
        cpBox.filter(*ptrPtCldCollided);

        // If collision detected then exit this loop and check next orientation
        if(ptrPtCldCollided->size() > 0) break;
      }
      // If this doesn't have collision, this grasp is OK. So exit the loop. No more orientation or grasp check required
      if(ptrPtCldCollided->size() == 0){
        selectedGrasp = i;
        stop = true;
      }
    }
  }
}

// 15: Grasp and Collision check combined
int environment::graspAndCollisionCheck(){
  // Generating the normals for the object point cloud
  pcl::search::Search<pcl::PointXYZRGB>::Ptr KdTree{new pcl::search::KdTree<pcl::PointXYZRGB>};
  ne.setInputCloud(cPtrPtCldObject);
  ne.setSearchMethod(KdTree);
  ne.setKSearch(10);
  ne.compute(*ptrObjNormal);

  graspsPossible.clear();   // Clear the vector
  selectedGrasp = -1;

  graspPoint graspTemp;
  Eigen::Vector3f vectA, vectB;
  double A,B;
  int nGrasp = 0;
  // std::cout << "IN1" << std::endl;
  for (int i = 0; i < ptrPtCldObject->size()-1; i++){
    for (int j = i+1; j < ptrPtCldObject->size(); j++){
      graspTemp.p1 = ptrPtCldObject->points[i];
      graspTemp.p2 = ptrPtCldObject->points[j];

      // Ignoring point closer to table
      if(graspTemp.p1.z <= tableCentre[2]+0.03 || graspTemp.p2.z <= tableCentre[2]+0.03) continue;

      // Vector connecting the two grasp points and its distance
      vectA = graspTemp.p1.getVector3fMap() - graspTemp.p2.getVector3fMap();
      vectB = graspTemp.p2.getVector3fMap() - graspTemp.p1.getVector3fMap();
      graspTemp.gripperWidth = vectA.norm() + voxelGridSize; // Giving a tolerance based on voxel grid size

      // If grasp width is greater than the limit then skip the rest
      if(graspTemp.gripperWidth > maxGripperWidth) continue;

      // Using normals to find the angle
      A = std::min(pcl::getAngle3D(vectA,ptrObjNormal->points[i].getNormalVector3fMap()),
                   pcl::getAngle3D(vectB,ptrObjNormal->points[i].getNormalVector3fMap()))*180/M_PI;
      B = std::min(pcl::getAngle3D(vectA,ptrObjNormal->points[j].getNormalVector3fMap()),
                   pcl::getAngle3D(vectB,ptrObjNormal->points[j].getNormalVector3fMap()))*180/M_PI;

      graspTemp.quality = 180 - ( A + B );

      // If grasp quality is less than the min requirement then skip the rest
      if (graspTemp.quality < minGraspQuality) continue;

      nGrasp++;
      // Push this into the vector
      if (graspsPossible.size() == 0){
        graspsPossible.push_back(graspTemp);
      }else{
        graspsPossible[0] = graspTemp;
      }
      collisionCheck();
      if(selectedGrasp == 0) {
        return(nGrasp);
      }
    }
  }
  // std::sort(graspsPossible.begin(),graspsPossible.end(),compareGrasp);

  // For thin objects grasp pair would not be feasible, so each point is considered as a grasp pair
  // Adding these grasps in the end
  Eigen::Vector3f xyPlaneA(0,0,1);
  Eigen::Vector3f xyPlaneB(0,0,-1);
  for(int i = 0; i < ptrPtCldObject->size(); i++){

    A = std::min(pcl::getAngle3D(xyPlaneA,ptrObjNormal->points[i].getNormalVector3fMap()),
                 pcl::getAngle3D(xyPlaneB,ptrObjNormal->points[i].getNormalVector3fMap()))*180/M_PI;

    // If the point is too close to table and its normal vector is along z axis this skip it
    if (A > 45 && ptrPtCldObject->points[i].z < tableCentre[2]+0.02){
      continue;
    }
    graspTemp.p1 = ptrPtCldObject->points[i];
    // Translating it along the +ve normal vector
    graspTemp.p1.x += (voxelGridSize)/2*ptrObjNormal->points[i].normal_x;
    graspTemp.p1.y += (voxelGridSize)/2*ptrObjNormal->points[i].normal_y;
    graspTemp.p1.z += (voxelGridSize)/2*ptrObjNormal->points[i].normal_z;

    graspTemp.p2 = ptrPtCldObject->points[i];
    // Translating it along the -ve normal vector
    graspTemp.p2.x -= (voxelGridSize)/2*ptrObjNormal->points[i].normal_x;
    graspTemp.p2.y -= (voxelGridSize)/2*ptrObjNormal->points[i].normal_y;
    graspTemp.p2.z -= (voxelGridSize)/2*ptrObjNormal->points[i].normal_z;

    graspTemp.gripperWidth = voxelGridSize;
    graspTemp.quality = 180;
    nGrasp++;
    if (graspsPossible.size() == 0){
      graspsPossible.push_back(graspTemp);
    }else{
      graspsPossible[0] = graspTemp;
    }
    collisionCheck();
    if(selectedGrasp == 0){
      return(nGrasp);
    }
  }
  return(nGrasp);
}

// ******************** ENVIRONMENT CLASS FUNCTIONS END ********************

// Function to do a single pass
void singlePass(environment &av, std::vector<double> kinectPose, bool firstTime, bool findGrasp){
  av.moveKinectViewsphere(kinectPose);
  av.readKinect();
  av.fuseLastData();
  av.dataExtract();
  if (firstTime == true) av.genUnexploredPtCld();
  av.updateUnexploredPtCld();
  if (findGrasp == true){
    av.graspAndCollisionCheck();
    // av.graspsynthesis(); av.collisionCheck();
  }
}

// ******************** SET OF FUNCTIONS FOR TESTING ********************

// 1: A test function spawn and delete objects in gazebo
void testSpawnDeleteObj(environment &av){
  std::cout << "*** In object spawn and delete testing function ***" << std::endl;
  int flag = 0;
  for (int i = 0; i < av.objectDict.size(); i++) {
    for (int j = 0; j < av.objectPosesDict[i].size(); j++) {
      av.spawnObject(i,j,0);
      printf("Object %d/%d with configuration %d/%d spawned. Enter any key to continue. ",
             i+1,int(av.objectDict.size()),
             j+1,int(av.objectPosesDict[i].size()));
      std::cin >> flag;
      av.deleteObject(i);
      boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    }
  }
  std::cout << "*** End ***" << std::endl;
}

// 2A: A test function to check if the "moveKinect" functions are working
void testKinectMovement(environment &av){
  std::cout << "*** In kinect movement testing function ***" << std::endl;
  int flag = 0;
  do {
    std::cout << "Enter your choice 1:Cartesian, 2:Viewsphere, 0:Exit : "; std::cin >> flag;
    if (flag == 1) {
      std::vector<double> pose(6);
      std::cout << "Enter kinect pose data" << std::endl;
      std::cout << "X : ";      std::cin >> pose[0];
      std::cout << "Y : ";      std::cin >> pose[1];
      std::cout << "Z : ";      std::cin >> pose[2];
      std::cout << "Roll : ";   std::cin >> pose[3];
      std::cout << "Pitch : ";  std::cin >> pose[4];
      std::cout << "Yaw : ";    std::cin >> pose[5];

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

// 2B: A test function to move the kinect in a viewsphere continuously
void testMoveKinectInViewsphere(environment &av){
  std::cout << "*** In Kinect move in viewsphere testing function ***" << std::endl;
  for (double i = M_PI/2; i > 0;) {
    for (double j = 0; j < 2*M_PI;) {
      av.moveKinectViewsphere({1.4,j,i});
      j += 2*M_PI/10;
    }
    std::cout << (int)((M_PI/2-i)/(M_PI/2/10)+1)<< " out of 10 completed." << std::endl;
    i -= M_PI/2/10;
  }
  std::cout << "*** End ***" << std::endl;
}

// 3: A test function to check if the "readKinect" function is working
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
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
    viewer->initCameraParameters();
    int vp(0);
    viewer->createViewPort(0.0,0.0,1.0,1.0,vp);
    viewer->addCoordinateSystem(1.0);
    viewer->setCameraPosition(0,0,-1,0,0,1,0,-1,0);

    // Adding the point cloud
    rbgVis(viewer,av.cPtrPtCldLast,"Raw Data",vp);

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

// 4: A test function to check fusing of data
void testPtCldFuse(environment &av, int objID, int flag){
  std::cout << "*** In point cloud data fusion testing function ***" << std::endl;
  av.spawnObject(objID,0,0);

  // Setting up the point cloud visualizer
  ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
  viewer->initCameraParameters();
  int vp[4] = {};
  viewer->createViewPort(0.0,0.5,0.5,1.0,vp[0]);
  viewer->createViewPort(0.5,0.5,1.0,1.0,vp[1]);
  viewer->createViewPort(0.0,0.0,0.5,0.5,vp[2]);
  viewer->createViewPort(0.5,0.0,1.0,0.5,vp[3]);
  viewer->addCoordinateSystem(1.0);
  viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

  // 4 kinect position to capture and fuse
  std::vector<std::vector<double>> kinectPoses = {{1.4,-M_PI,M_PI/3},
                                                  {1.4,-M_PI/2,M_PI/3},
                                                  {1.4,0,M_PI/3},
                                                  {1.4,M_PI/2,M_PI/3}};

  for (int i = 0; i < 4; i++) {
    av.moveKinectViewsphere(kinectPoses[i]);
    av.readKinect();
    av.fuseLastData();
    if (flag == 1){
      rbgVis(viewer,av.cPtrPtCldEnv,"Fuse "+std::to_string(i),vp[i]);
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

// 5: A test function to extract table and object data
void testDataExtract(environment &av, int objID, int flag){
  std::cout << "*** In table and object extraction testing function ***" << std::endl;
  av.spawnObject(objID,0,0);

  std::vector<double> kinectPose = {1.4,-M_PI,M_PI/3};
  av.moveKinectViewsphere(kinectPose);
  av.readKinect();
  av.fuseLastData();
  av.dataExtract();

  if(flag==1){
    // Setting up the point cloud visualizer
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
    viewer->initCameraParameters();
    int vp[2] = {};
    viewer->createViewPort(0.0,0.0,0.5,1.0,vp[0]);
    viewer->createViewPort(0.5,0.0,1.0,1.0,vp[1]);
    viewer->addCoordinateSystem(1.0);
    viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

    // ADding the point clouds
    rbgVis(viewer,av.cPtrPtCldTable,"Table",vp[0]);
    rbgVis(viewer,av.cPtrPtCldObject,"Object",vp[1]);
    std::cout << "Showing the table and object extacted. Close viewer to continue" << std::endl;

    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
  av.deleteObject(objID);
  std::cout << "*** End ***" << std::endl;
}

// 6: A test function to generate unexplored point cloud
void testGenUnexpPtCld(environment &av, int objID, int flag){
  std::cout << "*** In unexplored point cloud generation testing function ***" << std::endl;
  av.spawnObject(objID,0,0);

  std::vector<double> kinectPose = {1.4,-M_PI,M_PI/3};
  av.moveKinectViewsphere(kinectPose);
  av.readKinect();
  av.fuseLastData();
  av.dataExtract();
  av.genUnexploredPtCld();

  if(flag==1){
    // Setting up the point cloud visualizer
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
    viewer->initCameraParameters();
    int vp(0);
    viewer->createViewPort(0.0,0.0,1.0,1.0,vp);
    viewer->addCoordinateSystem(1.0);
    viewer->setCameraPosition(2,1,3,-1,-1,-1,-1,-1,1);

    // Adding the point clouds
    rbgVis(viewer,av.cPtrPtCldObject,"Object",vp);
    rbgVis(viewer,av.cPtrPtCldUnexp,"Unexplored pointcloud",vp);
    std::cout << "Showing the object extacted and unexplored point cloud generated. Close viewer to continue" << std::endl;
    while (!viewer->wasStopped()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
  av.deleteObject(objID);
  std::cout << "*** End ***" << std::endl;
}

// 7: A test function to update unexplored point cloud
void testUpdateUnexpPtCld(environment &av, int objID, int flag){
  std::cout << "*** In unexplored point cloud update testing function ***" << std::endl;
  av.spawnObject(objID,0,0);

  // Setting up the point cloud visualizer
  ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
  viewer->initCameraParameters();
  int vp[4] = {};
  viewer->createViewPort(0.0,0.5,0.5,1.0,vp[0]);
  viewer->createViewPort(0.5,0.5,1.0,1.0,vp[1]);
  viewer->createViewPort(0.0,0.0,0.5,0.5,vp[2]);
  viewer->createViewPort(0.5,0.0,1.0,0.5,vp[3]);
  viewer->addCoordinateSystem(1.0);
  viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

  // 4 kinect position to capture and fuse
  std::vector<std::vector<double>> kinectPoses = {{1.4,-M_PI,M_PI/3},
                                                  {1.4,-M_PI/2,M_PI/3},
                                                  {1.4,0,M_PI/3},
                                                  {1.4,M_PI/2,M_PI/3}};

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
      rbgVis(viewer,av.cPtrPtCldEnv,"Env "+std::to_string(i),vp[i]);
      rbgVis(viewer,av.ptrPtCldUnexp,"Unexp "+std::to_string(i),vp[i]);
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

// 8: A test function to load and update gripper
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
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
    viewer->initCameraParameters();
    int vp(0);
    viewer->createViewPort(0.0,0.0,1.0,1.0,vp);
    viewer->setCameraPosition(0.5,0,0,-1,0,0,0,0,1);

    // Adding the point cloud
    rbgVis(viewer,av.cPtrPtCldGripper,"Gripper",vp);
    for (int i = 0; i < 3; i++) {
      viewer->addCube(av.minPtCol[i].x,av.maxPtCol[i].x,
                      av.minPtCol[i].y,av.maxPtCol[i].y,
                      av.minPtCol[i].z,av.maxPtCol[i].z,0.0,1.0,0.0,"Cube"+std::to_string(i),vp);
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "Cube"+std::to_string(i));
    }
    for (int i = 3; i < 5; i++) {
      viewer->addCube(av.minPtCol[i].x,av.maxPtCol[i].x,
                      av.minPtCol[i].y,av.maxPtCol[i].y,
                      av.minPtCol[i].z,av.maxPtCol[i].z,1.0,0.0,0.0,"Cube"+std::to_string(i),vp);
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

// 9: Grasp synthesis test function
void testGraspsynthesis(environment &av, int objID, int flag){
  std::cout << "*** In grasp synthesis testing function ***" << std::endl;
  av.spawnObject(objID,0,0);

  // 4 kinect position
  std::vector<std::vector<double>> kinectPoses = {{1.4,-M_PI,M_PI/3},
                                                  {1.4,-M_PI/2,M_PI/3},
                                                  {1.4,0,M_PI/3},
                                                  {1.4,M_PI/2,M_PI/3}};

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
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
    viewer->initCameraParameters();
    int vp(0);
    viewer->createViewPort(0.0,0.0,1.0,1.0,vp);
    viewer->addCoordinateSystem(1.0);
    viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

    // Adding the point clouds
    rbgVis(viewer,av.cPtrPtCldEnv,"Environment",vp);
    if (av.graspsPossible.size() > 3){
      for (int i = 0; i < 3; i++){
        viewer->addSphere<pcl::PointXYZRGB>(av.graspsPossible[i].p1,0.0050,0.0,0.0,(i+1.0)/3.0,"GP_"+std::to_string(i)+"_A",vp);
        viewer->addSphere<pcl::PointXYZRGB>(av.graspsPossible[i].p2,0.0050,0.0,0.0,(i+1.0)/3.0,"GP_"+std::to_string(i)+"_B",vp);
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

// 10A: A test function to check the collision check algorithm with dummy data
void testCollisionDummy(environment &av, bool result, int flag){
  std::cout << "*** In dummy collision testing function ***" << std::endl;

  graspPoint graspTemp; // Creating a dummy grasp
  av.graspsPossible.push_back(graspTemp);

  av.loadGripper();

  // Creating a dummy unexplored point cloud
  av.ptrPtCldUnexp->clear();
  pcl::common::CloudGenerator<pcl::PointXYZRGB, pcl::common::UniformGenerator<float>> generator;
  std::uint32_t seed = static_cast<std::uint32_t> (time (nullptr));
  pcl::common::UniformGenerator<float>::Parameters x_params(-0.03, 0.03, seed++);
  generator.setParametersForX(x_params);
  pcl::common::UniformGenerator<float>::Parameters y_params(-0.15, 0.15, seed++);
  generator.setParametersForY(y_params);
  if(result == true){
    pcl::common::UniformGenerator<float>::Parameters z_params(-0.15, -0.02, seed++);
    generator.setParametersForZ(z_params);
  }else{
    pcl::common::UniformGenerator<float>::Parameters z_params(-0.15, 0.15, seed++);
    generator.setParametersForZ(z_params);
  }

  int dummy = generator.fill(5000, 1, *av.ptrPtCldUnexp);
  // Setting color to blue
  for (int i = 0; i < av.ptrPtCldUnexp->size(); i++) {
    av.ptrPtCldUnexp->points[i].b = 200;
  }

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
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
    viewer->initCameraParameters();
    int vp = {};
    viewer->createViewPort(0.0,0.0,1.0,1.0,vp);
    // viewer->addCoordinateSystem(1.0);
    viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

    rbgVis(viewer,av.ptrPtCldUnexp,"Unexplored",vp);
    rbgVis(viewer,av.ptrPtCldCollided,"Collision",vp);
    av.updateGripper(0,0);    // Only for visulization purpose
    rbgVis(viewer,av.ptrPtCldGripper,"Gripper",vp);
    std::cout << "Showing the Gripper(Black), Unexplored(Blue), Collision(Red) points. Close viewer to continue" << std::endl;

    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
  std::cout << "*** End ***" << std::endl;
}

// 10B: A test function to check the collision check algorithm with object and grasp points
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
  std::vector<std::vector<double>> kinectPoses = {{1.4,-M_PI,M_PI/3},
                                                  {1.4,-M_PI/2,M_PI/3},
                                                  {1.4,0,M_PI/3},
                                                  {1.4,M_PI/2,M_PI/3}};

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
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
    viewer->initCameraParameters();
    int vp[2] = {};
    viewer->createViewPort(0.0,0.0,0.5,1.0,vp[0]);
    viewer->createViewPort(0.5,0.0,1.0,1.0,vp[1]);
    viewer->addCoordinateSystem(1.0);
    viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

    rbgVis(viewer,av.ptrPtCldEnv,"Environment",vp[0]);
    rbgVis(viewer,av.ptrPtCldObject,"Object",vp[1]);
    // rbgNormalVis(viewer,av.ptrPtCldObject,av.ptrObjNormal,"Object",vp[1]);

    for (int i = 0; i < av.ptrPtCldUnexp->size(); i++) {
      av.ptrPtCldUnexp->points[i].r = 0;
      av.ptrPtCldUnexp->points[i].b = 200;
      av.ptrPtCldUnexp->points[i].g = 0;
    }
    rbgVis(viewer,av.ptrPtCldUnexp,"Unexplored",vp[1]);

    if(av.selectedGrasp == -1){
      std::cout << "No grasp orientation for the grasp points found." << std::endl;
      std::cout << "Showing the object (blue), collision check points (red). Close viewer to continue" << std::endl;
    }else{
      av.updateGripper(av.selectedGrasp,0);    // Only for visulization purpose
      rbgVis(viewer,av.ptrPtCldGripper,"Gripper",vp[0]);
      rbgVis(viewer,av.ptrPtCldGripper,"Gripper1",vp[1]);
      std::cout << "Showing the object (blue), collision check points (red), selected gripper position (black). Close viewer to continue" << std::endl;
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

// 11: A test function to check rollback feature
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

  std::vector<double> kinectPose = {1.4,-M_PI,M_PI/3};
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
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
    int vp[2] = {};
    viewer->initCameraParameters();
    viewer->createViewPort(0.0,0.0,0.5,1.0,vp[0]);
    viewer->createViewPort(0.5,0.0,1.0,1.0,vp[1]);
    viewer->addCoordinateSystem(1.0);
    viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

    rbgVis(viewer,av.ptrPtCldEnv,"Environment",vp[0]);
    rbgVis(viewer,av.ptrPtCldObject,"Object",vp[1]);
    // rbgNormalVis(viewer,av.ptrPtCldObject,av.ptrObjNormal,"Object",vp[1]);

    for (int i = 0; i < av.ptrPtCldUnexp->size(); i++) {
      av.ptrPtCldUnexp->points[i].r = 0;
      av.ptrPtCldUnexp->points[i].b = 200;
      av.ptrPtCldUnexp->points[i].g = 0;
    }
    rbgVis(viewer,av.ptrPtCldUnexp,"Unexplored",vp[1]);

    if(av.selectedGrasp != -1){
      av.updateGripper(av.selectedGrasp,0);    // Only for visulization purpose
      rbgVis(viewer,av.ptrPtCldGripper,"Gripper",vp[0]);
      rbgVis(viewer,av.ptrPtCldGripper,"Gripper1",vp[1]);
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

      rbgVis(viewer,av.ptrPtCldEnv,"Environment",vp[0]);
      rbgVis(viewer,av.ptrPtCldObject,"Object",vp[1]);
      // rbgNormalVis(viewer,av.ptrPtCldObject,av.ptrObjNormal,"Object",vp[1]);

      for (int i = 0; i < av.ptrPtCldUnexp->size(); i++) {
        av.ptrPtCldUnexp->points[i].r = 0;
        av.ptrPtCldUnexp->points[i].b = 200;
        av.ptrPtCldUnexp->points[i].g = 0;
      }
      rbgVis(viewer,av.ptrPtCldUnexp,"Unexplored",vp[1]);

      if(av.selectedGrasp != -1){
        av.updateGripper(av.selectedGrasp,0);    // Only for visulization purpose
        rbgVis(viewer,av.ptrPtCldGripper,"Gripper",vp[0]);
        rbgVis(viewer,av.ptrPtCldGripper,"Gripper1",vp[1]);
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

// 12: A function to test saving pointcloud
void testSavePCD(environment &av, int objID){
  std::cout << "*** In point cloud save testing function ***" << std::endl;
  std::string dir = "./DataRecAV/";
  std::string time;
  ptCldColor::Ptr ptrPtCldTemp{new ptCldColor};

  av.spawnObject(objID,0,0);
  av.loadGripper();

  // 4 kinect poses
  std::vector<std::vector<double>> kinectPoses = {{1.4,-M_PI,M_PI/3},
                                                  {1.4,-M_PI/2,M_PI/3},
                                                  {1.4,0,M_PI/3},
                                                  {1.4,M_PI/2,M_PI/3}};

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

// 13: A function to test reading csv file
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

// 14: A function to test reading PCD based on CSV file data
void testReadPCD(std::string filename){
  std::cout << "*** In PCD read testing function ***" << std::endl;
  std::string dir = "./DataRecAV/";
  int colID = 12;
  ptCldColor::Ptr ptrPtCldTemp{new ptCldColor};
  std::vector<std::vector<std::string>> data;
  data = readCSV(filename);

  // Setting up the point cloud visualizer
  ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
  int vp[3] = {};
  viewer->createViewPort(0.0,0.5,0.5,1.0,vp[0]);
  viewer->createViewPort(0.5,0.5,1.0,1.0,vp[1]);
  viewer->createViewPort(0.25,0.0,0.75,0.5,vp[2]);
  viewer->addCoordinateSystem(1.0);
  viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

  std::cout << "Viewing first 4 data recerded." << std::endl;
  for(int i = 0; i < 4; i++){
    if (colID <= data[i].size()){
      readPointCloud(ptrPtCldTemp,dir,data[i][colID-1],1);
      rbgVis(viewer,ptrPtCldTemp,"Obj",vp[0]);
      readPointCloud(ptrPtCldTemp,dir,data[i][colID-1],2);
      rbgVis(viewer,ptrPtCldTemp,"Unexp",vp[1]);
      readPointCloud(ptrPtCldTemp,dir,data[i][colID-1],3);
      rbgVis(viewer,ptrPtCldTemp,"Env",vp[2]);

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

int main (int argc, char** argv){

  // Initialize ROS
  ros::init (argc, argv, "Testing_Model");
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
  std::cout << "Enter your choice : "; cin >> choice;

  if (choice >= 5 && choice <= 13) {
    std::cout << "Objects available :" << std::endl;
    std::cout << "1: Drill" << std::endl;
    std::cout << "2: Square Prism" << std::endl;
    std::cout << "3: Rectangular Prism" << std::endl;
    std::cout << "4: Bowl" << std::endl;
    std::cout << "5: Big Bowl" << std::endl;
    std::cout << "6: Cup" << std::endl;
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
      testKinectRead(activeVision,objID-1,1);         break;
    case 6:
      testPtCldFuse(activeVision,objID-1,1);          break;
    case 7:
      testDataExtract(activeVision,objID-1,1);        break;
    case 8:
      testGenUnexpPtCld(activeVision,objID-1,1);      break;
    case 9:
      testUpdateUnexpPtCld(activeVision,objID-1,1);   break;
    case 10:
      testGraspsynthesis(activeVision,objID-1,1);     break;
    case 11:
      // for (int i = 1; i <= 4; i++) {
      //   for (int j = 0; j <= 5; j++) {
      //     testComplete(activeVision,j,i,graspMode,0,1);
      //     activeVision.reset();
      //   }
      // }
      testComplete(activeVision,objID-1,4,graspMode,1,0);
      break;
    case 12:
      testSaveRollback(activeVision,objID-1,1);  break;
    case 13:
      testSavePCD(activeVision,objID-1);         break;
    case 14:
      testReadCSV(filename,colID);               break;
    case 15:
      testReadPCD(filename);                     break;
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
