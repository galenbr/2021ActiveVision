#include <active_vision/environment.h>

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
  objectDict = {{"prismAV6x6x6","Prism 6x6x6"},
                {"prismAV10x8x4","Prism 10x8x4"},
                {"prismAV20x6x5","Prism 20x6x5"},
                {"bowlAV","Bowl"},
                {"cinderBlockAV","Cinder Block"},
                {"handleAV","Door Handle"},
                {"gasketAV","Gasket"},
                {"drillAV","Cordless Drill"}};

  // Stores stable poses for the objects (Z(m), Roll(Rad), Pitch(Rad))
  objectPosesDict = {{{0.050,0.000,0.000}},
                     {{0.060,1.570,0.000},{0.070,1.570,1.570}},
                     {{0.040,0.000,0.000},{0.050,1.570,0.000},{0.120,1.570,1.570}},
                     {{0.015,0.000,0.000}},
                     {{0.015,0.000,0.000}},
                     {{0.015,0.000,0.000}},
                     {{0.015,0.000,0.000}},
                     {{0.012,0.000,0.000},{0.084,1.459,-0.751},{0.082,1.398,-0.014},{0.040,0.280,-1.040},{0.249,2.860,-0.400},{0.068,-1.287,-0.477},{0.048,-1.570,1.480}}};

  objectPosesYawLimits = {{0,89},{0,179},{0,179},{0,179},{0,1},{0,89},{0,359},{0,179},{0,359}};

  voxelGridSize = 0.01;          // Voxel Grid size for environment
  voxelGridSizeUnexp = 0.01;     // Voxel Grid size for unexplored point cloud
  viewsphereRad = 1;
  tableCentre = {1.5,0,1};       // Co-ordinates of table centre
  minUnexp = {0,0,0};
  maxUnexp = {0,0,0};
  scale = 3;                     // Scale value for unexplored point cloud generation
  maxGripperWidth = 0.08;        // Gripper max width
  minGraspQuality = 150;         // Min grasp quality threshold
  selectedGrasp = -1;            // Index of the selected grasp

  addNoise = false;
  depthNoise = 0;
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
  configTemp.unexpMin = minUnexp;
  configTemp.unexpMax = maxUnexp;
  configurations.push_back(configTemp);
  //std::cout << "State saved : " << name << std::endl;
  return configurations.size()-1;
}

// Rollback to a configuration
void environment::rollbackConfiguration(int index){
  *ptrPtCldEnv = configurations[index].env;
  *ptrPtCldUnexp = configurations[index].unexp;
  lastKinectPoseViewsphere = configurations[index].kinectPose;
  minUnexp = configurations[index].unexpMin;
  maxUnexp = configurations[index].unexpMax;
  //std::cout << "Rolled back to state : " << configurations[index].description << std::endl;
}

// 1A: Callback function to point cloud subscriber
void environment::cbPtCld(const ptCldColor::ConstPtr& msg){
  if(readFlag[0]==1){
    *ptrPtCldLast = *msg;
    if(addNoise == true){
      for(int i = 0; i < ptrPtCldLast->points.size(); i++){
        if(!isinf(ptrPtCldLast->points[i].z)){
          float stdDev = (ptrPtCldLast->points[i].z)*depthNoise/100;
          std::normal_distribution<float> normDistb{0,stdDev};
          float noise = normDistb(generator);
          // Truncating to 1 sigma limit
          noise = std::min(noise,stdDev); noise = std::max(-stdDev,noise);
          ptrPtCldLast->points[i].z += noise;
        }
      }
    }
    readFlag[0] = 0;
  }
}

// Function to set noise variables
void environment::setPtCldNoise(float num){

  depthNoise = abs(num);
  if(depthNoise != 0) addNoise = true;
  else addNoise = false;
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
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.01+viewsphereRad*depthNoise/100);
  Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0); //y axis
  seg.setAxis(axis);
  seg.setEpsAngle(10.0f*(M_PI/180.0f) );
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

  Eigen::Vector4f centroid, minTab, maxTab;
  pcl::compute3DCentroid(*ptrPtCldTable, centroid);
  pcl::getMinMax3D(*ptrPtCldTable, minTab, maxTab);
  // std::cout << tableCoeff->values[0] << " " << tableCoeff->values[1] << " " << tableCoeff->values[2] << " " << tableCoeff->values[3] << std::endl;
  // std::cout << minTab[2] << " " << centroid[2] << " " << maxTab[2] << std::endl;

  // Using convex hull to get the table boundary which would be like a rectangle
  cvHull.setInputCloud(cPtrPtCldTable);
  cvHull.setDimension(2);
  cvHull.reconstruct(*ptrPtCldHull);

  // Double checking the hull dimensions
  if(cvHull.getDimension() != 2){
    std::cerr << "Convex hull dimension != 2" << std::endl;
    return;
  }

  // Using polygonal prism and hull the extract object above the table
  prism.setInputCloud(cPtrPtCldEnv);
  prism.setInputPlanarHull(cPtrPtCldHull);
  if(tableCoeff->values[3] < 0){
    prism.setHeightLimits(-1.5f,-(maxTab[2]-centroid[2]+viewsphereRad*depthNoise/100+0.005)); // Z height (min, max) in m
  }else{
    prism.setHeightLimits(maxTab[2]-centroid[2]+viewsphereRad*depthNoise/100+0.005,1.5f);     // Z height (min, max) in m
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
  // Min of 0.40m on each side
  // Note: Z scale is only used on +z axis
  minUnexp[0] = (minPtObj.x-std::max((scale-1)*(maxPtObj.x-minPtObj.x)/2,0.40f));
  minUnexp[1] = (minPtObj.y-std::max((scale-1)*(maxPtObj.y-minPtObj.y)/2,0.40f));
  minUnexp[2] = 1;
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
    proj[0] = round(proj[0]);
    proj[1] = round(proj[1]);
    if(proj[0] >=0 && proj[0] <= 640-1 && proj[1] >=0 && proj[1] <= 480-1){
      projIndex = proj[1]*(ptrPtCldLast->width)+proj[0];
      // If the z value of unexplored pt is greater than the corresponding
      // projected point in Kinect Raw data then that point is occluded.
      if (ptrPtCldLast->points[projIndex].z <= ptTemp[2]){
        occludedIndices->indices.push_back(i);
      }
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

  for(int i = 0; i < ptrPtCldObject->size()-1; i=i+2){
    for(int j = i+1; j < ptrPtCldObject->size(); j=j+2){
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
