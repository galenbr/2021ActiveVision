#include <active_vision/toolGraspSynthesis.h>

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

Eigen::Affine3f calcTfFromNormal(pcl::Normal normal, pcl::PointXYZRGB point){

  Eigen::Matrix4f tfMat; tfMat.setIdentity();
  Eigen::Affine3f tf;
  Eigen::Matrix3f rot;
  Eigen::Vector3f trans;
  calcTfFromNormal(normal, point, rot, trans);

  tfMat.block<3,3>(0,0) = rot;
  tfMat.block<3,1>(0,3) = trans;
  tf.matrix() = tfMat;

  return tf;
}

void calcTfFromNormal(pcl::Normal normal, pcl::PointXYZRGB point, Eigen::Matrix3f &rot, Eigen::Vector3f &trans){
  Eigen::Matrix4f tfMat; tfMat.setIdentity();
  Eigen::Affine3f tf;

  Eigen::Vector3f xAxis,yAxis,zAxis,xyPlane(0,0,1);

  xAxis = {normal.normal_x,normal.normal_y,normal.normal_z}; xAxis.normalize();
  yAxis = xAxis.cross(xyPlane);
  if(yAxis.norm() == 0) yAxis << normal.normal_x,0,0;
  yAxis.normalize();
  zAxis = xAxis.cross(yAxis);

  rot << xAxis[0], yAxis[0], zAxis[0],
         xAxis[1], yAxis[1], zAxis[1],
         xAxis[2], yAxis[2], zAxis[2];
  trans = point.getVector3fMap();
}

// Check if franka can reach the point
bool checkFrankReach(ros::ServiceClient &IKClient, geometry_msgs::Pose &p){
  moveit_planner::Inv poseIKMsg;
  poseIKMsg.request.pose = p;
  return IKClient.call(poseIKMsg);
}

graspPoint::graspPoint(){
  quality = 0;              // Quality of the grasp
  gripperWidth = 0.05;      // Gripper width for the grasp
  pose = {0,0,0,0,0,0};     // Base pose of the gripper from where additional pitch is used to change its orientation
  addnlPitch = 0;           // Pitch relative to the Base Pose         
}

// Function to compare grasp point for sorting
bool compareGrasp(graspPoint A, graspPoint B){
  // Approach 2
  if(abs(A.lineDistance - B.lineDistance) <= 0.01){
    if(abs(A.pose[2] - B.pose[2]) <= 0.06){
      if(abs(A.distance - B.distance) <= 0.01){
        return(A.quality > B.quality);
      }else{
        return(A.distance < B.distance);
      }
    }
    else{
      return(A.pose[2] > B.pose[2]);
    }
  }else{
   return(A.lineDistance < B.lineDistance);
  }
  // Approach 1
  // if(abs(A.distance - B.distance) <= 0.005){
  //   return(A.quality > B.quality);
  // }else{
  //   return(A.distance < B.distance);
  // }
}

// Setting up the grasp parameters
graspSynthesis::graspSynthesis(ros::NodeHandle *nh){
  path = ros::package::getPath("active_vision");                                   // Path to the active_vision package folder
  
  nh->getParam("/active_vision/simulationMode", simulationMode);

  nh->getParam("/active_vision/graspSynthesis/fingerZOffset", fingerZOffset);      // Z axis offset between gripper hand and finger
  
  nh->getParam("/active_vision/graspSynthesis/maxGripperWidth", maxGripperWidth);  // Gripper max width
  if(simulationMode == "FRANKA")  maxGripperWidth += 0.01;

  nh->getParam("/active_vision/graspSynthesis/minGraspQuality", minGraspQuality);  // Min grasp quality threshold
  selectedGrasp = -1;                                                              // Index of the selected grasp

  nh->getParam("/active_vision/graspSynthesis/graspCurvatureConstraint", graspCurvatureConstraint);
  nh->getParam("/active_vision/graspSynthesis/graspSurPatchConstraint", graspSurPatchConstraint);

  if(simulationMode != "SIMULATION"){
    IKClient =  nh->serviceClient<moveit_planner::Inv>("inverse_kinematics_collision_check");
  }

  loadGripper();
}

// Load Gripper Hand and Finger file
void graspSynthesis::loadGripper(){
  std::string pathToHand =   path+"/models/gripperAV/hand.ply";
  std::string pathToFinger = path+"/models/gripperAV/finger.ply";
  std::string pathToCamera = path+"/models/gripperAV/realsense.ply";
  
  // Gripper Hand
  if(pcl::io::loadPLYFile<pcl::PointXYZRGB>(pathToHand, *ptrPtCldGrpHnd) == -1)    PCL_ERROR ("Couldn't read file hand.ply \n");
  // Gripper Camera
  if(pcl::io::loadPLYFile<pcl::PointXYZRGB>(pathToCamera, *ptrPtCldGrpCam) == -1)  PCL_ERROR ("Couldn't read file realsense.ply \n");
  // Gripper Left Finger
  if(pcl::io::loadPLYFile<pcl::PointXYZRGB>(pathToFinger, *ptrPtCldGrpLfgr) == -1) PCL_ERROR ("Couldn't read file finger.ply \n");
  // Gripper Right Finger (Mirror of the left one)
  pcl::transformPointCloud(*ptrPtCldGrpLfgr, *ptrPtCldGrpRfgr, pcl::getTransformation(0,0,0,0,0,M_PI));

  // Find the min max 3D coordinates for the three segments
  pcl::getMinMax3D(*ptrPtCldGrpHnd, minPtGrp[0],maxPtGrp[0]);
  pcl::getMinMax3D(*ptrPtCldGrpLfgr,minPtGrp[1],maxPtGrp[1]);
  pcl::getMinMax3D(*ptrPtCldGrpRfgr,minPtGrp[2],maxPtGrp[2]);
  pcl::getMinMax3D(*ptrPtCldGrpCam, minPtGrp[3],maxPtGrp[3]);
  std::cout << "Ignore the PLY reader error on 'face' and 'rgb'." << std::endl;
}

// Update gripper
// 0 -> Visualization
// 1 -> Axis Collision Check
// 2 -> Gripper Collision Check
void graspSynthesis::updateGripper(int index ,int choice){

  // Gripper orientation which is used in cropbox during collision check
  tfGripper = pcl::getTransformation(graspsPossible[index].pose[0],graspsPossible[index].pose[1],
                                     graspsPossible[index].pose[2],graspsPossible[index].pose[3],
                                     graspsPossible[index].pose[4],graspsPossible[index].pose[5])*
              pcl::getTransformation(0,0,0,0,graspsPossible[index].addnlPitch,0)*
              pcl::getTransformation(0,0,-0.0447-fingerZOffset,0,0,0);

  if(choice == 0){
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

    // adding the camera
    *ptrPtCldGripper += *ptrPtCldGrpCam;

    pcl::transformPointCloud(*ptrPtCldGripper, *ptrPtCldGripper, tfGripper);
    ptrPtCldTemp->clear();

  }else if(choice == 1){
    // Left Finger Basic Check
    minPtCol[3].x = -0.0125; maxPtCol[3].x = 0.0125;
    minPtCol[3].y =  0.0;    maxPtCol[3].y = 0.0250;
    minPtCol[3].z =  0.0322; maxPtCol[3].z = 0.0572;
    // Applying transformation for the gripper width
    minPtCol[3] = pcl::transformPoint(minPtCol[3],pcl::getTransformation(0,graspsPossible[index].gripperWidth/2,fingerZOffset,0,0,0));
    maxPtCol[3] = pcl::transformPoint(maxPtCol[3],pcl::getTransformation(0,graspsPossible[index].gripperWidth/2,fingerZOffset,0,0,0));

    // Right Finger Basic Check
    minPtCol[4].x = -0.0125; maxPtCol[4].x = 0.0125;
    minPtCol[4].y = -0.0250; maxPtCol[4].y = 0.0;
    minPtCol[4].z =  0.0322; maxPtCol[4].z = 0.0572;
    // Applying transformation for the gripper width
    minPtCol[4] = pcl::transformPoint(minPtCol[4],pcl::getTransformation(0,-graspsPossible[index].gripperWidth/2,fingerZOffset,0,0,0));
    maxPtCol[4] = pcl::transformPoint(maxPtCol[4],pcl::getTransformation(0,-graspsPossible[index].gripperWidth/2,fingerZOffset,0,0,0));

  }else if(choice == 2){
    // Hand
    minPtCol[0] = minPtGrp[0];
    maxPtCol[0] = maxPtGrp[0];

    // Camera
    minPtCol[5] = minPtGrp[3];
    maxPtCol[5] = maxPtGrp[3]; maxPtCol[5].x += 0.005; maxPtCol[5].z += 0.005;

    // Left Finger
    minPtCol[1] = pcl::transformPoint(minPtGrp[1],pcl::getTransformation(0,graspsPossible[index].gripperWidth/2,fingerZOffset,0,0,0));
    maxPtCol[1] = pcl::transformPoint(maxPtGrp[1],pcl::getTransformation(0,graspsPossible[index].gripperWidth/2,fingerZOffset,0,0,0));

    // Right Finger
    minPtCol[2] = pcl::transformPoint(minPtGrp[2],pcl::getTransformation(0,-graspsPossible[index].gripperWidth/2,fingerZOffset,0,0,0));
    maxPtCol[2] = pcl::transformPoint(maxPtGrp[2],pcl::getTransformation(0,-graspsPossible[index].gripperWidth/2,fingerZOffset,0,0,0));
  }
}

// Estimating if the contact patch is sufficient
bool graspSynthesis::isContactPatchOk(long int ptIdx){
  int w = 100;
  cv::Mat patchMask = cv::Mat::zeros(w+1,w+1,CV_8UC1);
  int patchWidth = 26;
  float patchArea = patchWidth*patchWidth;
  circle(patchMask, cv::Point(w/2,w/2), int(patchWidth/2.0*sqrt(2)), 255, cv::FILLED, cv::LINE_8);

  pcl::PointXYZRGB minBound,maxBound;
  minBound.x = -0.005; maxBound.x = 0.005;
  minBound.y = -0.030; maxBound.y = 0.030;
  minBound.z = -0.030; maxBound.z = 0.030;

  pcl::CropBox<pcl::PointXYZRGB> cpBox;
  cpBox.setInputCloud(ptrPtCldObject);
  cpBox.setMin(minBound.getVector4fMap());
  cpBox.setMax(maxBound.getVector4fMap());

  Eigen::Affine3f tf = calcTfFromNormal(ptrObjNormal->points[ptIdx],ptrPtCldObject->points[ptIdx]);
  cpBox.setRotation(getEuler(tf));
  cpBox.setTranslation(getTranslation(tf));

  static ptCldColor::Ptr objFiltered{new ptCldColor};
  cpBox.filter(*objFiltered);
  Eigen::Affine3f tfTranspose = homoMatTranspose(tf);
  pcl::transformPointCloud(*objFiltered, *objFiltered, tfTranspose);

  cv::Mat surface = cv::Mat::zeros(w+1,w+1,CV_8UC1);
  Eigen::Vector3f proj;
  std::vector<cv::Point> projPts;
  for(int i = 0; i<objFiltered->points.size(); i++){
    proj[0] = int(objFiltered->points[i].y*1000+w/2);
    proj[1] = int(objFiltered->points[i].z*1000+w/2);
    if(proj[0] >= 0 && proj[0] < w+1 && proj[1] >= 0 && proj[1] < w+1){
      projPts.push_back(cv::Point(proj[1],proj[0]));
    }else{
      ROS_WARN("Error in isContactPatchOk projection.");
    }
  }

  std::vector<std::vector<cv::Point>> hullPts(1);
  cv::convexHull(projPts, hullPts[0]);
  cv::drawContours(surface, hullPts, -1, 255,-1);

  cv::Mat surfacePatch; surface.copyTo(surfacePatch, patchMask);
  // Finding the minimum inscribed circle
  double max_val; cv::Point max_loc; cv::Mat1f dt;
  cv::distanceTransform(surfacePatch, dt, cv::DIST_L2, 5, cv::DIST_LABEL_PIXEL);
  cv::minMaxLoc(dt, nullptr, &max_val, nullptr, &max_loc);

  if(max_val >= patchWidth/1.9) return true;
  else                          return false;
}

// Preprocessing the data
void graspSynthesis::preprocessing(){
  // Generating the normals for the object point cloud.
  pcl::search::Search<pcl::PointXYZRGB>::Ptr KdTree{new pcl::search::KdTree<pcl::PointXYZRGB>};
  ne.setInputCloud(cPtrPtCldObject);
  ne.setSearchMethod(KdTree);
  ne.setKSearch(10);
  ne.compute(*ptrObjNormal);

  // Calculating the geometric centroid
  pcl::getMinMax3D(*ptrPtCldObject, minPtObj, maxPtObj);

  *ptrPtCldTemp = *ptrPtCldUnexp + *ptrPtCldObject;
  pass.setInputCloud(cPtrPtCldTemp);
  pass.setFilterFieldName("x"); pass.setFilterLimits(minPtObj.x,maxPtObj.x); pass.filter(*ptrPtCldTemp);
  pass.setFilterFieldName("y"); pass.setFilterLimits(minPtObj.y,maxPtObj.y); pass.filter(*ptrPtCldTemp);
  pass.setFilterFieldName("z"); pass.setFilterLimits(minPtObj.z,maxPtObj.z); pass.filter(*ptrPtCldTemp);
  
  Eigen::Vector4f cenObject; pcl::compute3DCentroid(*ptrPtCldTemp, cenObject);
  centroidObj.x = cenObject[0]; centroidObj.y = cenObject[1]; centroidObj.z = cenObject[2];
  
  // Checking object surfaces for contact patch and curvature constraint
  float avgCurvature;
  if(graspCurvatureConstraint){
    for(int i = 0; i < cPtrPtCldObject->points.size(); i++) avgCurvature += ptrObjNormal->points[i].curvature;
    avgCurvature /= cPtrPtCldObject->points.size();
  }

  useForGrasp.resize(cPtrPtCldObject->points.size());
  for(int i = 0; i < cPtrPtCldObject->points.size(); i++){
    useForGrasp[i] = true;

    // Curvature check
    if(graspCurvatureConstraint && ptrObjNormal->points[i].curvature > 2*avgCurvature){
      useForGrasp[i] = false;
    }

    // Surface Patch constraint check
    if(graspSurPatchConstraint && !(isContactPatchOk(i))){
      useForGrasp[i] = false;
    }

    // Ignoring points closer to the table
    if(cPtrPtCldObject->points[i].z < minPtObj.z+0.01){
      useForGrasp[i] = false;
    }
  }
}

// Given a grasp point pair find the gripper orientation
void graspSynthesis::findGripperPose(int index){

  Eigen::Vector3f xAxis,yAxis,zAxis;
  Eigen::Vector3f xyPlane(0,0,1);

  // Calculating the x,y,z axis which is at the midpoint of the fingers
  yAxis = graspsPossible[index].p1.getVector3fMap() - graspsPossible[index].p2.getVector3fMap(); yAxis.normalize();
  zAxis = yAxis.cross(xyPlane); zAxis.normalize();
  xAxis = yAxis.cross(zAxis);

  // Finding RPY based on the axis directions
  tf::Matrix3x3 rotMat, pitch180, yaw180;
  pitch180.setRPY(0,M_PI,0);
  yaw180.setRPY(0,0,M_PI);
  double Roll,Pitch,Yaw;
  rotMat.setValue(xAxis[0],yAxis[0],zAxis[0],
                  xAxis[1],yAxis[1],zAxis[1],
                  xAxis[2],yAxis[2],zAxis[2]);


  if(rotMat.getColumn(2).getX() < 0) rotMat *= pitch180;
  if(rotMat.getColumn(0).getZ() < 0) rotMat *= yaw180;

  // std::cout << rotMat.getColumn(0).getX()  << "," << rotMat.getColumn(0).getY() << "," << rotMat.getColumn(0).getZ() << std::endl;
  // std::cout << rotMat.getColumn(1).getX()  << "," << rotMat.getColumn(1).getY() << "," << rotMat.getColumn(1).getZ() << std::endl;
  // std::cout << rotMat.getColumn(2).getX()  << "," << rotMat.getColumn(2).getY() << "," << rotMat.getColumn(2).getZ() << std::endl;

  rotMat.getRPY(Roll,Pitch,Yaw);

  // Setting the coordinates as the midpoint between the two fingers i.e. midpoint of the two pointclouds
  std::vector<float> pose = {0,0,0,0,0,0};
  pose[0] = (graspsPossible[index].p1.x + graspsPossible[index].p2.x)/2;
  pose[1] = (graspsPossible[index].p1.y + graspsPossible[index].p2.y)/2;
  pose[2] = (graspsPossible[index].p1.z + graspsPossible[index].p2.z)/2;
  pose[3] = Roll; pose[4] = Pitch; pose[5] = Yaw;

  graspsPossible[index].pose = pose;
}

// Check if a pose is viable for a arm
bool graspSynthesis::checkArmPose(Eigen::Matrix4f tfMat, geometry_msgs::Pose &p){
  tfMat *= pcl::getTransformation(0,0,0,0,-M_PI/2,M_PI).matrix();
  if(simulationMode == "FRANKA") tfMat *= pcl::getTransformation(0,0,0,0,0,M_PI/4).matrix();

  Eigen::Quaternionf quat(tfMat.block<3,3>(0,0));
  p.position.x = tfMat(0,3);  p.position.y = tfMat(1,3);  p.position.z = tfMat(2,3);
  p.orientation.x = quat.x(); p.orientation.y = quat.y(); p.orientation.z = quat.z(); p.orientation.w = quat.w();

  return checkFrankReach(IKClient,p);
}

// Check if grasp is viable for arm
bool graspSynthesis::checkGraspArm(graspPoint &graspData){
  Eigen::Affine3f tfGrasp;
  tfGrasp = tfGripper*pcl::getTransformation(0,0,0,0,-M_PI/2,-M_PI);

  Eigen::Affine3f tfPreGrasp;
  float clearance = 0;
  float rad = std::min(double(std::max(maxPtObj.x - minPtObj.x,maxPtObj.y - minPtObj.y)),0.25)/2.0;
  do{
    tfPreGrasp = tfGrasp*pcl::getTransformation(-clearance+0.0447+fingerZOffset,0,0,0,0,0);
    clearance += 0.01;
  }while((sqrt(pow(tfPreGrasp(0,3)-centroidObj.x,2) + pow(tfPreGrasp(1,3)-centroidObj.y,2)) <= rad &&
          tfPreGrasp(2,3)>0 && tfPreGrasp(2,3)<std::min(maxPtObj.z+0.02,0.35)));

  tfPreGrasp = tfGrasp*pcl::getTransformation(-clearance,0,0,0,0,0);

  geometry_msgs::Pose p;
  bool res = checkArmPose(tfGrasp.matrix(),p) * checkArmPose(tfPreGrasp.matrix(),p);
  return res;
}

// Finding pairs of grasp points from object point cloud
void graspSynthesis::calcGraspPairs(){

  graspsPossible.clear();   // Clear the vector
  selectedGrasp = -1;

  graspPoint graspTemp;
  Eigen::Vector3f vectA, vectB;
  double A,B;

  pcl::PointXYZRGB centroidGrasp;
  Eigen::Vector4f zAxis(0,0,1,0);
  
  // Checking for each pair of points (alternate points skipped to speedup the process)
  for(int i = 0; i < ptrPtCldObject->size()-1; i++){
    if(useForGrasp[i] == false) continue;
    for(int j = i+1; j < ptrPtCldObject->size(); j++){
      if(useForGrasp[j] == false) continue;

      graspTemp.p1 = ptrPtCldObject->points[i];
      graspTemp.p2 = ptrPtCldObject->points[j];    

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

      centroidGrasp.x = (graspTemp.p1.x + graspTemp.p2.x) / 2;
      centroidGrasp.y = (graspTemp.p1.y + graspTemp.p2.y) / 2;
      centroidGrasp.z = (graspTemp.p1.z + graspTemp.p2.z) / 2;
      graspTemp.distance = pcl::euclideanDistance(centroidGrasp,centroidObj);
      centroidGrasp.z = centroidObj.z; graspTemp.lineDistance = pcl::euclideanDistance(centroidGrasp,centroidObj);

      // Push this into the vector
      graspsPossible.push_back(graspTemp);
      findGripperPose(graspsPossible.size()-1);
    }
  }

  std::sort(graspsPossible.begin(),graspsPossible.end(),compareGrasp);

  // For thin objects grasp pair would not be feasible, so each point is considered as a grasp pair
  for(int i = 0; thinObject && i < ptrPtCldObject->size(); i++){
    if(useForGrasp[i] == false) continue;

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

    centroidGrasp.x = (graspTemp.p1.x + graspTemp.p2.x) / 2;
    centroidGrasp.y = (graspTemp.p1.y + graspTemp.p2.y) / 2;
    centroidGrasp.z = (graspTemp.p1.z + graspTemp.p2.z) / 2;
    graspTemp.distance = pcl::euclideanDistance(centroidGrasp,centroidObj);
    centroidGrasp.z = centroidObj.z; graspTemp.lineDistance = pcl::euclideanDistance(centroidGrasp,centroidObj);

    graspsPossible.push_back(graspTemp);
    findGripperPose(graspsPossible.size()-1);
  }
  
}

// Collision check for gripper and unexplored point cloud
void graspSynthesis::collisionCheck(){
  // Setting up the point cloud visualizer during visualize mode
  if(debugCollisionCheck){
    if(viewer == NULL){
      ptCldVis::Ptr tempViewer{new ptCldVis("PCL Visualizer")};
      viewer = tempViewer;
      keyPress.setup(viewer,1);
    }
    setupViewer(viewer, 1, vp);
    pcl::PointXYZ centre; centre.x = centroidObj.x; centre.y = centroidObj.y; centre.z = centroidObj.z;
    setCamView(viewer,{0.5,-M_PI,0},centre,vp[0]);
  }

  ptrPtCldCollided->clear();    // Reset the collision cloud
  *ptrPtCldCollCheck = *ptrPtCldUnexp + *ptrPtCldObject;
  cpBox.setInputCloud(ptrPtCldCollCheck);

  bool stop = false;
  int nOrientations;
  std::vector<int> orientations;
  // nOrientations = 8; orientations = {0,45,90,135,180,225,270,315};
  // nOrientations = 8; orientations = {90,45,135,0,180,315,225,270};
  // nOrientations = 9; orientations = {90,68,112,45,135,22,157,0,180};
  // nOrientations = 9; orientations = {-45,-68,-22,-90,0,-112,-135,-157,-180};
  nOrientations = 7; orientations = {-90,-68,-122,-45,-135,-22,-157,0,180};

  // Loop through all the possible grasps available
  for(int i = 0; (i < graspsPossible.size()) && (stop == false); i++){
    findGripperPose(i);

    // Do axis collision check
    updateGripper(i,1);
    for(int j = 3; j < 5; j++){
      cpBox.setMin(minPtCol[j].getVector4fMap());
      cpBox.setMax(maxPtCol[j].getVector4fMap());
      cpBox.setRotation(getEuler(tfGripper));
      cpBox.setTranslation(getTranslation(tfGripper));
      cpBox.filter(*ptrPtCldCollided);
      // If collision detected then exit this loop and check next grasp pair
      if(ptrPtCldCollided->size() > 0) break;
    }
    if(debugCollisionCheck){
      if(keyPress.ok){
        updateGripper(i,0);
        for(int ii = 0; ii < ptrPtCldCollided->size(); ii++){
          ptrPtCldCollided->points[ii].r = 0;
          ptrPtCldCollided->points[ii].g = 255;
          ptrPtCldCollided->points[ii].b = 0;
        }
        addRGB(viewer,ptrPtCldUnexp,"Unexp",vp[0]);
        addRGB(viewer,ptrPtCldObject,"Object",vp[0]);
        addRGB(viewer,ptrPtCldGripper,"Gripper",vp[0]);
        addRGB(viewer,ptrPtCldCollided,"Coll",vp[0]);
        addHelp(viewer,5,vp[0]);
        viewer->addText("Grasp "+std::to_string(i)+"/"+std::to_string(graspsPossible.size()),5,65,25,1,0,0,"ID",vp[0]);
        if(ptrPtCldCollided->size() > 0){
          viewer->addText("Basic check : Fail",5,95,25,1,0,0,"BasicCheck",vp[0]);
        }else{
          viewer->addText("Basic check : Pass",5,95,25,0,1,0,"BasicCheck",vp[0]);
        }
      
        keyPress.called = false;
        while(!viewer->wasStopped() && keyPress.called==false){
          viewer->spinOnce(100);
          boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
        viewer->resetStoppedFlag();
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
      }
    }
    
    // Move to next grasp if collision found
    if(ptrPtCldCollided->size() > 0) continue;

    // Do gripper collision check for each orientation
    for(int j = 0; (j < nOrientations) && (stop == false); j++){
      graspsPossible[i].addnlPitch = orientations[j]*M_PI/180;
      updateGripper(i,2);
      if(simulationMode != "SIMULATION" && (!checkGraspArm(graspsPossible[i]))) continue;
      for(int k = 5; k >= 0; k--){
        if(k == 4 || k == 3) continue;
        cpBox.setMin(minPtCol[k].getVector4fMap());
        cpBox.setMax(maxPtCol[k].getVector4fMap());
        cpBox.setRotation(getEuler(tfGripper));
        cpBox.setTranslation(getTranslation(tfGripper));
        cpBox.filter(*ptrPtCldCollided);

        // If collision detected then exit this loop and check next orientation
        if(k == 2 || k == 1){
          if(ptrPtCldCollided->size() > 5) break;
        }else{
          if(ptrPtCldCollided->size() > 0) break;
        }
      }
      if(debugCollisionCheck){
        if(keyPress.ok){
          updateGripper(i,0);
          for(int ii = 0; ii < ptrPtCldCollided->size(); ii++){
            ptrPtCldCollided->points[ii].r = 0;
            ptrPtCldCollided->points[ii].g = 255;
            ptrPtCldCollided->points[ii].b = 0;
          }
          addRGB(viewer,ptrPtCldUnexp,"Unexp",vp[0]);
          addRGB(viewer,ptrPtCldObject,"Object",vp[0]);
          addRGB(viewer,ptrPtCldGripper,"Gripper",vp[0]);
          addRGB(viewer,ptrPtCldCollided,"Coll",vp[0]);
          addHelp(viewer,5,vp[0]);
          viewer->addText("Grasp "+std::to_string(i)+"/"+std::to_string(graspsPossible.size()-1),5,65,25,1,0,0,"GID",vp[0]);
          viewer->addText("Orientation "+std::to_string(j)+"/"+std::to_string(nOrientations-1),5,95,25,1,0,0,"OID",vp[0]);
          if(ptrPtCldCollided->size() > 0){
            viewer->addText("Adv check : Fail",5,125,25,1,0,0,"AdvCheck",vp[0]);
          }else{
            viewer->addText("Adv check : Pass",5,125,25,0,1,0,"AdvCheck",vp[0]);
          }
        
          keyPress.called = false;
          while(!viewer->wasStopped() && keyPress.called==false){
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
          }
          viewer->resetStoppedFlag();
          viewer->removeAllPointClouds();
          viewer->removeAllShapes();
        }
      }
      // If this doesn't have collision, this grasp is OK. So exit the loop. No more orientation or grasp check required
      if(ptrPtCldCollided->size() == 0){
        selectedGrasp = i;
        stop = true;
      }
    }
  }
}

// Grasp synthesis callback
bool graspSynthesis::graspCallback(active_vision::graspSRV::Request &request, active_vision::graspSRV::Response &response){
  // Decoding the request
  pcl::fromROSMsg(request.object, *ptrPtCldObject);
  pcl::fromROSMsg(request.unexplored, *ptrPtCldUnexp);
  voxelGridSize = request.voxelGridSize;
  thinObject = request.thinObject;

  preprocessing();
  calcGraspPairs();
  collisionCheck();

  // Converting to the response format
  response.execTime = 0;
  response.selectedID = selectedGrasp;
  response.totalGrasps = graspsPossible.size();
  if(selectedGrasp != -1){
    response.graspQuality = graspsPossible[selectedGrasp].quality;
    response.graspWidth = graspsPossible[selectedGrasp].gripperWidth;
    response.contactA = {graspsPossible[selectedGrasp].p1.x,
                         graspsPossible[selectedGrasp].p1.y,
                         graspsPossible[selectedGrasp].p1.z};
    response.contactB = {graspsPossible[selectedGrasp].p2.x,
                         graspsPossible[selectedGrasp].p2.y,
                         graspsPossible[selectedGrasp].p2.z};
    response.pose = {graspsPossible[selectedGrasp].pose[0],graspsPossible[selectedGrasp].pose[1],
                     graspsPossible[selectedGrasp].pose[2],graspsPossible[selectedGrasp].pose[3],
                     graspsPossible[selectedGrasp].pose[4],graspsPossible[selectedGrasp].pose[5]};
    response.addnlPitch = graspsPossible[selectedGrasp].addnlPitch;
  }
  return true;
}

// Grasp visualize callback
bool graspSynthesis::graspVisCallback(active_vision::graspVisSRV::Request &request, active_vision::graspVisSRV::Response &response){
  updateGripper(request.graspID,0);
  pcl::toROSMsg(*cPtrPtCldGripper,response.graspPtCld);
  return true;
}

// Visualize the gripper model
bool graspSynthesis::viewGripperCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response){
  graspPoint graspTemp;
  graspTemp.pose[2] = 0.0447+fingerZOffset;
  graspsPossible = {graspTemp};

  // Setting up the point cloud visualizer during the first run only
  if(viewer == NULL){
    ptCldVis::Ptr tempViewer{new ptCldVis("PCL Visualizer")};
    viewer = tempViewer;
    keyPress.setup(viewer,1);
  }
  setupViewer(viewer, 1, vp);
  viewer->removeCoordinateSystem();
  viewer->addCoordinateSystem(0.1);
  viewer->setCameraPosition(0.5,0,0,0,0,0,0,0,1);
  
  updateGripper(0,0);
  addRGB(viewer,cPtrPtCldGripper,"Gripper",vp[0]);
  
  updateGripper(0,1);
  updateGripper(0,2);
  for(int i = 0; i < 6; i++){
    if(i == 3 || i == 4) continue;
    viewer->addCube(minPtCol[i].x,maxPtCol[i].x,
                    minPtCol[i].y,maxPtCol[i].y,
                    minPtCol[i].z,maxPtCol[i].z,0.0,1.0,0.0,"Cube"+std::to_string(i),vp[0]);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "Cube"+std::to_string(i));
  }
  for(int i = 3; i < 5; i++){
    viewer->addCube(minPtCol[i].x,maxPtCol[i].x,
                    minPtCol[i].y,maxPtCol[i].y,
                    minPtCol[i].z,maxPtCol[i].z,1.0,0.0,0.0,"Cube"+std::to_string(i),vp[0]);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "Cube"+std::to_string(i));
  }
  
  ROS_INFO("Showing the gripper and the bounding boxes for collision check. Close viewer to continue");
  keyPress.ok = true;
  while(!viewer->wasStopped() && keyPress.ok){
    viewer->spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds(100000));
  }
  viewer->close();

  return true;
}

void help(){
  std::cout << "******* Grasp Synthesis Module Help *******" << std::endl;
  std::cout << "Arguments : [DebugCollisionCheck]" << std::endl;
  std::cout << "DebugCollisionCheck : Optional. Set 1 to visualize the process." << std::endl;
  std::cout << "*******" << std::endl;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "GraspSynthesisServer");
  ros::NodeHandle nh;
  
  if(argc > 2){
    ROS_ERROR("Incorrect number of arguments");
    help(); return(-1);
  }

  graspSynthesis graspVar(&nh);
  graspVar.debugCollisionCheck = false;

  if(argc == 2 && std::atoi(argv[1]) == 1){
    graspVar.debugCollisionCheck = true;
    ROS_INFO("\033[33mmGrasp Synthesis : Debug collision check enabled\033[0m");
  }
  
  ros::ServiceServer graspService = nh.advertiseService("/graspSynthesis/graspCalculate", &graspSynthesis::graspCallback, &graspVar);
  ROS_INFO("\033[32mGrasp Synthesis : Grasp synthesize service ready.\033[0m");
  ros::ServiceServer graspVisService = nh.advertiseService("/graspSynthesis/graspVisualize", &graspSynthesis::graspVisCallback, &graspVar);
  ROS_INFO("\033[32mGrasp Synthesis : Grasp visualizing service ready.\033[0m");
  ros::ServiceServer viewGripperService = nh.advertiseService("/graspSynthesis/viewGripper", &graspSynthesis::viewGripperCallback, &graspVar);
  ROS_INFO("\033[32mGrasp Synthesis : View gripper service ready.\033[0m");
  ros::spin();

  return 0;
}