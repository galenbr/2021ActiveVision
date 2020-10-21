#include "active_vision/helperFunctions.h"

// Fuction to view a rgb point cloud
void rbgVis(ptCldVis::Ptr viewer, ptCldColor::ConstPtr cloud, std::string name,int vp){
  viewer->setBackgroundColor(0.5,0.5,0.5,vp);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->removePointCloud(name,vp);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud,rgb,name,vp);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,name,vp);
}

// Fuction to view a rgb point cloud with normals
void rbgNormalVis(ptCldVis::Ptr viewer, ptCldColor::ConstPtr cloud, ptCldNormal::ConstPtr normal, std::string name,int vp){
  viewer->setBackgroundColor(0.5,0.5,0.5,vp);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud,rgb,name,vp);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,name,vp);
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud,normal,5,0.01,name+"_normal",vp);
}

// Class to store data of environment and its processing
class environment{
private:
  ros::Rate r{10};                      // ROS sleep rate
  ros::Publisher pubKinectPose;         // Publisher : Kinect pose
  ros::Subscriber subKinectPtCld;       // Subscriber : Kinect pointcloud
  // NOT USED (JUST FOR REFERENCE)
  /*ros::Subscriber subKinectRGB;       // Subscriber : Kinect RGB
  ros::Subscriber subKinectDepth;       // Subscriber : Kinect DepthMap */
  ros::ServiceClient gazeboSpawnModel;  // Service : Spawn Model
  ros::ServiceClient gazeboDeleteModel; // Service : Delete Model

  pcl::PassThrough<pcl::PointXYZRGB> pass;         // Passthrough filter
  pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;      // VoxelGrid object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;      // Segmentation object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;   // Extracting object
  pcl::ConvexHull<pcl::PointXYZRGB> cvHull;        // Convex hull object
  pcl::CropHull<pcl::PointXYZRGB> cpHull;          // Crop hull object
  pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;   // Prism object
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;  // Normal Estimation

  Eigen::MatrixXf projectionMat;   // Camera projection matrix

  int readFlag[3]={};              // Flag used to read data from kinect only when needed
  float fingerZOffset{0.0584};     // Z axis offset between gripper hand and finger

  std::string path;                                     // Path the active vision package

public:
  // NOT USED (JUST FOR REFERENCE)
  /* cv_bridge::CvImageConstPtr ptrRgbLast{new cv_bridge::CvImage};    // RGB image from camera
  cv_bridge::CvImageConstPtr ptrDepthLast{new cv_bridge::CvImage};  // Depth map from camera */
  Eigen::Affine3f tfKinOptGaz;     // Transform : Kinect Optical Frame to Kinect Gazebo frame
  Eigen::Affine3f tfGazWorld;      // Transform : Kinect Gazebo Frame to Gazebo World frame
  Eigen::Affine3f tfGripper;       // Transform : For gripper based on grasp points found

  // PtCld: Last recorded viewpoint
  ptCldColor::Ptr ptrPtCldLast{new ptCldColor};      ptCldColor::ConstPtr cPtrPtCldLast{ptrPtCldLast};

  // PtCld: Environment after fusing multiple view points, extracted table, object and its normal
  ptCldColor::Ptr ptrPtCldEnv{new ptCldColor};       ptCldColor::ConstPtr cPtrPtCldEnv{ptrPtCldEnv};
  ptCldColor::Ptr ptrPtCldTable{new ptCldColor};     ptCldColor::ConstPtr cPtrPtCldTable{ptrPtCldTable};
  ptCldColor::Ptr ptrPtCldObject{new ptCldColor};    ptCldColor::ConstPtr cPtrPtCldObject{ptrPtCldObject};
  ptCldNormal::Ptr ptrObjNormal{new ptCldNormal};    ptCldNormal::ConstPtr cPtrObjNormal{ptrObjNormal};

  // PtCld: Sorting the convex hull generated
  ptCldColor::Ptr ptrPtCldHull{new ptCldColor};      ptCldColor::ConstPtr cPtrPtCldHull{ptrPtCldHull};

  // PtCld: Unexplored point cloud, point clould used for collision check (unexplored + table)
  ptCldColor::Ptr ptrPtCldUnexp{new ptCldColor};     ptCldColor::ConstPtr cPtrPtCldUnexp{ptrPtCldUnexp};
  ptCldColor::Ptr ptrPtCldCollCheck{new ptCldColor}; ptCldColor::ConstPtr cPtrPtCldCollCheck{ptrPtCldCollCheck};

  // PtCld: Gripper related
  ptCldColor::Ptr ptrPtCldGrpHnd{new ptCldColor};    ptCldColor::ConstPtr cPtrPtCldGrpHnd{ptrPtCldGrpHnd};
  ptCldColor::Ptr ptrPtCldGrpRfgr{new ptCldColor};   ptCldColor::ConstPtr cPtrPtCldGrpRfgr{ptrPtCldGrpRfgr};
  ptCldColor::Ptr ptrPtCldGrpLfgr{new ptCldColor};   ptCldColor::ConstPtr cPtrPtCldGrpLfgr{ptrPtCldGrpLfgr};
  ptCldColor::Ptr ptrPtCldGripper{new ptCldColor};   ptCldColor::ConstPtr cPtrPtCldGripper{ptrPtCldGripper};

  // PtCld: Points colliding with gripper
  ptCldColor::Ptr ptrPtCldCollided{new ptCldColor};  ptCldColor::ConstPtr cPtrPtCldCollided{ptrPtCldCollided};

  // PtCld: Temporary variable
  ptCldColor::Ptr ptrPtCldTemp{new ptCldColor};      ptCldColor::ConstPtr cPtrPtCldTemp{ptrPtCldTemp};

  // Variables used in table and object extraction
  pcl::ModelCoefficients::Ptr tableCoeff{new pcl::ModelCoefficients()};
  pcl::PointIndices::Ptr tableIndices{new pcl::PointIndices()};
  pcl::PointIndices::Ptr objectIndices{new pcl::PointIndices()};

  std::vector<pcl::Vertices> hullVertices;          // Used in convex hull during collision check
  std::vector<double> lastKinectPoseCartesian;      // Last Kinect pose where it was moved in cartesian co-ordiantes
  std::vector<double> lastKinectPoseViewsphere;     // Last Kinect pose where it was moved in viewsphere co-ordinates
  std::vector<double> minUnexp;                     // Min x,y,z of unexplored pointcloud generated
  std::vector<double> maxUnexp;                     // Max x,y,z of unexplored pointcloud generated
  std::vector<graspPoint> graspsPossible;           // List of possible grasps
  pcl::PointXYZRGB minPtObj, maxPtObj;              // Min and Max x,y,z co-ordinates of the object

  std::vector<std::vector<std::string>> objectDict; // List of objects which can be spawned
  double voxelGridSize{0.01};                       // Voxel Grid size for environment
  double voxelGridSizeUnexp{0.02};                  // Voxel Grid size for unexplored point cloud
  std::vector<double> tableCentre{1.5,0,1};         // Co-ordinates of table centre
  int scale{3};                                     // Scale value for unexplored point cloud generation
  double maxGripperWidth{0.075};                    // Gripper max width (Actual is 8 cm)
  double minGraspQuality{150};                      // Min grasp quality threshold
  int selectedGrasp{-1};                            // Index of the selected grasp

  environment(ros::NodeHandle *nh){

    pubKinectPose = nh->advertise<gazebo_msgs::ModelState> ("/gazebo/set_model_state", 1);
    subKinectPtCld = nh->subscribe ("/camera/depth/points", 1, &environment::cbPtCld, this);
    // NOT USED (JUST FOR REFERENCE)
    /*subKinectRGB = nh->subscribe ("/camera/color/image_raw", 1, &environment::cbImgRgb, this);
    subKinectDepth = nh->subscribe ("/camera/depth/image_raw", 1, &environment::cbImgDepth, this);*/
    gazeboSpawnModel = nh->serviceClient< gazebo_msgs::SpawnModel> ("/gazebo/spawn_sdf_model");
    gazeboDeleteModel = nh->serviceClient< gazebo_msgs::DeleteModel> ("/gazebo/delete_model");

    // Transform : Kinect Optical Frame to Kinect Gazebo frame
    tfKinOptGaz = pcl::getTransformation(0,0,0,-M_PI/2,0,-M_PI/2);

    // Camera projection matrix
    projectionMat.resize(3,4);
    projectionMat << 554.254691191187, 0.0, 320.5, -0.0,
                     0.0, 554.254691191187, 240.5, 0.0,
                     0.0, 0.0, 1.0, 0.0;

    // Path to the active_vision package folder
    path = ros::package::getPath("active_vision");

    // Dictionary of objects to be spawned
    objectDict = {{"drillAV","Cordless Drill"},
                  {"squarePrismAV","Square Prism"},
                  {"rectPrismAV","Rectangular Prism"},
                  {"bowlAV","Bowl"},
                  {"bowl2AV","Bowl2"},
                  {"cupAV","Cup"}};
  }

  // 1A: Callback function to point cloud subscriber
  void cbPtCld (const ptCldColor::ConstPtr& msg){
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

  // 2: Spawning objects in gazebo on the table centre for a given RPY
  void spawnObject(int objectID, float R, float P, float Y){
    gazebo_msgs::SpawnModel spawnObj;
    geometry_msgs::Pose pose;

    //Create Matrix3x3 from Euler Angles
    tf::Matrix3x3 m_rot;
    m_rot.setEulerYPR(Y, P, R);

    // Convert into quaternion
    tf::Quaternion quat;
    m_rot.getRotation(quat);

    pose.position.x = tableCentre[0];
    pose.position.y = tableCentre[1];
    pose.position.z = tableCentre[2];
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();

    spawnObj.request.model_name = objectDict[objectID][1];

    std::ifstream ifs(path+"/models/"+objectDict[objectID][0]+"/model.sdf");
    std::string sdfFile( (std::istreambuf_iterator<char>(ifs)),
                         (std::istreambuf_iterator<char>()));
    spawnObj.request.model_xml = sdfFile;

    spawnObj.request.reference_frame = "world";
    spawnObj.request.initial_pose = pose;

    gazeboSpawnModel.call(spawnObj);

    boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
  }

  // 3: Deleting objects in gazebo
  void deleteObject(int objectID){
    gazebo_msgs::DeleteModel deleteObj;
    deleteObj.request.model_name = objectDict[objectID][1];

    gazeboDeleteModel.call(deleteObj);
  }

  // 4: Load Gripper Hand and Finger file
  void loadGripper(){
    std::string pathToHand = path+"/models/gripperAV/hand.ply";
    std::string pathToFinger = path+"/models/gripperAV/finger.ply";
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

    std::cout << "Ignore the PLY reader error on 'face' and 'rgb'." << std::endl;
  }

  // 5: Update gripper
  // 0 -> Hand + Left finger + Right finger
  // 1 -> Hand only
  // 2 -> Left Finger only
  // 3 -> Right FInger only
  void updateGripper(int index, int choice){
    updateGripper(index, choice, &graspsPossible);
  }

  void updateGripper(int index ,int choice, std::vector<graspPoint> *graspsP){
    if (choice == 0) {
      // Adding the gripper hand
      *ptrPtCldGripper=*ptrPtCldGrpHnd;

      // Translating the left finger and adding
      pcl::transformPointCloud(*ptrPtCldGrpLfgr, *ptrPtCldTemp,
                              pcl::getTransformation(0,(*graspsP)[index].gripperWidth/2,fingerZOffset,0,0,0));
      *ptrPtCldGripper += *ptrPtCldTemp;

      // Translating the right finger and adding
      pcl::transformPointCloud(*ptrPtCldGrpRfgr, *ptrPtCldTemp,
                              pcl::getTransformation(0,-(*graspsP)[index].gripperWidth/2,fingerZOffset,0,0,0));
      *ptrPtCldGripper += *ptrPtCldTemp;
    } else if (choice == 1) {
      *ptrPtCldGripper = *ptrPtCldGrpHnd;
    } else if (choice == 2){
      // Translating the left finger and setting
      pcl::transformPointCloud(*ptrPtCldGrpLfgr, *ptrPtCldTemp,
                               pcl::getTransformation(0,(*graspsP)[index].gripperWidth/2,fingerZOffset,0,0,0));
      *ptrPtCldGripper = *ptrPtCldTemp;
    } else if (choice == 3){
      // Translating the right finger and setting
      pcl::transformPointCloud(*ptrPtCldGrpRfgr, *ptrPtCldTemp,
                               pcl::getTransformation(0,-(*graspsP)[index].gripperWidth/2,fingerZOffset,0,0,0));
      *ptrPtCldGripper = *ptrPtCldTemp;
    }

    tfGripper = pcl::getTransformation((*graspsP)[index].pose[0],(*graspsP)[index].pose[1],
                                       (*graspsP)[index].pose[2],(*graspsP)[index].pose[3],
                                       (*graspsP)[index].pose[4],(*graspsP)[index].pose[5])*
                pcl::getTransformation(0,0,0,0,(*graspsP)[index].addnlPitch,0)*
                pcl::getTransformation(0,0,-0.0447-fingerZOffset,0,0,0);
    pcl::transformPointCloud(*ptrPtCldGripper, *ptrPtCldGripper, tfGripper);

    ptrPtCldTemp->clear();
  }

  // 6A: Function to move the kinect. Args: Array of X,Y,Z,Roll,Pitch,Yaw
  void moveKinectCartesian(std::vector<double> pose){
    gazebo_msgs::ModelState ModelState = kinectCartesianModel(pose);
    
    // Publishing it to gazebo
    pubKinectPose.publish(ModelState);
    ros::spinOnce();
    boost::this_thread::sleep(boost::posix_time::milliseconds(250));

    // Storing the kinect pose
    lastKinectPoseCartesian = pose;
  }

  // 6B: Funtion to move the Kinect in a viewsphere which has the table cente as its centre
  // R (Radius)
  // Theta (Polar Angle) -> 0 to 2*PI
  // Phi (Azhimuthal angle) -> 0 to PI/2
  void moveKinectViewsphere(std::vector<double> pose){
    // Converting it to the required gazebo format
    gazebo_msgs::ModelState ModelState = kinectViewSphereModel(pose, tableCentre);

    // Publishing it to gazebo
    pubKinectPose.publish(ModelState);
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
  void readKinect(){
    readFlag[0] = 1; // readFlag[1] = 1; readFlag[2] = 1;
    while (readFlag[0]==1 || readFlag[1]==1 || readFlag[2]==1) {
      ros::spinOnce();
      r.sleep();
    }
  }

  // 8: Function to Fuse last data with existing data
  void fuseLastData(void){
    fuseLastData(ptrPtCldEnv);
  }

  void fuseLastData(ptCldColor::Ptr retEnv){
    tfGazWorld = transformGazWorld(lastKinectPoseCartesian);
    fuseData(tfGazWorld, tfKinOptGaz, ptrPtCldLast, ptrPtCldEnv, retEnv, voxelGridSize);
  }

  // 9: Extracting the major plane (Table) and object
  void dataExtract(void){
    dataExtract(cPtrPtCldEnv, ptrPtCldObject);
  }

  void dataExtract(ptCldColor::ConstPtr cEnv, ptCldColor::Ptr target){
    extractObj(cEnv, target, ptrPtCldTable, ptrPtCldHull, &minPtObj, &maxPtObj, voxelGridSize);
  }

  // 10: Generating unexplored point cloud
  void genUnexploredPtCld(){
    if (ptrPtCldUnexp->width != 0){
      std::cerr << "Unexplored point cloud already created. Not creating new one." << std::endl;
      return;
    }

    // Setting the min and max limits based on the object dimension and scale.
    // Note: Z scale is only used on +z axis
    minUnexp.push_back(minPtObj.x-(scale-1)*(maxPtObj.x-minPtObj.x)/2);
    minUnexp.push_back(minPtObj.y-(scale-1)*(maxPtObj.y-minPtObj.y)/2);
    minUnexp.push_back(minPtObj.z);
    maxUnexp.push_back(maxPtObj.x+(scale-1)*(maxPtObj.x-minPtObj.x)/2);
    maxUnexp.push_back(maxPtObj.y+(scale-1)*(maxPtObj.y-minPtObj.y)/2);
    maxUnexp.push_back(maxPtObj.z+(scale-1)*(maxPtObj.z-minPtObj.z)/2);

    // Considering a point for every 1 cm and then downsampling it
    float nPts = (maxUnexp[0]-minUnexp[0])*(maxUnexp[1]-minUnexp[1])*(maxUnexp[2]-minUnexp[2])*1000000;
    // std::cout << minUnexp[0] << " " << minUnexp[1] << " " << minUnexp[2] << std::endl;
    // std::cout << maxUnexp[0] << " " << maxUnexp[1] << " " << maxUnexp[2] << std::endl;

    pcl::common::CloudGenerator<pcl::PointXYZRGB, pcl::common::UniformGenerator<float>> generator;
    std::uint32_t seed = static_cast<std::uint32_t> (time (nullptr));
    pcl::common::UniformGenerator<float>::Parameters x_params(minUnexp[0], maxUnexp[0], seed++);
    generator.setParametersForX(x_params);
    pcl::common::UniformGenerator<float>::Parameters y_params(minUnexp[1], maxUnexp[1], seed++);;
    generator.setParametersForY(y_params);
    pcl::common::UniformGenerator<float>::Parameters z_params(minUnexp[2], maxUnexp[2], seed++);;
    generator.setParametersForZ(z_params);
    int result = generator.fill(int(nPts), 1, *ptrPtCldUnexp);
    if (result != 0) {
      std::cerr << "Error creating unexplored point cloud." << std::endl;
      return;
    }

    voxelGrid.setInputCloud(cPtrPtCldUnexp);
    voxelGrid.setLeafSize(voxelGridSizeUnexp, voxelGridSizeUnexp, voxelGridSizeUnexp);
    voxelGrid.filter(*ptrPtCldUnexp);
  }

  // 11: Updating the unexplored point cloud
  void updateUnexploredPtCld(){
    // Transforming the point cloud to Kinect frame from world frame
    updateunexploredPtCld(tfGazWorld, tfKinOptGaz, projectionMat, ptrPtCldUnexp,  ptrPtCldLast, cPtrPtCldTable, minPtObj, maxPtObj, ptrPtCldUnexp, ptrPtCldCollCheck, voxelGridSizeUnexp);
  }

  // 12: Reset selected grasp and synthesize all possible grasps for the current ptCldObject
  void graspSynthesis(void){
    graspSynthesis(ptrPtCldObject);
  }

  // 12b: Reset selected grasp and synthesize all possible grasps for a provided object
  void graspSynthesis(ptCldColor::Ptr object){
    selectedGrasp = -1;
    graspsynthesis(graspsPossible, object, tableCentre, minGraspQuality, maxGripperWidth, voxelGridSize);
  }

  // 13: Given a grasp point pair find the gripper orientation
  void findGripperPose(int index){
    graspsPossible[index].pose = genGripperPose(graspsPossible, index);
  }

  // 14: Collision check for gripper and unexplored point cloud
  void collisionCheck(){

    cvHull.setInputCloud(cPtrPtCldGripper);
    cvHull.setDimension(3);

    cpHull.setInputCloud(ptrPtCldCollCheck);
    cpHull.setHullCloud(ptrPtCldHull);
    cpHull.setDim(3);
    cpHull.setCropOutside(true);

    bool stop = false;
    int nOrientations = 8;
    // Loop through all the possible grasps available
    for(int i = 0; (i < graspsPossible.size()) && (stop == false); i++){
      findGripperPose(i);
      // Check for each orientation
      for(int j = 0; (j < nOrientations) && (stop == false); j++){
        graspsPossible[i].addnlPitch = j*(2*M_PI)/nOrientations;
        // Get concave hull of the gripper fingers and hand in sequence and check
        for(int k = 3; k >= 1; k--){
          ptrPtCldCollided->clear();    // Reset the collision cloud
          updateGripper(i,k);
          cvHull.reconstruct(*ptrPtCldHull,hullVertices);
          cpHull.setHullIndices(hullVertices);
          cpHull.filter(*ptrPtCldCollided);
          // std::cout << i << " Orientation " << j << " " << "Object " << k << " Ptcld size " << ptrPtCldCollided->size() << std::endl;
          // If collision detected then exit this loop and check next orientation
          if(ptrPtCldCollided->size() > 0){
            break;
          }
        }
        // If this doesn't have collision, this grasp is OK. So exit the loop. No more orientation or grasp check required
        if(ptrPtCldCollided->size() == 0){
          selectedGrasp = i;
          // std::cout << "Setting selected grasp to " << selectedGrasp << std::endl;
          stop = true;
        }
      }
    }
  }
};

// 1: A test function spawn and delete objects in gazebo
void testSpawnDeleteObj(environment &av){
  std::cout << "*** In object spawn and delete testing function ***" << std::endl;
  int flag = 0;
  for (int i = 0; i < av.objectDict.size(); i++) {
    av.spawnObject(i,0,0,0);
    std::cout << "Object " << i+1 << " spawned. Enter any key to continue. "; std::cin >> flag;
    av.deleteObject(i);
    std::cout << "Object " << i+1 << " deleted." << std::endl;
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
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
      std::cout << "Theta (Polar Angle) (0->2*PI) : ";      std::cin >> pose[1];
      std::cout << "Phi (Azhimuthal Angle) (0->PI/2): ";    std::cin >> pose[2];

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
void testKinectRead(environment &av, int flag){
  std::cout << "*** In kinect data read testing function ***" << std::endl;
  av.spawnObject(0,0,0,0);

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
  av.deleteObject(0);
  std::cout << "*** End ***" << std::endl;
}

// 4: A test function to check fusing of data
void testPtCldFuse(environment &av, int flag){
  std::cout << "*** In point cloud data fusion testing function ***" << std::endl;
  av.spawnObject(0,0,0,0);

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
  av.deleteObject(0);
  std::cout << "*** End ***" << std::endl;
}

// 5: A test function to extract table and object data
void testDataExtract(environment &av, int flag){
  std::cout << "*** In table and object extraction testing function ***" << std::endl;
  av.spawnObject(0,0,0,0);

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
  av.deleteObject(0);
  std::cout << "*** End ***" << std::endl;
}

// 6: A test function to generate unexplored point cloud
void testGenUnexpPtCld(environment &av, int flag){
  std::cout << "*** In unexplored point cloud generation testing function ***" << std::endl;
  av.spawnObject(0,0,0,0);

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
  av.deleteObject(0);
  std::cout << "*** End ***" << std::endl;
}

// 7: A test function to update unexplored point cloud
void testUpdateUnexpPtCld(environment &av, int flag){
  std::cout << "*** In unexplored point cloud update testing function ***" << std::endl;
  av.spawnObject(0,0,0,0);

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
      // rbgVis(viewer,av.cPtrPtCldEnv,"Env "+std::to_string(i),vp[i]);
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
  av.deleteObject(0);
  std::cout << "*** End ***" << std::endl;
}

// 8: A test function to load and update gripper
void testGripper(environment &av, int flag, float width){
  std::cout << "*** In gripper testing function ***" << std::endl;

  graspPoint graspTemp; // Creating a dummy grasp
  av.graspsPossible.push_back(graspTemp);

  av.loadGripper();
  av.updateGripper(0,0);

  if (flag == 1) {
    // Setting up the point cloud visualizer
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
    viewer->initCameraParameters();
    int vp = {};
    viewer->createViewPort(0.0,0.0,1.0,1.0,vp);
    viewer->addCoordinateSystem(0.1);
    viewer->setCameraPosition(0.5,0,0,-1,0,0,0,0,1);

    // Adding the point cloud
    rbgVis(viewer,av.cPtrPtCldGripper,"Gripper",vp);

    std::cout << "Showing the gripper. Close viewer to continue" << std::endl;

    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
  std::cout << "*** End ***" << std::endl;
}

// 9: Grasp synthesis test function
void testGraspsynthesis(environment &av, int flag){
  std::cout << "*** In grasp synthesis testing function ***" << std::endl;
  av.spawnObject(0,0,0,0);

  // 4 kinect position
  std::vector<std::vector<double>> kinectPoses = {{1.4,-M_PI,M_PI/3},
                                                  {1.4,-M_PI/2,M_PI/3},
                                                  {1.4,0,M_PI/3},
                                                  {1.4,M_PI/2,M_PI/3}};

  for (int i = 0; i < 4; i++) {
    std::cout << "t1" << std::endl;
    av.moveKinectViewsphere(kinectPoses[i]);
    std::cout << "t2" << std::endl;
    av.readKinect();
    std::cout << "t3" << std::endl;
    av.fuseLastData();
    std::cout << "t4" << std::endl;
    av.dataExtract();
  }

  std::cout << "Min grasp quality threshold is " << av.minGraspQuality << std::endl;
  av.graspSynthesis();

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
  av.deleteObject(0);
  std::cout << "*** End ***" << std::endl;
}

// 10A: A test function to check the collision check algorithm with dummy data
void testCollisionDummy(environment &av, bool result, int flag){
  std::cout << "*** In dummy collision testing function ***" << std::endl;

  graspPoint graspTemp; // Creating a dummy grasp
  av.graspsPossible.push_back(graspTemp);

  av.loadGripper();

  // Creating a dummy unexplored point cloud
  av.ptrPtCldCollCheck->clear();
  pcl::common::CloudGenerator<pcl::PointXYZRGB, pcl::common::UniformGenerator<float>> generator;
  std::uint32_t seed = static_cast<std::uint32_t> (time (nullptr));
  pcl::common::UniformGenerator<float>::Parameters x_params(0, 0.03, seed++);
  generator.setParametersForX(x_params);
  pcl::common::UniformGenerator<float>::Parameters y_params(-0.15, 0.15, seed++);
  generator.setParametersForY(y_params);
  if(result == true){
    pcl::common::UniformGenerator<float>::Parameters z_params(-0.15, -0.01, seed++);
    generator.setParametersForZ(z_params);
  }else{
    pcl::common::UniformGenerator<float>::Parameters z_params(-0.15, 0.15, seed++);
    generator.setParametersForZ(z_params);
  }

  int dummy = generator.fill(5000, 1, *av.ptrPtCldCollCheck);
  // Setting color to blue
  for (int i = 0; i < av.ptrPtCldCollCheck->size(); i++) {
    av.ptrPtCldCollCheck->points[i].b = 200;
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

    rbgVis(viewer,av.ptrPtCldCollCheck,"Collision Check",vp);
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
void testCollision(environment &av, int objID, int flag){
  std::cout << "*** In collision testing function ***" << std::endl;
  std::chrono::high_resolution_clock::time_point start[4], end[4];
  double elapsed[4];

  av.spawnObject(objID,0,0,0);
  av.loadGripper();

  // 4 kinect poses
  std::vector<std::vector<double>> kinectPoses = {{1.4,-M_PI,M_PI/3},
                                                  {1.4,-M_PI/2,M_PI/3},
                                                  {1.4,0,M_PI/3},
                                                  {1.4,M_PI/2,M_PI/3}};

  start[0] = std::chrono::high_resolution_clock::now();

  start[1] = std::chrono::high_resolution_clock::now();
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
  end[1] = std::chrono::high_resolution_clock::now();

  std::cout << "Number of points in object point cloud : " << av.ptrPtCldObject->points.size() << std::endl;
  // std::cout << "Number of points in unexplored cloud : " << av.ptrPtCldUnexp->points.size() << std::endl;
  std::cout << "Number of points in collision check cloud : " << av.ptrPtCldCollCheck->points.size() << std::endl;
  start[2] = std::chrono::high_resolution_clock::now();
  av.graspSynthesis();
  end[2] = std::chrono::high_resolution_clock::now();
  std::cout << "Number of grasps found : " << av.graspsPossible.size() << std::endl;

  start[3] = std::chrono::high_resolution_clock::now();
  av.collisionCheck();
  end[3] = std::chrono::high_resolution_clock::now();
  std::cout << "Selected Grasp ID : " << av.selectedGrasp << std::endl;

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
  std::cout << "Grasp Synthesis = " << elapsed[2]/1000 << std::endl;
  std::cout << "Collision Check = " << elapsed[3]/1000 << std::endl << std::endl;

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

    for (int i = 0; i < av.ptrPtCldObject->size(); i++) {
      av.ptrPtCldObject->points[i].r = 0;
      av.ptrPtCldObject->points[i].b = 200;
      av.ptrPtCldObject->points[i].g = 0;
    }
    rbgVis(viewer,av.ptrPtCldObject,"Object",vp[1]);

    for (int i = 0; i < av.ptrPtCldCollCheck->size(); i++) {
      av.ptrPtCldCollCheck->points[i].r = 200;
      av.ptrPtCldCollCheck->points[i].b = 0;
      av.ptrPtCldCollCheck->points[i].g = 0;
    }
    rbgVis(viewer,av.ptrPtCldCollCheck,"Collision",vp[1]);

    if(av.selectedGrasp == -1){
      std::cout << "No grasp orientation for the grasp points found." << std::endl;
      std::cout << "Showing the object (blue), collision check points (red). Close viewer to continue" << std::endl;
    }else{
      av.updateGripper(av.selectedGrasp,0);    // Only for visulization purpose
      rbgVis(viewer,av.ptrPtCldGripper,"Gripper",vp[1]);
      std::cout << "Showing the object (blue), collision check points (red), selected gripper position (black). Close viewer to continue" << std::endl;
    }

    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }

  av.deleteObject(objID);
  std::cout << "*** End ***" << std::endl;
}

int main (int argc, char** argv){

  // Initialize ROS
  ros::init (argc, argv, "Testing_Model");
  ros::NodeHandle nh;

  environment activeVision(&nh);
  // Delay to ensure all publishers and subscribers are connected
  boost::this_thread::sleep(boost::posix_time::milliseconds(500));

  int choice, objID;
  std::cout << "Available choices for test functions : " << std::endl;
  std::cout << "1  : Spawn and delete 6 objects on the table." << std::endl;
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
  std::cout << "Enter your choice : "; cin >> choice;
  switch(choice){
    case 1:
      testSpawnDeleteObj(activeVision);
      break;
    case 2:
      testGripper(activeVision,1,0.05);
      break;
    case 3:
      testKinectMovement(activeVision);
      break;
    case 4:
      testMoveKinectInViewsphere(activeVision);
      break;
    case 5:
      testKinectRead(activeVision,1);
      break;
    case 6:
      testPtCldFuse(activeVision,1);
      break;
    case 7:
      testDataExtract(activeVision,1);
      break;
    case 8:
      testGenUnexpPtCld(activeVision,1);
      break;
    case 9:
      testUpdateUnexpPtCld(activeVision,1);
      break;
    case 10:
      testGraspsynthesis(activeVision,1);
      break;
    case 11:
      std::cout << "Objects available :" << std::endl;
      std::cout << "1: Drill" << std::endl;
      std::cout << "2: Square Prism" << std::endl;
      std::cout << "3: Rectangular Prism" << std::endl;
      std::cout << "4: Bowl" << std::endl;
      std::cout << "5: Big Bowl" << std::endl;
      std::cout << "6: Cup" << std::endl;
      std::cout << "Enter your choice : "; std::cin>>objID;
      std::cout << "Enter your voxel grid size (0.005 to 0.01) : "; std::cin >> activeVision.voxelGridSize;
      activeVision.voxelGridSize = std::max(activeVision.voxelGridSize,0.005);
      activeVision.voxelGridSize = std::min(activeVision.voxelGridSize,0.01);
      testCollision(activeVision,objID-1,1);
      break;
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
