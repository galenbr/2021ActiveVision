#include "mainWindow.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <iostream>

MainWindow::MainWindow(ros::NodeHandle& nh) :
  ui(new Ui::MainWindow), n{nh}
{
    ui->setupUi(this);

    // Setup services
    moveToPoseClient = n.serviceClient<moveit_planner::MovePose>("move_to_pose");
    moveToJointClient = n.serviceClient<moveit_planner::MoveJoint>("move_to_joint_space");
    moveAwayClient = n.serviceClient<moveit_planner::MoveAway>("move_away_point");
    moveCartClient = n.serviceClient<moveit_planner::MoveCart>("cartesian_move");
    randomizeClient = n.serviceClient<randomizer::Rand>("randomize");
    getImageClient = n.serviceClient<pcl_recorder::GetPointCloud>("get_point_cloud");
    getPoseClient = n.serviceClient<pose_estimator::PoseEstimation>("pose_estimator/gazebo_pose");
    gripperClient = n.serviceClient<franka_gripper_gazebo::GripMsg>("gazebo_franka_grip");

    // Setup UI connections
    connect(ui->pushButton, SIGNAL (released()), this, SLOT (randomizeClicked()));
    connect(ui->pushButton_2, SIGNAL (released()), this, SLOT (moveToStartClicked()));
    connect(ui->pushButton_3, SIGNAL (released()), this, SLOT (getReferenceClicked()));
    connect(ui->pushButton_4, SIGNAL (released()), this, SLOT (graspClicked()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::moveToStartClicked() {
  moveit_planner::MoveJoint msg;
  msg.request.val.push_back(0);		// 1
  msg.request.val.push_back(0);		// 2
  msg.request.val.push_back(0);		// 3
  msg.request.val.push_back(-3.1415/2);	// 4
  msg.request.val.push_back(0);		// 5
  msg.request.val.push_back(3.1415/2);	// 6
  msg.request.val.push_back(3.1415/4);	// 7
  msg.request.execute = true;
  if(!moveToJointClient.call(msg)) {
    ROS_ERROR("Could not move to start!");
  }
  else
    ROS_INFO("Moved to start!");
}

void MainWindow::randomizeClicked() {
  randomizer::Rand randMsg;
  randMsg.request.toStart = false;

  randomizeClient.call(randMsg);
}

void MainWindow::getReferenceClicked() {
  // Obtain reference pose
  getPoseClient.call(ref_pose_est);
}

void MainWindow::graspClicked() {
  // For now, just try to grasp the first object in the ref_pose_est
  // Hardcoded grasp pose
  Eigen::MatrixXd graspPosition(4, 4);
  graspPosition = Eigen::MatrixXd::Zero(4, 4);
  // Rotation
  graspPosition(0,1) = -1;	// x-axis
  graspPosition(0,0) = 0;	// x-axis
  graspPosition(1,0) = -1;	// y-axis
  graspPosition(1,1) = 0;	// y-axis
  graspPosition(2,2) = -1;	// z-axis
  // Position
  graspPosition(0,3) = -0.0587;
  graspPosition(1,3) = 0;
  graspPosition(2,3) = 0.06569;
  graspPosition(3,3) = 1;

  // Convert from quat + position to transform matrix
  Eigen::MatrixXd objectPose(4, 4);
  objectPose = Eigen::MatrixXd::Zero(4, 4);
  Eigen::Quaterniond curObjectQuat;
  curObjectQuat.x() = ref_pose_est.response.detected_object_poses[0].orientation.x;
  curObjectQuat.y() = ref_pose_est.response.detected_object_poses[0].orientation.y;
  curObjectQuat.z() = ref_pose_est.response.detected_object_poses[0].orientation.z;
  curObjectQuat.w() = ref_pose_est.response.detected_object_poses[0].orientation.w;
  objectPose.topLeftCorner(3, 3) = curObjectQuat.normalized().toRotationMatrix();
  objectPose(0,3) = ref_pose_est.response.detected_object_poses[0].position.x;
  objectPose(1,3) = ref_pose_est.response.detected_object_poses[0].position.y;
  objectPose(2,3) = ref_pose_est.response.detected_object_poses[0].position.z;
  objectPose(3,3) = 1;

  // Multiply to get final transform matrix
  Eigen::MatrixXd finalGraspTransform = objectPose * graspPosition;

  // Convert to quat
  Eigen::Matrix3d rot = finalGraspTransform.topLeftCorner(3, 3);
  Eigen::Quaterniond finalGraspQuat{rot};
  
  // Send final transform to server
  double graspDist = 0.3;
  geometry_msgs::Pose graspPose;
  graspPose.orientation.x = finalGraspQuat.x();
  graspPose.orientation.y = finalGraspQuat.y();
  graspPose.orientation.z = finalGraspQuat.z();
  graspPose.orientation.w = finalGraspQuat.w();
  graspPose.position.x = finalGraspTransform(0,3);
  graspPose.position.y = finalGraspTransform(1,3);
  graspPose.position.z = finalGraspTransform(2,3);

  // Send request
  moveit_planner::MoveAway msg;
  msg.request.pose = graspPose;
  msg.request.distance = graspDist;
  msg.request.execute = true;
  moveAwayClient.call(msg);

  // Open gripper
  std::cout<<"Sending gripper open request"<<std::endl;
  franka_gripper_gazebo::GripMsg gripMsg;
  gripMsg.request.force = 100;
  gripperClient.call(gripMsg);
  std::cout<<"DONE"<<std::endl;

  // Move closer
  std::cout<<"SENDING CARTESIAN MSG"<<std::endl;
  moveit_planner::MoveCart approachMsg;
  approachMsg.request.val.push_back(graspPose);
  approachMsg.request.execute = true;
  moveCartClient.call(approachMsg);
  std::cout<<"DONE"<<std::endl;

  // Grip
  gripMsg.request.force = -100;
  gripperClient.call(gripMsg);

  // Move away
  moveit_planner::MoveCart retreatMsg;
  retreatMsg.request.val.push_back(msg.response.awayPose);
  retreatMsg.request.execute = true;
  moveCartClient.call(retreatMsg);
}
