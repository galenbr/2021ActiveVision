#include "mainWindow.hpp"

MainWindow::MainWindow(ros::NodeHandle& nh) :
  ui(new Ui::MainWindow), n{nh}
{
    ui->setupUi(this);

    // Setup services
    moveToPoseClient = n.serviceClient<moveit_planner::MovePose>("move_to_pose");
    moveToJointClient = n.serviceClient<moveit_planner::MoveJoint>("move_to_joint_space");
    randomizeClient = n.serviceClient<randomizer::Rand>("randomize");
    getImageClient = n.serviceClient<pcl_recorder::GetPointCloud>("get_point_cloud");

    // Setup UI connections
    connect(ui->pushButton, SIGNAL (released()), this, SLOT (randomizeClicked()));
    connect(ui->pushButton_2, SIGNAL (released()), this, SLOT (moveToStartClicked()));
    connect(ui->pushButton_3, SIGNAL (released()), this, SLOT (getReferenceClicked()));
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
  pcl_recorder::GetPointCloud msg;

  getImageClient.call(msg);

  refPointCloud = msg.response.data;

  // TODO: Send image to pcl_processor to obtain position data
}
