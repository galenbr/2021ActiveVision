#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"

#include "randomizer/Rand.h"
#include "geometry_msgs/Point.h"
#include "moveit_planner/MovePose.h"
#include "moveit_planner/MoveJoint.h"
#include "pcl_recorder/GetPointCloud.h"

#include <QMainWindow>
#include "ui_mainWindow.h"

namespace Ui {
  class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(ros::NodeHandle &nh);
  virtual ~MainWindow();

private slots:
  // Callbacks
  void moveToStartClicked();
  void randomizeClicked();
  void getReferenceClicked();

private:
  ros::NodeHandle& n;
  ros::ServiceClient moveToPoseClient;
  ros::ServiceClient moveToJointClient;
  ros::ServiceClient randomizeClient;
  ros::ServiceClient getImageClient;
  
  Ui::MainWindow *ui;

  // Data
  sensor_msgs::PointCloud2 refPointCloud;
};

#endif // MAINWINDOW_H
