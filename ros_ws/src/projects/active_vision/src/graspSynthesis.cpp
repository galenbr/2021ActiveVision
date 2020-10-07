#include <boost/make_shared.hpp>
#include <tuple>
#include <cmath>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/crop_box.h>

#include <Eigen/Dense>

#include "active_vision/testingModel.h"

#define THRESHOLD 100.0

bool collisionCheck(ptCldColor::ConstPtr unexp_cloud, pcl::PointXYZRGB p1, pcl::Normal p1_normal){
	return true;/*
	pcl::CropBox<pcl::PointXYZRGB> cropBoxFilter;
	cropBoxFilter.setInputCloud(unexp_cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//Taken directly from 2018 code- TODO: Update
 	float newPt1x = float(p1.x + 0.015*p1_normal.normal_x);
 	float newPt1y = float(p1.y + 0.015*p1_normal.normal_y); 
 	float newPt1z = float(p1.z + 0.015*p1_normal.normal_z); 
	Eigen::Vector3f v_camera, v_point, axis;
  	Eigen::Vector3f v_point_norm, v_camera_norm;
  	v_camera << 1, 0, 0;
  	float angle = 0;
	v_point << p1_normal.normal_x, p1_normal.normal_y, p1_normal.normal_z;
	v_point_norm << v_point[0]/v_point.norm(),v_point[1]/v_point.norm(),v_point[2]/v_point.norm();
  	v_camera_norm << v_camera[0]/v_camera.norm(),v_camera[1]/v_camera.norm(),v_camera[2]/v_camera.norm();
  	axis = v_camera_norm.cross(v_point_norm);  // get axis with cross product between two vectors
  	axis << axis[0]/axis.norm(),axis[1]/axis.norm(),axis[2]/axis.norm();
  	if(v_point.norm() == 0){
      return false;    
    } else {
      angle = acos(v_point.dot(v_camera) / (v_point.norm() * v_camera.norm())); // "division by zero tackled"
    }

	Eigen::Quaternionf q1(cos(angle/2), axis.x()*sin(angle/2), axis.y()*sin(angle/2), axis.z()*sin(angle/2)); //axis angle to quaternion d
    q1.normalize();
    Eigen::Vector3f translation(newPt1x, newPt1y, newPt1z);
    Eigen::Vector3f euler = q1.toRotationMatrix().eulerAngles(0, 1, 2);

    Eigen::Vector4f min_pt (-0.015, -0.015, -0.015, 1);
    Eigen::Vector4f max_pt (0.015, 0.015, 0.015, 1);
    cropBoxFilter.setMin(min_pt);
    cropBoxFilter.setMax(max_pt);
    cropBoxFilter.setTranslation(translation);
    cropBoxFilter.setRotation(euler);
	cropBoxFilter.setMin(min_pt);
	cropBoxFilter.filter(*hand_cloud);
	return (hand_cloud->size() > 0);*/
}

double getAngle(pcl::PointXYZRGB p1, pcl::Normal p2, double factor){
	Eigen::Vector4f p = p1.getVector4fMap().normalized();
	Eigen::Vector4f q = p2.getNormalVector4fMap().normalized();
	return p.dot(q*factor);
}

std::tuple<double, pcl::PointXYZRGB, pcl::PointXYZRGB>  bruteForceSearch(ptCldColor::ConstPtr unexp_cloud, ptCldColor obj_cloud, pcl::PointCloud<pcl::Normal> obj_normals, double minDist, double maxDist){
	double bestQuality = 0;
	pcl::PointXYZRGB bestPointA;
	pcl::PointXYZRGB bestPointB;
	for (int i = 0; i < obj_cloud.points.size(); ++i){
		for (int j = i+1; j < obj_cloud.points.size(); ++j)
		{
			pcl::PointXYZRGB p1 = obj_cloud.points[i];
			pcl::PointXYZRGB p2 = obj_cloud.points[j];
			float distance = pcl::geometry::squaredDistance(p1, p2);
			if (distance >= minDist and distance <= maxDist){
				pcl::Normal n1 = obj_normals[i];
				pcl::Normal n2 = obj_normals[j];
				//Find both angles between the points and their normals
				double r1 = acos(getAngle(p1, n1, 1.0))*(180/M_PI);
				double r2 = acos(getAngle(p1, n1, -1.0))*(180/M_PI);
				double r3 = acos(getAngle(p2, n2, 1.0))*(180/M_PI);
				double r4 = acos(getAngle(p2, n2, -1.0))*(180/M_PI);
				//Take the best values for each
				double posAngle = std::min(r1, r3);
				double negAngle = std::min(r2, r4);
				if(posAngle <= THRESHOLD and negAngle <= THRESHOLD){
					//std::cout << p1 << " " << p2 << " "<< posAngle << " " << negAngle << std::endl;
					if(collisionCheck(unexp_cloud, p1, n1)){
						//Calculate total grasp quality
						double quality = 300 - (posAngle+negAngle);
						if(quality > bestQuality){
							bestQuality = quality;
							bestPointA = p1;
							bestPointB = p2;
							std::cout << p1 << " " << p2 << " "<< posAngle << " " << negAngle << std::endl;
						}
					}
				}
			} 
		}
	}
	//std::cout << bestQuality << bestPointA << bestPointB << std::endl;
	return std::make_tuple(bestQuality, bestPointA, bestPointB);
}

pcl::PointCloud<pcl::Normal>::Ptr calcNormals(ptCldColor::Ptr input){
	pcl::PointCloud<pcl::Normal>::Ptr ret (new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
  	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  	norm_est.setSearchMethod (tree);
  	norm_est.setKSearch (30);
  	norm_est.setInputCloud(input);
  	norm_est.compute(*ret);
  	return ret;
}

//Attempt to re-implement 'test_data_testing.cpp's grasp synthesis
void publishGraspSynthesis(environment &av, int flag){
	ptCldColor::ConstPtr unexp_cloud = av.cPtrPtCldUnexp;
	ptCldColor::Ptr obj_cloud = av.ptrPtCldObject;
	std::cout << obj_cloud->size() << std::endl;
	pcl::PointCloud<pcl::Normal>::Ptr obj_normals = calcNormals(obj_cloud);
	double quality = 300;
	pcl::PointXYZRGB graspPointA;
	pcl::PointXYZRGB graspPointB;
	std::tie(quality, graspPointA, graspPointB) = bruteForceSearch(unexp_cloud, *obj_cloud, *obj_normals, av.lowerGripperWidth, av.gripperWidth);
	//I take a hammer and FIX the code
	if(1 == flag){
		const pcl::PointXYZ *pointA = new const pcl::PointXYZ(graspPointA.x, graspPointA.y, graspPointA.z);
		const pcl::PointXYZ *pointB = new const pcl::PointXYZ(graspPointB.x, graspPointB.y, graspPointB.z);
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("PCL Viewer"));
    	viewer->initCameraParameters();
    	int vp(0);
    	viewer->createViewPort(0.0,0.0,1.0,1.0,vp);
    	viewer->addCoordinateSystem(1.0);
    	viewer->setCameraPosition(0,0,-1,0,0,1,0,-1,0);
    	viewer->addSphere(*pointA, .02, 0, 255, 0, "sphere1");
    	viewer->addSphere(*pointB, .02, 0, 255, 0, "sphere2");
    	rbgPtCldViewer(viewer,obj_cloud,"Raw Data",vp);
    	while (!viewer->wasStopped ()){
      		viewer->spinOnce(100);
      		boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
	}
	std::cout << quality << graspPointA << graspPointB << std::endl;
}

int main (int argc, char** argv){

  ros::init (argc, argv, "Grasping_Test");
  ros::NodeHandle nh;
  environment activeVision(&nh);
  sleep(1);
  //Set up the point clouds
  //testDataExtract(activeVision,0);
  //testGenUnexpPtCld(activeVision,0);
  //testUpdateUnexpPtCld(activeVision,0);
  testPtCldFuse(activeVision,0);
  //activeVision.dataExtract();
  testDataExtract(activeVision, 0);

  publishGraspSynthesis(activeVision,1);
}