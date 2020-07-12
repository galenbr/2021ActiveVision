/*************************************
 Mihir
**************************************/

#include <iostream>
#include <Eigen/Dense>
#include "ros/ros.h"
#include <math.h>


#define PI 3.142
#define rot PI/2 // Angle of rotation for one step
#define N 0
#define NW 1
#define W 2
#define SW 3
#define S 4
#define SE 5
#define E 6
#define NE 7

Eigen::Vector3d axis_array[8];


void rotate_vector_by_quaternion(Eigen::Vector3d& v, Eigen::Quaterniond& q, Eigen::Vector3d& vprime)
{
    // Extract the vector part of the quaternion
    Eigen::Vector3d u(q.vec());

    // Extract the scalar part of the quaternion
    float s = q.w();

    // Do the math
    vprime = 2.0f * v.dot(u) * u
          + (s*s - u.dot(u)) * v
          + 2.0f * s * u.cross(v);
}

void rotate(Eigen::Vector3d& v, Eigen::Vector3d& vprime, int dir){
  Eigen::Vector3d axis;
  axis = axis_array[dir];  //axis of rotation

  //axis << 2.0, 0.0, -2.0;
  Eigen::Quaterniond q(cos(rot/2), axis.x()*sin(rot/2), axis.y()*sin(rot/2), axis.z()*sin(rot/2)); //axis angle to quaternion d
  q.normalize(); //Need normalization for correct behaviour
  //std::cout<<q.vec()<<"\n";

  rotate_vector_by_quaternion(v, q, vprime);
  //Update all axes
  for (int i = 0; i<8; i++)
  {
    rotate_vector_by_quaternion(axis_array[i], q, axis_array[i]);
  }
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "move_in_sphere");
  int dir;
  //std::cout<<argv[3]<<"--\n";
  /*
  if(argc != 5){
    std::cout<<"Enter a 3D point and direction (between 0-7)";
    return 0;
  }
  */
  axis_array[0] << 0, 1, 0; 
  axis_array[1] << 0.0, 2.0, -2.0; 
  axis_array[2] << 0, 0, -1;
  axis_array[3] << 0.0, -2.0, -2.0;
  axis_array[4] << 0.0, -1.0, 0.0;
  axis_array[5] << 0.0, -2.0, 2.0;
  axis_array[6] << 0, 0, 1;
  axis_array[7] << 0.0, 2.0, 2.0;

  Eigen::Vector3d v, vprime, v_start, axis;
  //v << atof(argv[1]), atof(argv[2]), atof(argv[3]);

  v << -1, 0, 0;
  //v << 0.3595, 0.0, 643499;
  v_start << -v.norm(), 0, 0;
  axis = v_start.cross(v);
  float angle = acos( v.dot(v_start) / (v.norm() * v_start.norm()) );

  //float x_start = sqrt(v[0]*v[0]+v[1]*v[1]);
  //v_start << x_start, 0, v[2];
  //Apply rotation
  //float angle = atan2(v[1], -v[0]); // if initial front direction is not along X-axis

  axis << 0, 0, 1;  //Z-axis of rotation
  Eigen::Quaterniond q(cos(angle/2), axis.x()*sin(angle/2), axis.y()*sin(angle/2), axis.z()*sin(angle/2)); //axis angle to quaternion d
  q.normalize(); //Need normalization for correct behaviour
  for (int i = 0; i<8; i++)
  {
    rotate_vector_by_quaternion(axis_array[i], q, axis_array[i]);
    //std::cout << axis_array[i]<<"\n -------------\n";
  }

  //Eigen::Quaterniond q(0,0,0.707,0.707);
  

  //base_to_object(v, obj_pos);
  //object_to_base(v, obj_pos);
  //v << -1.0, 0.0, 0.0; //point in the front of the object sphere

  //Eigen::AngleAxisd(PI, 0.0, 0.0, 1.0); //check on this later.

  while(true)
  {
    std::cout<<"Enter a direction (between 0-7): ";
    std::cin>>dir;
    if (abs(dir)>7)
      {
        std::cout<<"Error"<<std::endl;
        continue;
      }
    rotate(v, v, dir);
    std::cout<<v<<"\n";  
  }

  return 0;
}
