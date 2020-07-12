#include <move_camera.h>



using namespace std;

move_camera::move_camera(){
  ros::AsyncSpinner spinner(1);
  spinner.start();

  transformationpub = node_handle.advertise<std_msgs::Float32MultiArray>("Transformation", 100);
  camera_origin_coords = node_handle.advertise<geometry_msgs::Vector3>("Camera_Coords",100);
  robot_feature_vector_pub = node_handle.advertise<geometry_msgs::Vector3>("Robot_Feature_Vector",100);

  object_pos[0] = 0.35;
  object_pos[1] = 0.0;
  object_pos[2] = 1.08;

  robot_pos[0] = 0.0;
  robot_pos[1] = 0.0;
  robot_pos[2] = 1.0;

  initial_pose.orientation.w = 0;
  initial_pose.orientation.x= 0.988;
  initial_pose.orientation.y = 0;
  initial_pose.orientation.z = 0.1485;

  initial_pose.position.x = 0.1500;
  initial_pose.position.y = 0.000;
  initial_pose.position.z = 0.6500;

}

geometry_msgs::Pose move_camera::move_camera_to_initialpos(){
  axis_array[0] << 0, 1, 0;
  axis_array[1] << 0.0, 0.707, -0.707;
  axis_array[2] << 0, 0, -1;
  axis_array[3] << 0.0, -0.707, -0.707;
  axis_array[4] << 0.0, -1.0, 0.0;
  axis_array[5] << 0.0, -0.707, 0.707;
  axis_array[6] << 0, 0, 1;
  axis_array[7] << 0.0, 0.707, 0.707;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  target_pose2.orientation.w = 0.81581;
  target_pose2.orientation.x= 0;
  target_pose2.orientation.y = 0.57832;
  target_pose2.orientation.z = 0;

  target_pose2.position.x = 0.1500;
  target_pose2.position.y = 0.000;
  target_pose2.position.z = 0.6500;

  q_net.x() =  target_pose2.orientation.x;
  q_net.y() =  target_pose2.orientation.y;
  q_net.z() =  target_pose2.orientation.z;
  q_net.w() =  target_pose2.orientation.w;

  q_net.normalize();

  camera_coords.x = target_pose2.position.x;
  camera_coords.y = target_pose2.position.y;
  camera_coords.z = target_pose2.position.z;
  camera_origin_coords.publish(camera_coords);


  robot_to_object_frame(robot_pos, object_pos, object_frame);

  end_effector_pos[0] = target_pose2.position.x + robot_pos[0] - object_pos[0];
  end_effector_pos[1] = target_pose2.position.y + robot_pos[1] - object_pos[1];
  end_effector_pos[2] = target_pose2.position.z + robot_pos[2] - object_pos[2];
 cartesian_to_spherical(end_effector_pos);

  move_to_initial_pose(target_pose2);

  robot_to_object_frame(robot_pos, object_pos, object_frame);

  v << target_pose2.position.x + object_frame[0], target_pose2.position.y + object_frame[1], target_pose2.position.z + object_frame[2]; // robot frame -> world -> object frame

  //Apply rotation
  v_start << -v.norm(), 0, 0;
  axis = v_start.cross(v);
  axis << axis[0]/axis.norm(),axis[1]/axis.norm(),axis[2]/axis.norm();
  float angle = acos(v.dot(v_start) / (v.norm() * v_start.norm()));

  Eigen::Quaterniond q1(cos(angle/2), axis.x()*sin(angle/2), axis.y()*sin(angle/2), axis.z()*sin(angle/2)); //axis angle to quaternion d
  q1.normalize(); //Need normalization for correct behaviour

  Rot_Mat = q1.normalized().toRotationMatrix();
  publishTransformation(Rot_Mat, target_pose2);

  for (int i = 0; i<8; i++)
  {
    rotate_vector_by_quaternion(axis_array[i], q1, axis_array[i]);
    //std::cout << axis_array[i]<<"\n -------------\n";
  }

  return target_pose2;
}



int move_camera::move_camera_to_random_dir(bool is_random, int dir){
  ros::AsyncSpinner spinner(1);
  spinner.start();

  if(is_random){
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()

    std::uniform_int_distribution<> dis(0, 7);
    dir = dis(gen);

  }
  if (abs(dir)>7)
  {
    std::cout<<"Error"<<std::endl;
    return 0;
  }

 // v <<  current_pose.position.x + 1.0, current_pose.position.y, current_pose.position.z + 0.1; // robot frame -> world -> object frame
  cout<<"V before rot:"<<v<<endl;
  v_before(0) = v(0);
  v_before(1) = v(1);
  v_before(2) = v(2);

  rotate(v, v, dir);
  cout<<"V after rot:"<<v<<endl;

  //robot_state::RobotState start_state(*group.getCurrentState());

  // group->setStartState(*start_state);
  q_net_before.w() = q_net.w();
  q_net_before.x() = q_net.x();
  q_net_before.y() = q_net.y();
  q_net_before.z() = q_net.z();

  q_net = q*q_net;
  q_net.normalize();

  target_pose2.orientation.w = (q_net).w();
  target_pose2.orientation.x= (q_net).x();
  target_pose2.orientation.y = (q_net).y();
  target_pose2.orientation.z = (q_net).z();

  target_pose2.position.x = v[0] - object_frame[0];
  target_pose2.position.y = v[1] - object_frame[1];
  target_pose2.position.z = v[2] - object_frame[2];

  camera_coords.x = target_pose2.position.x;
  camera_coords.y = target_pose2.position.y;
  camera_coords.z = target_pose2.position.z;
  camera_origin_coords.publish(camera_coords);

  end_effector_pos[0] = target_pose2.position.x + robot_pos[0] - object_pos[0];
  end_effector_pos[1] = target_pose2.position.y + robot_pos[1] - object_pos[1];
  end_effector_pos[2] = target_pose2.position.z + robot_pos[2] - object_pos[2];
  cartesian_to_spherical(end_effector_pos);

  int isPosePossible;
  isPosePossible = move_to_random_pose(target_pose2);
  if (isPosePossible == 1)
  {
    v_random <<  target_pose2.position.x + object_frame[0], target_pose2.position.y + object_frame[1], target_pose2.position.z + object_frame[2]; // robot frame -> world -> object frame

    //Apply rotation
    v_start << -v_random.norm(), 0, 0;
    axis = v_start.cross(v_random);
    axis << axis[0]/axis.norm(),axis[1]/axis.norm(),axis[2]/axis.norm();
    float angle = acos(v_random.dot(v_start) / (v_random.norm() * v_start.norm()));

    Eigen::Quaterniond q1(cos(angle/2), axis.x()*sin(angle/2), axis.y()*sin(angle/2), axis.z()*sin(angle/2)); //axis angle to quaternion d
    q1.normalize(); //Need normalization for correct behaviour

    Rot_Mat = q1.normalized().toRotationMatrix();
    publishTransformation(Rot_Mat, target_pose2);
    return dir;
  }

  if (isPosePossible == 0)
  {
    v(0) = v_before(0);
    v(1) = v_before(1);
    v(2) = v_before(2);
    q_net.w() = q_net_before.w();
    q_net.x() = q_net_before.x();
    q_net.y() = q_net_before.y();
    q_net.z() = q_net_before.z();

    target_pose2.position.x = -100;
    return -1;
  }

}

void move_camera::move_to_initial_pose(geometry_msgs::Pose initial_pose_to_go){
  ros::AsyncSpinner spinner(1);
  spinner.start();

  set_random_object_orientation(initial_pose_to_go);
}

int move_camera::move_to_random_pose(geometry_msgs::Pose random_pose_to_go){
  ros::AsyncSpinner spinner(1);
  spinner.start();

  if (target_pose2.position.z > 0.72 || target_pose2.position.x > 0.58 || target_pose2.position.y > 0.5 || target_pose2.position.z < 0.2 || target_pose2.position.x < -0.2 || target_pose2.position.y < -0.5)
  {
    return 0;
  }
  else
  {

    set_random_object_orientation(random_pose_to_go);

    return 1;
  }
}




void move_camera::robot_to_object_frame(double robot_pos[3], double obj_pos[3], double (&object_frame)[3])
{
  for(int i = 0; i < 2; i++)
  {
    object_frame[i] = robot_pos[i] - obj_pos[i];
  }
}

void move_camera::rotate_vector_by_quaternion(Eigen::Vector3d& v, Eigen::Quaterniond& q, Eigen::Vector3d& vprime)
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

void move_camera::rotate(Eigen::Vector3d& v, Eigen::Vector3d& vprime, int dir){
  Eigen::Vector3d axis;
  axis = axis_array[dir];  //axis of rotation

  //axis << 2.0, 0.0, -2.0;
  q.w() = cos(rot/2);
  q.x() = axis.x()*sin(rot/2);
  q.y() = axis.y()*sin(rot/2);
  q.z() = axis.z()*sin(rot/2); //axis angle to quaternion d
  q.normalize(); //Need normalization for correct behaviour
  //std::cout<<q.vec()<<"\n";

  rotate_vector_by_quaternion(v, q, vprime);
  //Update all axes
  for (int i = 0; i<8; i++)
  {
    rotate_vector_by_quaternion(axis_array[i], q, axis_array[i]);
  }
}

void move_camera::cartesian_to_spherical(double end_effector_pos[3])
{
  ros::AsyncSpinner spinner(1);
  spinner.start();
  double x = end_effector_pos[0];
  double y = end_effector_pos[1];
  double z = end_effector_pos[2];

  double radius_circle = sqrt(x*x + y*y);
  theta_robot = float(acos(x/radius_circle));

  radius_sphere_robot = float(sqrt(x*x + y*y + z*z));
  phi_robot = float(acos(radius_circle/float(z)));
  std::cout<<"Camera theta robot is: "<<theta_robot<<std::endl;
  std::cout<<"Camera phi robot is: "<<phi_robot<<std::endl;

  // robot_feature_vector.x = radius_sphere;
  // robot_feature_vector.y = theta;
  // robot_feature_vector.z = phi;
  // robot_feature_vector_pub.publish(robot_feature_vector);

}

void move_camera::publishTransformation(Eigen::Matrix3d R, geometry_msgs::Pose pose1)
{
 // std_msgs::Float32MultiArray transformArray;
 ros::AsyncSpinner spinner(1);
 spinner.start();
 transformArray.data.clear();
 transformArray.data.push_back(R(0,0));
 transformArray.data.push_back(R(0,1));
 transformArray.data.push_back(R(0,2));
 transformArray.data.push_back(R(1,0));
 transformArray.data.push_back(R(1,1));
 transformArray.data.push_back(R(1,2));
 transformArray.data.push_back(R(2,0));
 transformArray.data.push_back(R(2,1));
 transformArray.data.push_back(R(2,2));
 transformArray.data.push_back(pose1.position.x);
 transformArray.data.push_back(pose1.position.y);
 transformArray.data.push_back(pose1.position.z);

  transformationpub.publish(transformArray);

}


void move_camera::set_random_object_orientation(geometry_msgs::Pose pose_to_go){
  // geometry_msgs::Pose new_pose;

  // new_pose.position.x = 1.0;
  // new_pose.position.y = 3.0;
  pose_to_go.position.z = pose_to_go.position.z + 1;
  // new_pose.orientation.x = float(rand() % 100) * 2.0*M_PI/100.0;
  // new_pose.orientation.y = float(rand() % 100) * 2.0*M_PI/100.0;
  // new_pose.orientation.z = float(rand() % 100) * 2.0*M_PI/100.0 ;
  // new_pose.orientation.w = 0.0;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  geometry_msgs::Twist new_twist;
  new_twist.linear.x = 0.0;
  new_twist.linear.y = 0.0;
  new_twist.linear.z = 0.0;
  new_twist.angular.x = 0.0;
  new_twist.angular.y = 0.0;
  new_twist.angular.z = 0.0;

  std::string name = "kinect";
  std::string frame = "world";
  gazebo_msgs::ModelState modelstate;
  modelstate.model_name = name;
  modelstate.reference_frame = frame;
  modelstate.pose = pose_to_go;
  modelstate.twist = new_twist;

  ros::ServiceClient state_client = node_handle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState setmodelstate;
  setmodelstate.request.model_state = modelstate;
  bool A;
  std::string B;

  A,B = state_client.call(setmodelstate);
  std::cout<<"bool returned"<<A<<std::endl;
  std::cout<<"string returned"<<B<<std::endl;
}

float move_camera::return_robot_theta()
{
  std::cout<<" return function theta: "<<theta_robot<<std::endl;
  return theta_robot;
}

float move_camera::return_robot_phi()
{
  std::cout<<" return function phi: "<<phi_robot<<std::endl;
  return phi_robot;
}
