#ifndef _HAND_H_
#define _HAND_H_
#include <gripper_controls/PositionCommand.h>
#include <gripper_controls/Holdcommand.h>

class Hand
{
private:
  int finger_state;

  float read_position(int motor_num);
  bool command_position(int motor_num, float position);
  bool command_torque(int motor_num, float torque);
  bool set_actuator_modes(int size, int mode[]);
  bool set_friction_left(bool friction_surface);
  bool set_friction_right(bool friction_surface);

public:
  Hand();
  bool slide_left_down(gripper_controls::PositionCommand::Request &req, gripper_controls::PositionCommand::Response &res);
  bool slide_left_up(gripper_controls::PositionCommand::Request &req, gripper_controls::PositionCommand::Response &res);
  bool slide_right_down(gripper_controls::PositionCommand::Request &req, gripper_controls::PositionCommand::Response &res);
  bool slide_right_up(gripper_controls::PositionCommand::Request &req, gripper_controls::PositionCommand::Response &res);
  bool rotate_anticlockwise(gripper_controls::PositionCommand::Request &req, gripper_controls::PositionCommand::Response &res);
  bool rotate_clockwise(gripper_controls::PositionCommand::Request &req, gripper_controls::PositionCommand::Response &res);
  bool hold_object(gripper_controls::Holdcommand::Request &req, gripper_controls::Holdcommand::Response &res);
};

#endif // _HAND_H_
