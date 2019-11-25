#include "moveit_planner.hpp"

namespace moveit_planner {
  // TODO: Implement motion planning functions
  MoveitPlanner::MoveitPlanner(const std::string& robot_desc) :
    robot_description{robot_desc}, loader{robot_desc} {
      modelPtr = loader.getModel();
    }
  MoveitPlanner::~MoveitPlanner() {}
}
