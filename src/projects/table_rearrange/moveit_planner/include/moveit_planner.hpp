// Header file
#ifndef MOVEIT_PLANNER_H
#define MOVEIT_PLANNER_H

// std includes
#include <string>

// moveit includes
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>

namespace moveit_planner {

  // Class containing motion-planning methods (uses moveit C++ API)
  // NOTE: Requires moveit to be launched
  class MoveitPlanner {
  public:
    MoveitPlanner(const std::string&);
    ~MoveitPlanner();

    // Getters/Setters
    std::string getDescription() {return robot_description;};
  private:
    std::string robot_description;

    // moveit stuff
    robot_model_loader::RobotModelLoader loader;
    robot_model::RobotModelPtr modelPtr;
  };
}

#endif
