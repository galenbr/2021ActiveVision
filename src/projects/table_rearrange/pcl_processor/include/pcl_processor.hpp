#include "ros/ros.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PCLProcessor {
public:
  PCLProcessor(NodeHandle& n);
  ~PCLProcessor();

  // PCL Functions
private:
  NodeHandle nh;
};
