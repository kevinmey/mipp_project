#include <ros/ros.h>

#include <string>
#include <math.h> /* sqrt, pow */
 
class RRTPlanner
{
public:
  // Constructor
  RRTPlanner(ros::NodeHandle n, ros::NodeHandle np);
  // Destructor
  ~RRTPlanner();
  
  /* 
  *  Utility functions
  */
  
  /**
  * @brief Get parameters from rosparam in launch.
  * @param np Private nodehandle.
  */
  void getParams(ros::NodeHandle np);
private:
};
