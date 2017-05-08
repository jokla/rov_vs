#include <ros/ros.h>
#include "rov_vs.h"

int main( int argc, char** argv )
{
  ros::init( argc, argv, "rov_vs" );

  ros::NodeHandle n(std::string("~"));

  rov_vs *node = new rov_vs(n);

  node->spin();

  delete node;

  return 0;
}




