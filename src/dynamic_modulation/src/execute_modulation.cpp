#include "ros/ros.h"
#include <ellipse/ellipse_extractor.h>
#include "modulation_manager.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamic_modulation");

  ros::NodeHandle mh;
  ros::NodeHandle mh_local("~");

  //modulation::Modulation_manager manager(mh, mh_local);

  double frequency;
  mh_local.param<double>("frequency", frequency, 25);
  ros::Rate rate(frequency);

  while (ros::ok())
  {
    //  manager.run();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
