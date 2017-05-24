#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_geometry/laser_geometry.h"
#include "laser_line_extraction/line_extraction_ros.h"
#include "laser_line_extraction/line.h"
#include <ellipse/ellipse_extractor.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_scan_listener");

  ros::NodeHandle nh;
  ros::NodeHandle nh_local("~");
  line_extraction::LineExtractionROS line_extractor(nh, nh_local);
  ellipse_extraction::Ellipse_extractor extractor(nh);

  double frequency;
  nh_local.param<double>("frequency", frequency, 25);
  ros::Rate rate(frequency);

  std::vector<line_extraction::Line> lines;
  while (ros::ok())
  {
    line_extractor.run();
    line_extractor.extract(lines);
    extractor.extract(lines);

    extractor.publish(true);

    ros::spinOnce();
    rate.sleep();

  }

  return 0;
}
