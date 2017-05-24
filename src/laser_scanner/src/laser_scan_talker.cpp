#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"

int main(int argc, char **argv)
{
// %Tag(INIT)%
  ros::init(argc, argv, "laser_scan_talker");
// %EndTag(INIT)%

// %Tag(NODEHANDLE)%
  //ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

// %Tag(PUBLISHER)%
  //ros::Publisher chatter_pub = n.advertise<sensor_msgs::PointCloud>("Laser_Point_Cloud", 1000);
// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%
  //ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  /*int count = 0;
  while (ros::ok())
  {

    chatter_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();

    ++count;
  }*/


  return 0;
}
