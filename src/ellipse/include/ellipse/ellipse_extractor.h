#ifndef ELLIPSE_EXTRACTOR
#define ELLIPSE_EXTRACTOR

#include "laser_line_extraction/line.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "ellipse/ellipse.h"
#include "ellipse/EllipseSeg.h"
#include "ellipse/EllipseList.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include "tf/transform_datatypes.h"
#include <Eigen/Core>
#include <Eigen/Geometry>


namespace ellipse_extraction
{

class Ellipse_extractor
{
private:
  ros::Publisher ellipse_publisher_;
  ros::Publisher ellipse_marker_;
  ros::Publisher ellipse_ends_;
  tf::TransformListener listener_;

  ros::NodeHandle nh_;
  std::vector<ellipse_extraction::Ellipse> ellipses_;

  void populateMarkerMsg(visualization_msgs::MarkerArray& marker_msg);
  void populateEllipseListMsg(ellipse::EllipseList& ellipse_list);
  void populateEndMarkerMsg(visualization_msgs::Marker& marker_msg);

public:
  Ellipse_extractor(ros::NodeHandle &nh);
  ~Ellipse_extractor();
  void extract(std::vector<line_extraction::Line> &lines);
  void publish(bool visual);

  std::vector<ellipse_extraction::Ellipse>& getEllipses();
  void extract_from_msg(ellipse::EllipseList& ellipse_list);
};

}

#endif
