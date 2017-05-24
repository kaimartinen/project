#ifndef MODULATION_MANAGER
#define MODULATION_MANAGER

#include "../../src/modulation.h"
#include <ros/ros.h>
#include <ellipse/EllipseList.h>
#include <ellipse/EllipseSeg.h>
#include <ellipse/ellipse_extractor.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <laser_line_extraction/line_extraction_ros.h>
#include <laser_line_extraction/line.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "tf/transform_datatypes.h"


namespace modulation {

class Modulation_manager {
private:
  ros::NodeHandle m_;
  ros::Subscriber ellipse_sub_;
  bool ellipses_updated_;
  bool running_;
  modulation::Modulation modulation_;
  tf::TransformListener listener_;

  void callback_ellipse(const ellipse::EllipseList::ConstPtr&);
  void transformMsg(std::vector<ellipse_extraction::Ellipse>& ellipses, ellipse::EllipseList::ConstPtr msg);


public:
  Modulation_manager(ros::NodeHandle& m);
  Modulation_manager();
  ~Modulation_manager();

  Eigen::VectorXf run(Eigen::VectorXf& curr_pose, Eigen::VectorXf& curr_speed);

};

}

#endif
