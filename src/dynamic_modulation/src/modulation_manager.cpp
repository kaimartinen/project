#include "../include/dynamic_modulation/modulation_manager.h"


namespace modulation {

  Modulation_manager::Modulation_manager(ros::NodeHandle& m):
  m_(m),
  modulation_(),
  running_(false),
  ellipses_updated_(false)
  {
    ellipse_sub_ = m_.subscribe("ellipses", 1000, &Modulation_manager::callback_ellipse, this);
    listener_.waitForTransform("base_laser_link", "map", ros::Time(0), ros::Duration(10.0));
    ROS_INFO("Modulation_manager started");
  }

  Modulation_manager::Modulation_manager():
  m_(),
  modulation_(),
  running_(false),
  ellipses_updated_(false)
  {
    ellipse_sub_ = m_.subscribe("ellipses", 1000, &Modulation_manager::callback_ellipse, this);
    listener_.waitForTransform("base_laser_link", "map", ros::Time(0), ros::Duration(10.0));

  }

  Modulation_manager::~Modulation_manager(){

  }

  void Modulation_manager::transformMsg(std::vector<ellipse_extraction::Ellipse>& ellipses, ellipse::EllipseList::ConstPtr msg) {
    ellipses.clear();
    for(ellipse::EllipseSeg segment : msg->ellipses) {
      ellipses.push_back(ellipse_extraction::Ellipse(segment.line));
    }
  }

  void Modulation_manager::callback_ellipse(const ellipse::EllipseList::ConstPtr& msg) {
    std::vector<ellipse_extraction::Ellipse> v;
    if (!running_)
    {
      transformMsg(v, msg);
      modulation_.setEllipses(v);
      std::vector<ellipse_extraction::Ellipse> ellipses = modulation_.getEllipses();
      ellipses_updated_ = true;
    }
  }

  Eigen::VectorXf Modulation_manager::run(Eigen::VectorXf& curr_pose, Eigen::VectorXf& curr_speed) {
    tf::StampedTransform map_base_transform;
    try
    {
      listener_.lookupTransform("base_laser_link", "map", ros::Time(0), map_base_transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }
    Eigen::Affine3d affine(Eigen::Affine3d::Identity());
    // tf::poseTFToEigen(map_base_transform, affine);
    Eigen::Vector3d trans;
    trans[0] = curr_pose(7);
    trans[1] = curr_pose(8);
    trans[2] = curr_pose(9);
    // trans = affine.inverse() * trans;
    Eigen::Matrix2f R_world;
    R_world = affine.linear().topLeftCorner<2,2>().cast <float> ();
    // Eigen::MatrixXf f = d.cast <float> ()
    running_ = true;
    modulation_.updateSpeedAndPosition(trans, curr_speed);

    // if (ellipses_updated_)
    // {
      // ROS_INFO("ellipses updated");
      // modulation_.computeModulationMatrix();
      running_ = false;
      // ROS_INFO("res = (%lf, %lf,\n\t\t\t\t\t\t\t %lf, %lf)", modulation_.modulation_(0,0), modulation_.modulation_(0,1), modulation_.modulation_(1,0), modulation_.modulation_(1,1));
      return modulation_.compModulation(R_world);
    // }
    // running_ = false;
    // return curr_speed;
  }

}
