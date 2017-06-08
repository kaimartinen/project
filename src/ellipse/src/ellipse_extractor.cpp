#include <ellipse/ellipse_extractor.h>
#include <ros/ros.h>
#include <ellipse/ellipse.h>
#include <laser_line_extraction/line.h>
#include <visualization_msgs/Marker.h>
#include <boost/array.hpp>

namespace ellipse_extraction {

  Ellipse_extractor::Ellipse_extractor(ros::NodeHandle& nh):
  nh_(nh)
  {
    ellipse_marker_ = nh.advertise<visualization_msgs::MarkerArray>("ellipse_marker", 1);
    ellipse_ends_ = nh.advertise<visualization_msgs::Marker>("ellipse_ends", 1);
    ellipse_publisher_ = nh.advertise<ellipse::EllipseList>("ellipses", 1);
    listener_.waitForTransform("map", "base_laser_link", ros::Time(0), ros::Duration(10.0));
  }

  Ellipse_extractor::~Ellipse_extractor(){

  }

  void Ellipse_extractor::extract(std::vector<line_extraction::Line> &lines) {
    ellipses_.clear();
    double width = 2.4;
    for (std::vector<line_extraction::Line>::const_iterator cit = lines.begin();
         cit != lines.end(); ++cit) {

      line_extraction::Line newline = *cit;
      // transform ellipses to /map frame
      tf::StampedTransform map_base_transform;
      try
      {
        listener_.lookupTransform("map", "base_laser_link", ros::Time(0), map_base_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
      }

      Eigen::Affine3d affine(Eigen::Affine3d::Identity());
      tf::poseTFToEigen(map_base_transform, affine);
      Eigen::Vector3d trans;
      trans[0] = cit->getStart()[0];
      trans[1] = cit->getStart()[1];
      trans[2] = 0;
      trans = affine * trans;
      boost::array<double, 2> start;
      start[0] = trans[0];
      start[1] = trans[1];
      newline.setStart(start);


      trans[0] = cit->getEnd()[0];
      trans[1] = cit->getEnd()[1];
      trans = affine * trans;
      boost::array<double, 2> end;
      end[0] = trans[0];
      end[1] = trans[1];
      newline.setEnd(end);


      // ellipses_.push_back(ellipse_extraction::Ellipse(*cit, (cit->length() / 2), width));
      ellipses_.push_back(ellipse_extraction::Ellipse(newline, (cit->length() / 2), width));
    }
  }

  void Ellipse_extractor::populateMarkerMsg(visualization_msgs::MarkerArray& marker_array)
  {
    visualization_msgs::Marker marker_msg;
    marker_msg.ns = "ellipse_extraction";




    marker_msg.type = visualization_msgs::Marker::CYLINDER;
    marker_msg.color.r = 0.0;
    marker_msg.color.g = 0.0;
    marker_msg.color.b = 1.0;
    marker_msg.color.a = 0.2;
    marker_msg.lifetime = ros::Duration(0.5);
    // marker_msg.header.frame_id = "base_laser_link";
    marker_msg.header.frame_id = "map";
    marker_msg.header.stamp = ros::Time::now();
    for (int i = 0; i < ellipses_.size(); i++)
    {
      marker_msg.id = i;
      marker_msg.pose.position.x = ellipses_[i].getPPoint()[0];
      marker_msg.pose.position.y = ellipses_[i].getPPoint()[1];
      marker_msg.pose.orientation.x = 0;
      marker_msg.pose.orientation.y = 0;
      marker_msg.pose.orientation.z = -sin(ellipses_[i].getAlpha()/2);
      marker_msg.pose.orientation.w = cos(ellipses_[i].getAlpha()/2);
      marker_msg.scale.y = ellipses_[i].getWidth()*2;
      marker_msg.scale.x = ellipses_[i].getHeight()*2;
      marker_msg.scale.z = 1.0;
      marker_array.markers.push_back(marker_msg);
    }

  }

  void Ellipse_extractor::populateEndMarkerMsg(visualization_msgs::Marker& marker_msg) {
    marker_msg.ns = "ellipse_extraction";

    marker_msg.type = visualization_msgs::Marker::POINTS;
    marker_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.color.r = 1.0;
    marker_msg.color.g = 0.0;
    marker_msg.color.b = 0.0;
    marker_msg.color.a = 1.0;
    marker_msg.header.frame_id = "base_laser_link";
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.scale.y = 0.05;
    marker_msg.scale.x = 0.05;
    marker_msg.scale.z = 0.05;

    marker_msg.id = 1;

    for (int i = 0; i < ellipses_.size(); i++)
    {
      Eigen::Vector2f startVec, endVec;
      startVec = {ellipses_[i].getStart()[0], ellipses_[i].getStart()[1]};
      endVec = {ellipses_[i].getEnd()[0], ellipses_[i].getEnd()[1]};

      startVec = ellipses_[i].getR() * startVec;
      endVec = ellipses_[i].getR() * endVec;

      geometry_msgs::Point point;
      point.x = startVec(0);
      point.y = startVec(1);
      point.z = 0.5;
      marker_msg.points.push_back(point);
      marker_msg.colors.push_back(marker_msg.color);

      point.x = endVec(0);
      point.y = endVec(1);
      point.z = 0.5;
      marker_msg.colors.push_back(marker_msg.color);
      marker_msg.points.push_back(point);

    }

  }

  void Ellipse_extractor::populateEllipseListMsg(ellipse::EllipseList& ellipse_list) {
    for (int i = 0; i < ellipses_.size(); i++)
    {
      ellipse::EllipseSeg ellipse;
      laser_line_extraction::LineSegment line;
      line.radius = ellipses_[i].getLine().getRadius();
      line.angle =  ellipses_[i].getLine().getAngle();
      line.covariance =  ellipses_[i].getLine().getCovariance();
      line.start =  ellipses_[i].getLine().getStart();
      line.end =  ellipses_[i].getLine().getEnd();
      ellipse.line = line;
      ellipse.width = ellipses_[i].getWidth();
      ellipse.height = ellipses_[i].getHeight();
      ellipse.p1 = ellipses_[i].getP1();
      ellipse.p2 = ellipses_[i].getP2();
      ellipse_list.ellipses.push_back(ellipse);
    }
    ellipse_list.size = ellipses_.size();
    ellipse_list.header.frame_id = "base_link";
    ellipse_list.header.stamp = ros::Time::now();
  }

  void Ellipse_extractor::publish(bool visual){
    ellipse::EllipseList ellipses;
    populateEllipseListMsg(ellipses);
    ellipse_publisher_.publish(ellipses);

    if (visual) {
      visualization_msgs::MarkerArray marker_msg;
      visualization_msgs::Marker points;
      populateMarkerMsg(marker_msg);
      populateEndMarkerMsg(points);
      ellipse_marker_.publish(marker_msg);
      ellipse_ends_.publish(points);
    }
  }

  void Ellipse_extractor::extract_from_msg(ellipse::EllipseList& ellipse_list) {
    ellipses_.clear();
    for(size_t i = 0; i < ellipse_list.size; i++) {
      laser_line_extraction::LineSegment line_seg = ellipse_list.ellipses[i].line;
      boost::array<double, 4> cov = {line_seg.covariance[0], line_seg.covariance[1],
                                     line_seg.covariance[2], line_seg.covariance[3]};
      boost::array<double, 2> start ={line_seg.start[0], line_seg.start[1]};
      boost::array<double, 2> end ={line_seg.end[0], line_seg.end[1]};
      //problem with indices calc
      std::vector<unsigned int> indices = {};
      line_extraction::Line line(line_seg.angle, line_seg.radius,
                                 cov, start, end, indices);
      ellipses_.push_back(ellipse_extraction::Ellipse(line, ellipse_list.ellipses[i].height, ellipse_list.ellipses[i].width));
    }
  }

  std::vector<ellipse_extraction::Ellipse>& Ellipse_extractor::getEllipses() {
    return ellipses_;
  }

}
