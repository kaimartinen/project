#ifndef MODULATION
#define MODULATION

#include <ellipse/ellipse.h>
#include <pr2_arm_base_control/Mu.h>
#include <pr2_arm_base_control/Priors.h>
#include <pr2_arm_base_control/Sigma.h>
#include <pr2_arm_base_control/GMM.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <cmath>
#include <ros/ros.h>


namespace modulation {

class Modulation {
private:
  std::vector<ellipse_extraction::Ellipse> ellipses_;
  Eigen::Vector3d position_;
  Eigen::Vector3d gripper_position_;
  Eigen::VectorXf speed_;

  std::vector<double> lambda_;
  std::vector<double> gamma_;
  std::vector<std::vector<double> > xi_wave_;

  void computeXiWave();
  void computeGamma();
  double computeWeight(int k);
  std::vector<double> computeEigenvalue(int k);
  std::vector<double>  computeHyperplane(int k);
  std::vector<std::vector<double> > computeEBase(int k, std::vector<double> normal);
  Eigen::MatrixXf assembleD_k(int k);
  Eigen::MatrixXf assembleE_k(int k);


public:
  Modulation(Eigen::Vector3d& curr_position, Eigen::VectorXf& curr_speed);
  Modulation();
  ~Modulation();

  Eigen::MatrixXf modulation_;

  void updateSpeedAndPosition(Eigen::Vector3d& curr_pose, Eigen::VectorXf& curr_speed,Eigen::Vector3d& curr_gripper_pose);
  void computeModulationMatrix(Eigen::Matrix2f R_world);

  std::vector<ellipse_extraction::Ellipse>& getEllipses();

  void setEllipses(std::vector<ellipse_extraction::Ellipse> ellipses);

  static double computeL2Norm(std::vector<double> v);

  Eigen::VectorXf compModulation(Eigen::Matrix2f R_world);


};

}

#endif
