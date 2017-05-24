#ifndef GAUSSIAN_MIXTURE_MODEL_H
#define GAUSSIAN_MIXTURE_MODEL_H

#include <ros/ros.h>
#include <ros/assert.h>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include "string_tools/stringutil.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "tf/transform_datatypes.h"
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_modulation/modulation_manager.h>
#include <ellipse/ellipse.h>
// #include "pr2_arm_base_control.h"
// Custom messages for GMM
#include <pr2_arm_base_control/array1d.h>
#include <pr2_arm_base_control/array2d.h>
#include <pr2_arm_base_control/GMM.h>
#include <pr2_arm_base_control/Mu.h>
#include <pr2_arm_base_control/Priors.h>
#include <pr2_arm_base_control/Sigma.h>
#include <boost/shared_ptr.hpp>





class GaussianMixtureModel
{
  public:
  	// GaussianMixtureModel();
  	~GaussianMixtureModel();
  	GaussianMixtureModel(ros::NodeHandle &nh,int nr_modes);
  	void MsgToModel(pr2_arm_base_control::GMM gmm_msg);
  	void adaptModel(tf::Transform newGoal);
  	void adaptModel2(tf::Transform newGoal);
  	bool loadFromFile(std::string & filename);
  	void publishMuMarker();
  	void returnModulationMatrix(Eigen::VectorXf eigen_current_pose);
  	void integrateModel(double current_time,double dt,Eigen::VectorXf* current_pose,Eigen::VectorXf* current_speed);
	void precomputeTrajectory(int nrPoints,std::vector<geometry_msgs::Pose>* waypointsWorld,
							  std::vector<geometry_msgs::Pose>* torsoPoints,tf::Transform* gripper_pose,
							  tf::Transform* base_pose);


  	int getNr_modes() const { return _nr_modes; };
  	std::string getType() const { return _type;};
	double getkP() const { return _kP;};
	double getkV() const { return _kV;};
	std::vector<double> getPriors() const { return _Priors;};
	std::vector<Eigen::VectorXf> getMu() const { return _MuEigen;};
	std::vector<Eigen::MatrixXf> getSigma() const { return _Sigma;};
	tf::StampedTransform getGoalState() const { return _goalState;};
	tf::Transform getStartState() const { return _startState;};




  protected:
	int _nr_modes;
	std::string _type;
	double _kP;
	double _kV;
	double _max_speed_gripper;
	double _max_speed_base;
	std::vector<double> _Priors;
	std::vector<std::vector<double> > _Mu;
	std::vector<Eigen::VectorXf> _MuEigen;
	std::vector<Eigen::MatrixXf> _Sigma;
	tf::StampedTransform _goalState;
	tf::Transform _startState;
	tf::Transform _related_object_pose;
	tf::Transform _related_object_grasp_pose;
	std::string _related_object_name;

	ros::NodeHandle nh_;
	ros::Publisher Mu_pub_;
	ros::Publisher Traj_pub_;
	boost::shared_ptr<modulation::Modulation_manager> manager_;
  	// modulation::Modulation_manager manager_;

	template <typename T>
	bool parseVector(std::ifstream & is, std::vector<T> & pts, const std::string & name);
	double gaussPDF(double* current_time,int mode_nr);
	template <typename T1, typename T2>
	T1 extract(const T2& full, const T1& ind);

};



#endif // GAUSSIAN_MIXTURE_MODEL_H
