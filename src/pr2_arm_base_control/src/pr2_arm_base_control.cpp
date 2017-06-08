#include "pr2_arm_base_control.h"


// Constructor
RobotDriver::RobotDriver(ros::NodeHandle &nh)
{
	nh_ = nh;
	base_goal_sub_ = nh_.subscribe("/PRCommunicator/Precomp_Trajectory", 10, &RobotDriver::followTrajectory,this);

	// base_scan_sub_ = nh_.subscribe("/base_scan", 10, &RobotDriver::laserObstacleCallback,this);
	//set up the publishers for the robot controller topics
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
	cmd_arm_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/r_arm_controller/command", 1);
	cmd_torso_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/torso_controller/command", 1);
	DesPose_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/Controller/DesiredPose", 1, true);
	//wait for the listener to get the first message
	listener_.waitForTransform("odom_combined","map",  ros::Time(0), ros::Duration(10.0));
	listener_.waitForTransform("map","base_footprint",  ros::Time(0), ros::Duration(10.0));
	listener_.waitForTransform("map","torso_lift_link",  ros::Time(0), ros::Duration(10.0));
	listener_.waitForTransform("map","r_gripper_tool_frame",  ros::Time(0), ros::Duration(10.0));
	// Start planning scene client:
	client_get_scene_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

	// GaussianMixtureModel gaussian_mixture_model(nh_,8);
	// _gaussian_mixture_model = gaussian_mixture_model;
	_gaussian_mixture_model.reset(new GaussianMixtureModel (nh_,8));


	ROS_INFO("Loaded Robot driver. Waiting for Goal");



	tf::StampedTransform current_base_transform;
	tf::StampedTransform current_gripper_transform;
	try
	{
		// listener_.lookupTransform("odom_combined", "base_footprint", ros::Time(0), current_transform);
		listener_.lookupTransform("map", "torso_lift_link", ros::Time(0), current_base_transform);
		listener_.lookupTransform("map", "r_gripper_tool_frame", ros::Time(0), current_gripper_transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}

	// std::vector<geometry_msgs::Pose> gripper_points;
	// std::vector<geometry_msgs::Pose> torso_points;
	//_gaussian_mixture_model->precomputeTrajectory(2000,&gripper_points,&torso_points,&current_gripper_transform,&current_base_transform);
	ROS_INFO("Trajectory send.");
	base_scan_sub_ = nh_.subscribe("/GMM_Path", 10, &RobotDriver::executeAction,this);
	var_command_sub_ = nh_.subscribe("/ReturnModulationMatrix", 10, &RobotDriver::executeCommand,this);
	// executeAction();
}


void RobotDriver::executeCommand(const std_msgs::String msg)
{
	tf::StampedTransform current_base_transform;
	tf::StampedTransform current_gripper_transform;
	try
	{
		listener_.lookupTransform("map", "torso_lift_link", ros::Time(0), current_base_transform);
		listener_.lookupTransform("map", "r_gripper_tool_frame", ros::Time(0), current_gripper_transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}
	Eigen::VectorXf eigen_current_pose(14);
	eigen_current_pose << current_gripper_transform.getOrigin().getX(),current_gripper_transform.getOrigin().getY(), current_gripper_transform.getOrigin().getZ(),
							   current_gripper_transform.getRotation().x(),current_gripper_transform.getRotation().y(),current_gripper_transform.getRotation().z(),current_gripper_transform.getRotation().w(),
							   current_base_transform.getOrigin().getX(),current_base_transform.getOrigin().getY(), current_base_transform.getOrigin().getZ(),
							   current_base_transform.getRotation().x(),current_base_transform.getRotation().y(),current_base_transform.getRotation().z(),current_base_transform.getRotation().w();
	_gaussian_mixture_model->returnModulationMatrix(eigen_current_pose);
}


void RobotDriver::executeAction(const std_msgs::String msg)
{
	ROS_INFO("GMM Path received, executing action");
	// std::string filename = "/home/twelsche/bachelorprojekt/src/pr2_arm_base_control/GMM_Data_reach_Door_handle.csv";
	// std::string filename = "/home/twelsche/bachelorprojekt/src/pr2_arm_base_control/GMM_Data_open_Door.csv";
	std::string filename = msg.data;
	if(!_gaussian_mixture_model->loadFromFile(filename)) {
		return;
	}

	tf::Transform newGoal;
	newGoal.setOrigin(tf::Vector3(1.47,0.325,0.935));
	newGoal.setRotation(tf::Quaternion(0.0,0.0,sqrt(0.5),-sqrt(0.5)));
	// _gaussian_mixture_model->adaptModel(newGoal);

	_gaussian_mixture_model->publishMuMarker();

	tf::StampedTransform current_base_transform;
	tf::StampedTransform current_gripper_transform;
	tf::StampedTransform current_transform;
	try
	{
		listener_.lookupTransform("map", "base_footprint", ros::Time(0), current_transform);
		listener_.lookupTransform("map", "torso_lift_link", ros::Time(0), current_base_transform);
		listener_.lookupTransform("map", "r_gripper_tool_frame", ros::Time(0), current_gripper_transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}

	geometry_msgs::PoseArray trajectory_pose_array;
	trajectory_pose_array.header.stamp = ros::Time::now();//ros::Time(10.0);
	trajectory_pose_array.header.frame_id = "map";
	geometry_msgs::Pose current_gripper_pose;
	tf::poseTFToMsg(current_gripper_transform,current_gripper_pose);
	geometry_msgs::Pose current_base_pose;
	tf::poseTFToMsg(current_base_transform,current_base_pose);
	double current_time = 0;


	std::vector<geometry_msgs::Pose> gripper_points;
	std::vector<geometry_msgs::Pose> torso_points;
	//_gaussian_mixture_model->precomputeTrajectory(3600,&gripper_points,&torso_points,&current_gripper_transform,&current_base_transform);
	//return;



	// set initial pose for arm joints
	trajectory_msgs::JointTrajectory newArmPose;
    trajectory_msgs::JointTrajectoryPoint jointPoint;
	newArmPose.points.push_back(jointPoint);
	newArmPose.points[0].time_from_start = ros::Duration(0.1);

	// set initialt torso pose
	trajectory_msgs::JointTrajectory newTorsoPose;
	trajectory_msgs::JointTrajectoryPoint torsoPoint;
	torsoPoint.positions.push_back(current_base_transform.getOrigin().getZ()-0.75);
	newTorsoPose.joint_names.push_back("torso_lift_joint");
	newTorsoPose.points.push_back(torsoPoint);
	newTorsoPose.points[0].time_from_start = ros::Duration(0.1);


	// Set up stuff for IK
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("right_arm");
	const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
	int counter = 0;
	for(std::size_t i=0; i < joint_names.size(); ++i)
	{
		ROS_INFO_STREAM(joint_names[i].c_str());
		if (i != 3 && i != 6)
		{
			newArmPose.joint_names.push_back(joint_names[counter]);
		}
		counter++;
	}

	Eigen::VectorXf eigen_current_speed(14);
	eigen_current_speed << 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
	Eigen::VectorXf eigen_current_pose(14);
	eigen_current_pose << current_gripper_transform.getOrigin().getX(),current_gripper_transform.getOrigin().getY(), current_gripper_transform.getOrigin().getZ(),
							   current_gripper_transform.getRotation().x(),current_gripper_transform.getRotation().y(),current_gripper_transform.getRotation().z(),current_gripper_transform.getRotation().w(),
							   current_base_transform.getOrigin().getX(),current_base_transform.getOrigin().getY(), current_base_transform.getOrigin().getZ(),
							   current_base_transform.getRotation().x(),current_base_transform.getRotation().y(),current_base_transform.getRotation().z(),current_base_transform.getRotation().w();



	double initialTime = ros::Time::now().toSec();
	double lastTime = 0.0;
	double runningTime = 0.0;
	double slowFactor = 50.0;
	double lastVel = 0;
	ros::Rate rate(50.0);
	double dt = 1.0/(50.0*slowFactor);
	geometry_msgs::Twist base_cmd;
	tf::StampedTransform goal_transform =_gaussian_mixture_model->getGoalState();
	ROS_INFO("Starting motion execution");
	while(true)
	{

		// update current robot poses
		try
		{
			listener_.lookupTransform("map", "base_footprint", ros::Time(0), current_transform);
			listener_.lookupTransform("map", "torso_lift_link", ros::Time(0), current_base_transform);
			listener_.lookupTransform("map", "r_gripper_tool_frame", ros::Time(0), current_gripper_transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
		}

		// set current pose to robot pose
		// eigen_current_pose(0) = current_gripper_transform.getOrigin().getX();
		// eigen_current_pose(1) = current_gripper_transform.getOrigin().getY();
		// eigen_current_pose(2) = current_gripper_transform.getOrigin().getZ();
		eigen_current_pose(3) = current_gripper_transform.getRotation().x();
		eigen_current_pose(4) = current_gripper_transform.getRotation().y();
		eigen_current_pose(5) = current_gripper_transform.getRotation().z();
		eigen_current_pose(6) = current_gripper_transform.getRotation().w();
		eigen_current_pose(7) = current_base_transform.getOrigin().getX();
		eigen_current_pose(8) = current_base_transform.getOrigin().getY();
		eigen_current_pose(9) = current_base_transform.getOrigin().getZ();
		eigen_current_pose(10) = current_base_transform.getRotation().x();
		eigen_current_pose(11) = current_base_transform.getRotation().y();
		eigen_current_pose(12) = current_base_transform.getRotation().z();
		eigen_current_pose(13) = current_base_transform.getRotation().w();
		eigen_current_speed(7) = base_cmd.linear.x;
		eigen_current_speed(8) = base_cmd.linear.y;

		double currentTime = ros::Time::now().toSec();
		currentTime = (currentTime- initialTime);
		dt = (currentTime-lastTime)/slowFactor;
		runningTime +=  dt;
		lastTime = currentTime;
		// ROS_INFO("Time elapsed: (%g/%g) dt: %g",runningTime,1.2*goal_transform.stamp_.toSec(),dt);
		// do integration step
		_gaussian_mixture_model->integrateModel(runningTime, 5*dt,&eigen_current_pose,&eigen_current_speed);
		if(eigen_current_pose(7) != eigen_current_pose(7))
		{
			ROS_INFO("pose is nan");
			return;
		}

		// set desired poses to result from integration step
		current_gripper_pose.position.x = eigen_current_pose(0);
		current_gripper_pose.position.y = eigen_current_pose(1);
		current_gripper_pose.position.z = eigen_current_pose(2);
		current_gripper_pose.orientation.x = eigen_current_pose(3);
		current_gripper_pose.orientation.y = eigen_current_pose(4);
		current_gripper_pose.orientation.z = eigen_current_pose(5);
		current_gripper_pose.orientation.w = eigen_current_pose(6);

		current_base_pose.position.x = eigen_current_pose(7);
		current_base_pose.position.y = eigen_current_pose(8);
		current_base_pose.position.z = eigen_current_pose(9);
		current_base_pose.orientation.x = eigen_current_pose(10);
		current_base_pose.orientation.y = eigen_current_pose(11);
		current_base_pose.orientation.z = eigen_current_pose(12);
		current_base_pose.orientation.w = eigen_current_pose(13);


		// calculate base velocity based on current and desired base poses
		tf::Transform current_desired_base_transform;
		tf::poseMsgToTF(current_base_pose,current_desired_base_transform);
		tf::Transform relative_desired_pose = current_base_transform.inverse() * current_desired_base_transform;
		double roll_current,pitch_current,yaw_current;
		relative_desired_pose.getBasis().getRPY(roll_current,pitch_current,yaw_current);
		base_cmd.angular.z = yaw_current*1.0;
		base_cmd.linear.x = relative_desired_pose.getOrigin().getX()*100.0;
		base_cmd.linear.y = relative_desired_pose.getOrigin().getY()*100.0;

		cmd_vel_pub_.publish(base_cmd);
		lastVel = sqrt(base_cmd.linear.x*base_cmd.linear.x+base_cmd.linear.y*base_cmd.linear.y);


		// set and command new torso configuration
    	trajectory_msgs::JointTrajectoryPoint newTorsoPoint;
    	// newTorsoPoint.positions.push_back(fullBodyTraj_msg.joint_trajectory.points[currentPoseIdx].positions[nrJoints-1]);
    	if (current_base_pose.position.z-0.747 > 0.3135)
    		newTorsoPoint.positions.push_back(0.3135);
    	else
    		newTorsoPoint.positions.push_back(current_base_pose.position.z-0.7467);
    	newTorsoPose.points[0] = newTorsoPoint;
    	newTorsoPose.points[0].time_from_start = ros::Duration(0.1);
    	// cmd_torso_pub_.publish(newTorsoPose);






		// find ik for arm and command new pose:
		tf::Transform currentGripperTransform;
		tf::poseMsgToTF(current_gripper_pose,currentGripperTransform);
		tf::Transform currentToolFrameTransform;
		tf::poseMsgToTF(current_gripper_pose,currentToolFrameTransform);
		tf::Transform offsetGr;
		offsetGr.setIdentity();
		tf::Vector3 translationOffG(-0.18,0.0,0.0);
		offsetGr.setOrigin(translationOffG);
		currentGripperTransform = currentGripperTransform*offsetGr;
		// Transform to /base_footprint frame because planning scene operates here
		currentGripperTransform = current_transform.inverse()*currentGripperTransform;
		geometry_msgs::Pose gripperPoseMsg;
		tf::poseTFToMsg(currentGripperTransform,gripperPoseMsg);
		gripperPoseMsg.position.z += 0.1565;//
		// gripperPoseMsg.position.z -= newTorsoPose.points[0].positions[0]  - 0.17;// 0.1565;//  offset don"t know why this is needed???????
		Eigen::Affine3d state;
		tf::poseMsgToEigen(gripperPoseMsg,state);
		const Eigen::Affine3d &desiredState = state;



		bool found_ik = kinematic_state->setFromIK(joint_model_group, desiredState, 5, 0.1);
		// bool found_ik = kinematic_state->setFromIK(joint_model_group, desiredState, 5, 0.1,constraint_callback_fn);


		// If found solution create and publish arm pose command
		if (found_ik)
		{
			std::vector<double> joint_values;
			std::vector<std::string> joint_names2;
			kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
			trajectory_msgs::JointTrajectoryPoint armJointPoint;
			int counter = 0;
			for(std::size_t i=0; i < joint_names.size(); ++i)
			{
				if (i != 3 && i != 6)
				{
					armJointPoint.positions.push_back(joint_values[counter]);
					counter++;
				}

			}
			newArmPose.points[0] = armJointPoint;
    		newArmPose.points[0].time_from_start = ros::Duration(0.1);
			// publish command for the new joint pose for robot arm
	    	cmd_arm_pub_.publish(newArmPose);
	    	// ROS_INFO("Ik found");
		}
		else
		{
			// ROS_INFO("Ik not found");
		}



		// Publsh desired Pose as marker for rviz
    	visualization_msgs::Marker mark;
        mark.header.frame_id = "/map";
        mark.ns = "Base";
        mark.id = 1;
		double marker_size = 0.08;
        mark.type = visualization_msgs::Marker::CUBE;
	    mark.action = visualization_msgs::Marker::ADD;
	    mark.pose.orientation.w = 1.0;
	    mark.scale.x = marker_size;
	    mark.scale.y = marker_size;
	    mark.scale.z = marker_size;
	    mark.color.g = 1.0;
	    mark.color.a = 1.;


	    tf::Transform offsetT;
		offsetT.setIdentity();
		tf::Vector3 translationOffT(-0.05,0.0,0.0);
		offsetT.setOrigin(translationOffT);
		current_desired_base_transform = current_desired_base_transform*offsetT;
		geometry_msgs::Pose poseMsgT;
		tf::poseTFToMsg(current_desired_base_transform,poseMsgT);
	    mark.pose = poseMsgT;

	    visualization_msgs::Marker mark_gripper;
        mark_gripper.header.frame_id = "/map";
        mark_gripper.ns = "Gripper";
        mark_gripper.id = 2;
        mark_gripper.type = visualization_msgs::Marker::CUBE;
	    mark_gripper.action = visualization_msgs::Marker::ADD;
	    mark_gripper.pose.orientation.w = 1.0;
	    mark_gripper.scale.x = marker_size;
	    mark_gripper.scale.y = marker_size;
	    mark_gripper.scale.z = marker_size;
	    mark_gripper.color.b = 1.0;
	    mark_gripper.color.r = 1.0;
	    mark_gripper.color.a = 1.;

	    mark_gripper.pose = current_gripper_pose;
	    // mark_gripper.pose = gripperPoseMsg;

	    visualization_msgs::MarkerArray ma;
	    ma.markers.push_back(mark);
	    ma.markers.push_back(mark_gripper);



	    // Marker for the Frame
	    std_msgs::ColorRGBA red;
	    red.r = 1.0;
	    red.a = 0.7;
	    std_msgs::ColorRGBA blue;
	    blue.b = 1.0;
	    blue.a = 0.7;
	    std_msgs::ColorRGBA green;
	    green.g = 1.0;
	    green.a = 0.7;
	    // Base
	    visualization_msgs::Marker mark_base_frame;
        mark_base_frame.header.frame_id = "/map";
        mark_base_frame.ns = "Base_Frame";
        mark_base_frame.id = 3;
        mark_base_frame.type = visualization_msgs::Marker::LINE_LIST;
        mark_base_frame.action = visualization_msgs::Marker::ADD;
        mark_base_frame.pose.orientation.w = 1.0;
        mark_base_frame.scale.x = 0.008;
        mark_base_frame.color = blue;
        blue.a = 1.0;
        const double poseCoordsLength = 0.15;
        geometry_msgs::Point ptOrig;
        Eigen::Isometry3d basePose;
        tf::poseMsgToEigen(poseMsgT,basePose);
        ptOrig.x = basePose.translation().x();
        ptOrig.y = basePose.translation().y();
        ptOrig.z = basePose.translation().z();
        //ROS_INFO_STREAM("traj pt at " << pt);
        Eigen::Vector3d xCoord(poseCoordsLength, 0.0, 0.0);
        geometry_msgs::Point ptX;
        tf::pointEigenToMsg(basePose * xCoord, ptX);
        mark_base_frame.points.push_back(ptOrig);
        mark_base_frame.points.push_back(ptX);
        mark_base_frame.colors.push_back(red);
        mark_base_frame.colors.push_back(red);
        Eigen::Vector3d yCoord(0.0, poseCoordsLength, 0.0);
        geometry_msgs::Point ptY;
        tf::pointEigenToMsg(basePose * yCoord, ptY);
        mark_base_frame.points.push_back(ptOrig);
        mark_base_frame.points.push_back(ptY);
        mark_base_frame.colors.push_back(green);
        mark_base_frame.colors.push_back(green);
        Eigen::Vector3d zCoord(0.0, 0.0, poseCoordsLength);
        geometry_msgs::Point ptZ;
        tf::pointEigenToMsg(basePose * zCoord, ptZ);
        mark_base_frame.points.push_back(ptOrig);
        mark_base_frame.points.push_back(ptZ);
        mark_base_frame.colors.push_back(blue);
        mark_base_frame.colors.push_back(blue);
        //Gripper
        visualization_msgs::Marker mark_gripper_frame;
        mark_gripper_frame.header.frame_id = "/map";
        mark_gripper_frame.ns = "Gripper_Frame";
        mark_gripper_frame.id = 4;
        mark_gripper_frame.type = visualization_msgs::Marker::LINE_LIST;
        mark_gripper_frame.action = visualization_msgs::Marker::ADD;
        mark_gripper_frame.pose.orientation.w = 1.0;
        mark_gripper_frame.scale.x = 0.008;
        mark_gripper_frame.color = blue;
        geometry_msgs::Point ptOrig2;
        Eigen::Isometry3d gripperPose;
        tf::poseMsgToEigen(current_gripper_pose,gripperPose);
        ptOrig2.x = gripperPose.translation().x();
        ptOrig2.y = gripperPose.translation().y();
        ptOrig2.z = gripperPose.translation().z();

        tf::pointEigenToMsg(gripperPose * xCoord, ptX);
        mark_gripper_frame.points.push_back(ptOrig2);
        mark_gripper_frame.points.push_back(ptX);
        mark_gripper_frame.colors.push_back(red);
        mark_gripper_frame.colors.push_back(red);

        tf::pointEigenToMsg(gripperPose * yCoord, ptY);
        mark_gripper_frame.points.push_back(ptOrig2);
        mark_gripper_frame.points.push_back(ptY);
        mark_gripper_frame.colors.push_back(green);
        mark_gripper_frame.colors.push_back(green);

        tf::pointEigenToMsg(gripperPose * zCoord, ptZ);
        mark_gripper_frame.points.push_back(ptOrig2);
        mark_gripper_frame.points.push_back(ptZ);
        mark_gripper_frame.colors.push_back(blue);
        mark_gripper_frame.colors.push_back(blue);

        ma.markers.push_back(mark_base_frame);
	    ma.markers.push_back(mark_gripper_frame);








	    DesPose_pub_.publish(ma);


	    // check if gripper reached goal
	    // double dist_to_goal = goal_transform.getOrigin().distance2(currentToolFrameTransform.getOrigin());
	    double dist_to_goal = (goal_transform.getOrigin()-currentToolFrameTransform.getOrigin()).length();
	    if(dist_to_goal<0.005)
	    {
	    	ROS_INFO("Reached goal");
	    	break;
	    }
	    else if(runningTime>1.2*goal_transform.stamp_.toSec())
	    {
	    	ROS_INFO("Maximum duration exceeded");
	    	break;
	    }
	    // else
	    // 	ROS_INFO("dist_to_goal: %g at runningTime (%f/%f)",dist_to_goal,runningTime,goal_transform.stamp_.toSec());

		// try to keep the loop at constant rate:
	  	rate.sleep();
	  	// ROS_INFO("Velocity command send: (%g,%g),runningTime: %g",base_cmd.linear.x ,base_cmd.linear.y,runningTime);
	}
}



















// main callback function for motion generation, Following precomputed trajectory,
// TODO: Online motion generation from Model
	// - subscribe to a model msg instead of array for precomputed trajectory
	// - add another subscriber for the obstacles (obstacles as class variables)
	// -
void RobotDriver::followTrajectory(const geometry_msgs::PoseArray msg)
{
	// trnaform pose array to vector of poses for base and gripper
	// in map-Frame (waypointsWorld) and robot-Frame (waypoints)
	unsigned int traj_length = msg.poses.size();
	ROS_INFO("Trajectory msg with length %u received",traj_length);
	std::vector<geometry_msgs::Pose> waypointsWorld;
	std::vector<geometry_msgs::Pose> torsoPoints;
	tf::Transform offset;
	offset.setIdentity();
	tf::Vector3 translationOff(0.05,0.0,0.0);
	offset.setOrigin(translationOff);
	for (unsigned int j = 0 ; j < traj_length ; j++)
	{
		if (j % 2)
		{
			tf::Transform torsoTransform;
			tf::poseMsgToTF(msg.poses[j],torsoTransform);
			torsoTransform = torsoTransform*offset;
			geometry_msgs::Pose poseMsg;
			tf::poseTFToMsg(torsoTransform,poseMsg);
			torsoPoints.push_back(poseMsg);
		}
		else
		{
			waypointsWorld.push_back(msg.poses[j]);
		}
	}


 	// set initial pose for arm joints
	trajectory_msgs::JointTrajectory newArmPose;
    trajectory_msgs::JointTrajectoryPoint jointPoint;
	int trajectoryLength = waypointsWorld.size();//fullBodyTraj_msg.joint_trajectory.points.size();
	newArmPose.points.push_back(jointPoint);
	newArmPose.points[0].time_from_start = ros::Duration(0.1);

	// set initialt torso pose
	trajectory_msgs::JointTrajectory newTorsoPose;
	trajectory_msgs::JointTrajectoryPoint torsoPoint;
	torsoPoint.positions.push_back(torsoPoints[0].position.z-0.75);
	newTorsoPose.joint_names.push_back("torso_lift_joint");
	newTorsoPose.points.push_back(torsoPoint);
	newTorsoPose.points[0].time_from_start = ros::Duration(0.1);


	// Set up stuff for IK
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("right_arm");
	const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
	int counter = 0;
	for(std::size_t i=0; i < joint_names.size(); ++i)
	{
		ROS_INFO_STREAM(joint_names[i].c_str());
		if (i != 3 && i != 6)
		{
			newArmPose.joint_names.push_back(joint_names[counter]);
		}
		counter++;
	}

	// Collision constraint function GroupStateValidityCallbackFn(),
	// planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(kinematic_model));
	// robot_state::GroupStateValidityCallbackFn constraint_callback_fn = boost::bind(&validityFun::validityCallbackFn, planning_scene, kinematic_state,_2,_3);

	// variables for execute precomputed trajectory:
	double motionDuration = msg.header.stamp.toSec();
	double initialTime = ros::Time::now().toSec();
	int currentPoseIdx = 0;
	int currentHandIdx = 0;
	int Idx = 0;
	geometry_msgs::Twist base_cmd;
	tf::Transform goal_transform;
	geometry_msgs::Transform transform;
    geometry_msgs::Pose currentDesiredBasepose;


	// desired rate for the control loop
	ros::Rate rate(50.0);
	bool done = false;
	double slowFactor = 15.0;
	double lastTime = ros::Time::now().toSec()- initialTime;
	double runningTime = 0;
	double lastVel = 0;
	// Loop for executing precomputed trajectory
	while (!done && nh_.ok())
	{
		double currentTime = ros::Time::now().toSec();

		currentTime = (currentTime- initialTime);

		runningTime = runningTime + (currentTime-lastTime)/slowFactor;
		lastTime = currentTime;
		//get the current robot base pose
		tf::StampedTransform current_transform;
		try
		{
			// listener_.lookupTransform("odom_combined", "base_footprint", ros::Time(0), current_transform);
			listener_.lookupTransform("map", "base_footprint", ros::Time(0), current_transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			break;
		}

		// TODO: calculate desired pose based on model, currentPose and time
		//
		//
		//
		//



		// find closest point to currentTime
		Idx = round(trajectoryLength*runningTime/motionDuration);
		if (Idx > trajectoryLength-1)
			Idx = trajectoryLength-1;
		if (Idx > currentPoseIdx)
		{
			currentPoseIdx++;// = Idx;
			// ROS_INFO("currentPoseIdx: %d, trajectoryLength: %d, slowFactor: %g",currentPoseIdx,trajectoryLength-1,slowFactor);
			currentDesiredBasepose = torsoPoints[currentPoseIdx];
			tf::poseMsgToTF(currentDesiredBasepose,goal_transform);

	    	// set and command new torso configuration
	    	trajectory_msgs::JointTrajectoryPoint newTorsoPoint;
	    	// newTorsoPoint.positions.push_back(fullBodyTraj_msg.joint_trajectory.points[currentPoseIdx].positions[nrJoints-1]);
	    	if (torsoPoints[currentPoseIdx].position.z-0.747 > 0.3135)
	    		newTorsoPoint.positions.push_back(0.3135);
	    	else
	    		newTorsoPoint.positions.push_back(torsoPoints[currentPoseIdx].position.z-0.7467);
	    	newTorsoPose.points[0] = newTorsoPoint;
	    	newTorsoPose.points[0].time_from_start = ros::Duration(0.1);
	    	cmd_torso_pub_.publish(newTorsoPose);

	    	// Publsh desired Pose as marker for rviz
	    	visualization_msgs::Marker mark;
	        mark.header.frame_id = "/map";
	        mark.ns = "Base";
	        mark.id = 1;
			double marker_size = 0.08;
	        mark.type = visualization_msgs::Marker::CUBE;
		    mark.action = visualization_msgs::Marker::ADD;
		    mark.pose.orientation.w = 1.0;
		    mark.scale.x = marker_size;
		    mark.scale.y = marker_size;
		    mark.scale.z = marker_size;
		    mark.color.g = 1.0;
		    mark.color.a = 1.;


		    tf::Transform torsoPose;
		    tf::poseMsgToTF(currentDesiredBasepose,torsoPose);
		    tf::Transform offsetT;
			offsetT.setIdentity();
			tf::Vector3 translationOffT(-0.05,0.0,0.0);
			offsetT.setOrigin(translationOffT);
			torsoPose = torsoPose*offsetT;
			geometry_msgs::Pose poseMsgT;
			tf::poseTFToMsg(torsoPose,poseMsgT);
		    mark.pose = poseMsgT;
		    // mark.pose = currentDesiredBasepose;
		    // mark.pose.position.x = torsoPoints[currentPoseIdx].position.x;
		    // mark.pose.position.y = torsoPoints[currentPoseIdx].position.y;
		    // mark.pose.position.z = torsoPoints[currentPoseIdx].position.z;

		    visualization_msgs::Marker mark_gripper;
	        mark_gripper.header.frame_id = "/map";
	        mark_gripper.ns = "Gripper";
	        mark_gripper.id = 2;
	        mark_gripper.type = visualization_msgs::Marker::CUBE;
		    mark_gripper.action = visualization_msgs::Marker::ADD;
		    mark_gripper.pose.orientation.w = 1.0;
		    mark_gripper.scale.x = marker_size;
		    mark_gripper.scale.y = marker_size;
		    mark_gripper.scale.z = marker_size;
		    mark_gripper.color.b = 1.0;
		    mark_gripper.color.r = 1.0;
		    mark_gripper.color.a = 1.;
		    tf::Transform gripperPose;
		    tf::poseMsgToTF(waypointsWorld[currentPoseIdx],gripperPose);
		    tf::Transform offsetGr;
			offsetGr.setIdentity();
			tf::Vector3 translationOffG(0.18,0.0,0.0);
			offsetGr.setOrigin(translationOffG);
			gripperPose = gripperPose*offsetGr;
			geometry_msgs::Pose poseMsg;
			tf::poseTFToMsg(gripperPose,poseMsg);
		    mark_gripper.pose = poseMsg;

		    visualization_msgs::MarkerArray ma;
		    ma.markers.push_back(mark);
		    ma.markers.push_back(mark_gripper);
		    DesPose_pub_.publish(ma);
		}


		// calculate base velocity based on current and desired base poses
		tf::Transform relative_desired_pose = current_transform.inverse() * goal_transform;
		double roll_current,pitch_current,yaw_current;
		relative_desired_pose.getBasis().getRPY(roll_current,pitch_current,yaw_current);
		base_cmd.angular.z = yaw_current*1.0;
		base_cmd.linear.x = relative_desired_pose.getOrigin().getX()*1.0;
		base_cmd.linear.y = relative_desired_pose.getOrigin().getY()*1.0;
		//if(abs(base_cmd.linear.x) > 1.5)
			//base_cmd.linear.x = 1.5;
		//if(abs(base_cmd.linear.y) > 1.5)
                        //base_cmd.linear.y = 1.5;
		// send out command for base velocity
		if(sqrt(base_cmd.linear.x*base_cmd.linear.x+base_cmd.linear.y*base_cmd.linear.y)<0.015 && lastVel<0.01)
		{
			base_cmd.linear.x = 0.0;
			base_cmd.linear.y = 0.0;
			if(std::abs(base_cmd.angular.z)>0.02) // only react if angle is more then 1 degree off
			{
				cmd_vel_pub_.publish(base_cmd);
			}
			else
				base_cmd.angular.z = 0.0;
		}
		else
		{
			if(std::abs(base_cmd.angular.z)<0.02) // only react if angle is more then 1 degree off
			{
				// ROS_INFO("abs(v_a): %g, v_a: %g",std::abs(base_cmd.angular.z),base_cmd.angular.z);
				base_cmd.angular.z = 0.0;
			}
			cmd_vel_pub_.publish(base_cmd);
		}
		lastVel = sqrt(base_cmd.linear.x*base_cmd.linear.x+base_cmd.linear.y*base_cmd.linear.y);



		// find ik for arm and command new pose:
		tf::Transform currentGripperTransform;
		tf::poseMsgToTF(waypointsWorld[currentPoseIdx],currentGripperTransform);
		// Transform to /base_footprint frame because planning scene operates here
		currentGripperTransform = current_transform.inverse()*currentGripperTransform;
		geometry_msgs::Pose gripperPoseMsg;
		tf::poseTFToMsg(currentGripperTransform,gripperPoseMsg);
		// gripperPoseMsg.position.z -= 0.17; //  newTorsoPose.points[0].positions[0] - 0.17
		gripperPoseMsg.position.z -= newTorsoPose.points[0].positions[0] - 0.17;//  offset don"t know why this is needed???????
		Eigen::Affine3d state;
		tf::poseMsgToEigen(gripperPoseMsg,state);
		const Eigen::Affine3d &desiredState = state;



		bool found_ik = kinematic_state->setFromIK(joint_model_group, desiredState, 5, 0.1);
		// bool found_ik = kinematic_state->setFromIK(joint_model_group, desiredState, 5, 0.1,constraint_callback_fn);


		// If found solution create and publish arm pose command
		if (found_ik)
		{
			std::vector<double> joint_values;
			std::vector<std::string> joint_names2;
			kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
			trajectory_msgs::JointTrajectoryPoint armJointPoint;
			int counter = 0;
			for(std::size_t i=0; i < joint_names.size(); ++i)
			{
				if (i != 3 && i != 6)
				{
					armJointPoint.positions.push_back(joint_values[counter]);
					counter++;
				}

			}
			newArmPose.points[0] = armJointPoint;
    		newArmPose.points[0].time_from_start = ros::Duration(0.1);
			// publish command for the new joint pose for robot arm
	    	cmd_arm_pub_.publish(newArmPose);
			currentHandIdx = currentPoseIdx;
		}


		// check if reached end of motion
		if(currentPoseIdx > trajectoryLength-5){
			done = true;
			double dist_goal = sqrt(relative_desired_pose.getOrigin().x()*relative_desired_pose.getOrigin().x()+relative_desired_pose.getOrigin().y()*relative_desired_pose.getOrigin().y());
			ROS_INFO("Motion finished. Remaining distance for base to goal: %g m)",dist_goal);
		}
		// try to keep the loop at constant rate:
	  	rate.sleep();
	}

}


void RobotDriver::laserObstacleCallback(const sensor_msgs::LaserScan msg)
{
	// TODO: calculate modulation for velocity from obstacles in laserscan
	// ROS_INFO("LaserScan received");
}





int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  RobotDriver driver(nh);
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace validityFun
{

	bool validityCallbackFn(planning_scene::PlanningScenePtr &planning_scene,
	                        // const kinematics_constraint_aware::KinematicsRequest &request,
	                        // kinematics_constraint_aware::KinematicsResponse &response,
							robot_state::RobotStatePtr kinematic_state,
	                        const robot_state::JointModelGroup *joint_model_group,
	                        const double *joint_group_variable_values
	                        // const std::vector<double> &joint_group_variable_values
	                        )
	{
	  	kinematic_state->setJointGroupPositions(joint_model_group,joint_group_variable_values);
	  	// Now check for collisions
	    collision_detection::CollisionRequest collision_request;
	    // collision_request.group_name = "right_arm";
	    collision_detection::CollisionResult collision_result;
	    // ROS_INFO("Planning frame: %s",planning_scene->getPlanningFrame().c_str());



	    collision_detection::AllowedCollisionMatrix acm = planning_scene->getAllowedCollisionMatrix();
	    // moveit_msgs::AllowedCollisionMatrix acm_msg;
	    // acm.getMessage(acm_msg);
	    // ROS_INFO("acm_msg size: %lu",acm_msg.entry_names.size());
	    // ROS_INFO("acm_msg name: %s",acm_msg.entry_names[0].c_str());
	    planning_scene->checkCollision(collision_request, collision_result, planning_scene->getCurrentState());
	    // planning_scene->checkCollision(collision_request, collision_result, planning_scene->getCurrentState(),acm);
	    // planning_scene->checkCollision(collision_request, collision_result, *kinematic_state,acm);
	    if(collision_result.collision)
	    {
	      logDebug("IK solution is in collision");
	      // response.error_code_.val = response.error_code_.GOAL_IN_COLLISION;
	      return false;
	    }
	  return true;
	}
}
