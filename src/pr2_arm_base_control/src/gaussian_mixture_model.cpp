#include "gaussian_mixture_model.h"

// GaussianMixtureModel::GaussianMixtureModel()
// {
// }

GaussianMixtureModel::GaussianMixtureModel(ros::NodeHandle &nh,int nr_modes) :
nh_(nh)
{
	_nr_modes = nr_modes;
	_kP = 400;
	_kV = 10;
	_max_speed_gripper = 2.9;
	_max_speed_base = 1.9;

	manager_.reset(new modulation::Modulation_manager(nh));

	Mu_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/GMM/Mu", 1, true);
	Traj_pub_ = nh.advertise<geometry_msgs::PoseArray>("/PRCommunicator/Precomp_Trajectory_Raw", 1,true);
	Traj_pub2_ = nh.advertise<geometry_msgs::PoseArray>("/PRCommunicator/Precomp_Trajectory", 1,true);
	ROS_INFO("GMM initilaized");
}

GaussianMixtureModel::~GaussianMixtureModel()
{
}



void GaussianMixtureModel::MsgToModel(pr2_arm_base_control::GMM gmm_msg)
{
	//TODO
}

void GaussianMixtureModel::adaptModel2(tf::Transform newGoal)
{

}


void GaussianMixtureModel::adaptModel(tf::Transform newGoal)
{
	ROS_INFO("adapting Model");
	// Matrix for translation to new start
	tf::Transform T1;
	T1.setIdentity();
	T1.setOrigin(newGoal.getOrigin() - _related_object_pose.getOrigin());
	// Rotation
	tf::Transform GR;
	GR.setIdentity();
	GR.setRotation(newGoal.getRotation());
	// if grasp object
	tf::Transform MGR;
	MGR.setIdentity();
	MGR.setRotation(_related_object_pose.getRotation());
	tf::Transform T;
	T = GR * MGR.inverse();
	T.setOrigin(- (T*newGoal.getOrigin())+newGoal.getOrigin());
	ROS_INFO("T: (%g,%g,%g), (%g,%g,%g,%g)",T.getOrigin().x(),T.getOrigin().y(),T.getOrigin().z(),T.getRotation().x(),T.getRotation().y(),T.getRotation().z(),T.getRotation().w());

	// loop over gaussians
	std::vector<Eigen::VectorXf> _MuEigenBck = _MuEigen;
	_MuEigen.clear();
	for(int i=0;i<_nr_modes;i++)
	{
		tf::Transform tf_Mu_gr_i;
		tf_Mu_gr_i.setOrigin(tf::Vector3(_MuEigenBck[i](1),_MuEigenBck[i](2),_MuEigenBck[i](3)));
		tf_Mu_gr_i.setRotation(tf::Quaternion(_MuEigenBck[i](4),_MuEigenBck[i](5),_MuEigenBck[i](6),_MuEigenBck[i](7)));
		tf::Transform tf_Mu_base_i;
		tf_Mu_base_i.setOrigin(tf::Vector3(_MuEigenBck[i](8),_MuEigenBck[i](9),_MuEigenBck[i](10)));
		tf_Mu_base_i.setRotation(tf::Quaternion(_MuEigenBck[i](11),_MuEigenBck[i](12),_MuEigenBck[i](13),_MuEigenBck[i](14)));

		// perform translation
		tf_Mu_gr_i.setOrigin(T1*tf_Mu_gr_i.getOrigin());
		tf_Mu_base_i.setOrigin(T1*tf_Mu_base_i.getOrigin());
		// perform rotation
		tf_Mu_gr_i = T*tf_Mu_gr_i;
		tf_Mu_base_i = T*tf_Mu_base_i;

		// set _MuEigen
		double time_i = _MuEigenBck[i](0);
		Eigen::VectorXf Mu_i_eigen(15);
		Mu_i_eigen << time_i, tf_Mu_gr_i.getOrigin().x(),tf_Mu_gr_i.getOrigin().y(),tf_Mu_gr_i.getOrigin().z(),
					   tf_Mu_gr_i.getRotation().x(),tf_Mu_gr_i.getRotation().y(),tf_Mu_gr_i.getRotation().z(),tf_Mu_gr_i.getRotation().w(),
					   tf_Mu_base_i.getOrigin().x(),tf_Mu_base_i.getOrigin().y(),tf_Mu_base_i.getOrigin().z(),
					   tf_Mu_base_i.getRotation().x(),tf_Mu_base_i.getRotation().y(),tf_Mu_base_i.getRotation().z(),tf_Mu_base_i.getRotation().w();
		_MuEigen.push_back(Mu_i_eigen);

		// transform Sigma
		Eigen::MatrixXf TS(4,4);
		TS.setIdentity();
		Eigen::Matrix3d T_h(3,3);
		tf::matrixTFToEigen(T.getBasis(),T_h);
		TS.block(1,1,3,3) << T_h(0,0),T_h(0,1),T_h(0,2),T_h(1,0),T_h(1,1),T_h(1,2),T_h(2,0),T_h(2,1),T_h(2,2);
		Eigen::MatrixXf Sigma_gripper_t(4,4);
		Sigma_gripper_t = _Sigma[i].block(0,0,4,4);
		Sigma_gripper_t = TS*Sigma_gripper_t*TS;
		_Sigma[i].block(0,0,4,4) = Sigma_gripper_t;

		Eigen::MatrixXf Sigma_base_t(4,4);
		Sigma_base_t(0,0) = _Sigma[i](0,0);
		Sigma_base_t.block(1,0,3,1) = _Sigma[i].block(7,0,3,1);
		Sigma_base_t.block(0,1,1,3) = _Sigma[i].block(0,7,1,3);
		Sigma_base_t.block(1,1,3,3) = _Sigma[i].block(7,7,3,3);
		Sigma_base_t = TS*Sigma_base_t*TS;
		_Sigma[i](0,0) = Sigma_base_t(0,0);
		_Sigma[i].block(7,0,3,1) = Sigma_base_t.block(1,0,3,1);
		_Sigma[i].block(0,7,1,3) = Sigma_base_t.block(0,1,1,3);
		_Sigma[i].block(7,7,3,3) = Sigma_base_t.block(1,1,3,3);
	}

	ROS_INFO("Model adapted");
}

double GaussianMixtureModel::gaussPDF(double* current_time,int mode_nr)
{
	double t_m = *current_time - _MuEigen[mode_nr](0);
	double prob = t_m*t_m/_Sigma[mode_nr](0,0);
	prob = exp(-0.5 * prob)/sqrt(2.0*3.141592*_Sigma[mode_nr](0,0));
	return prob;
}


template <typename T1, typename T2>
T1 GaussianMixtureModel::extract(const T2& full, const T1& ind)
{
    int num_indices = ind.innerSize();
    T1 target(num_indices);
    for (int i = 0; i < num_indices; i++)
    {
        target[i] = full[ind[i]];
    }
    return target;
}

void GaussianMixtureModel::returnModulationMatrix(Eigen::VectorXf eigen_current_pose)
{
	Eigen::VectorXf eigen_current_speed(14);
	eigen_current_speed << 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
	manager_->run(eigen_current_pose, eigen_current_speed);
}


void GaussianMixtureModel::integrateModel(double current_time,double dt,Eigen::VectorXf* current_pose,Eigen::VectorXf* current_speed)
{
	Eigen::VectorXf ind_vec_out(14);
	ind_vec_out << 1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0;
	// *current_time += dt;
	// activation weights
	std::vector<double> H;
	double sumH = 0.0;
	for(int i=0;i<_nr_modes;i++)
	{
		double hi = gaussPDF(&current_time,i);
		H.push_back(hi);
		sumH += hi;
	}
	tf::Quaternion current_gripper_q((*current_pose)(3),(*current_pose)(4),(*current_pose)(5),(*current_pose)(6));
	tf::Quaternion current_base_q(0.0,0.0,(*current_pose)(12),(*current_pose)(13));
	// acceleration
	Eigen::VectorXf currF(14);
	currF << 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
	double curr_max_H = 0.0;
	int use_i = 0;
	for(int i=0;i<_nr_modes;i++)
	{
		Eigen::VectorXf FTmp(14);
		Eigen::MatrixXf Sigma_out_in(14,1);
		Sigma_out_in = _Sigma[i].block(1,0,14,1);
		FTmp = extract(_MuEigen[i],ind_vec_out) + Sigma_out_in*(1.0/_Sigma[i](0,0))*(current_time-_MuEigen[i](0));
		currF += FTmp * H[i]/sumH;
		// for Rotation part
		if(H[i]>curr_max_H)
		{
			curr_max_H = H[i];
			use_i = i;
		}
	}
	tf::Quaternion current_desired_gripper_q(_MuEigen[use_i](4),_MuEigen[use_i](5),_MuEigen[use_i](6),_MuEigen[use_i](7));
	tf::Quaternion relative = current_desired_gripper_q * current_gripper_q.inverse();
	tf::Vector3 v_rot_gripper =	relative.getAxis () * relative.getAngle();
	tf::Vector3 a_rot_gripper = _kP*v_rot_gripper - _kV*tf::Vector3((*current_speed)(3),(*current_speed)(4),(*current_speed)(5));

	tf::Quaternion current_desired_base_q(0.0,0.0,_MuEigen[use_i](13),_MuEigen[use_i](14));
	// tf::Quaternion relative_base = current_desired_base_q * current_base_q.inverse();
	tf::Quaternion relative_base = current_desired_base_q * current_base_q.inverse();
	// tf::Vector3 v_rot_base =	relative_base.getAxis () * relative_base.getAngle();
	double base_angle;
	if(relative_base.getAngle() > 3.1415)
		base_angle = relative_base.getAngle()- 2.0*3.1415;
	else
		base_angle = relative_base.getAngle();
	tf::Vector3 a_rot_base = _kP*tf::Vector3(0,0,-base_angle) - _kV*tf::Vector3(0,0,(*current_speed)(12));

	// ROS_INFO("relative_base_angle: %f",relative_base.getAngle());
	// ROS_INFO("a_angle: %f",a_rot_base.z());
	// (*current_speed)(7) = 0.0;
	// (*current_speed)(8) = 0.0;
	Eigen::VectorXf currAcc(14);
	currAcc = - *current_pose * _kP - *current_speed * _kV + currF * _kP;
	currAcc(3) = a_rot_gripper.x();
	currAcc(4) = a_rot_gripper.y();
	currAcc(5) = a_rot_gripper.z();
	currAcc(6) = 0;
	currAcc(10) = 0;
	currAcc(11) = 0;
	currAcc(12) = a_rot_base.z();
	currAcc(13) = 0;

	// update velocity
	*current_speed = *current_speed + currAcc*dt;
	// limit speed to max_speed
	if(std::abs(current_speed->coeffRef(0)) >_max_speed_gripper || std::abs(current_speed->coeffRef(1)) >_max_speed_gripper || std::abs(current_speed->coeffRef(2)) >_max_speed_gripper)
	{
		double max_element = std::max(std::abs(current_speed->coeffRef(0)),std::abs(current_speed->coeffRef(1)));
		max_element = std::max(max_element,std::abs((double)current_speed->coeffRef(2)));
		current_speed->coeffRef(0) = current_speed->coeffRef(0)/max_element*_max_speed_gripper;
		current_speed->coeffRef(1) = current_speed->coeffRef(1)/max_element*_max_speed_gripper;
		current_speed->coeffRef(2) = current_speed->coeffRef(2)/max_element*_max_speed_gripper;
	}

	if(std::abs(current_speed->coeffRef(7)) >_max_speed_base || std::abs(current_speed->coeffRef(8)) >_max_speed_base || std::abs(current_speed->coeffRef(9)) >_max_speed_base)
	{
		double max_element = std::max(std::abs(current_speed->coeffRef(7)),std::abs(current_speed->coeffRef(8)));
		current_speed->coeffRef(7) = current_speed->coeffRef(7)/max_element*_max_speed_base;
		current_speed->coeffRef(8) = current_speed->coeffRef(8)/max_element*_max_speed_base;
	}

	// Hier muss dann die Modulation der Geschwindigkeit basierend auf den ellipsen eingefügt werden!!!!!
	// ROS_INFO("Current base speed before run  : %lf, %lf, %lf", (*current_speed)[7], (*current_speed)[8], (*current_speed)[9]);
	*current_speed = manager_->run(*current_pose, *current_speed);
	// ROS_INFO("Base speed modulated after run: %lf, %lf, %lf", (*current_speed)[7], (*current_speed)[8], (*current_speed)[9]);

	if(std::abs(current_speed->coeffRef(7)) >_max_speed_base || std::abs(current_speed->coeffRef(8)) >_max_speed_base || std::abs(current_speed->coeffRef(9)) >_max_speed_base)
	{
		double max_element = std::max(std::abs(current_speed->coeffRef(7)),std::abs(current_speed->coeffRef(8)));
		current_speed->coeffRef(7) = current_speed->coeffRef(7)/max_element*_max_speed_base;
		current_speed->coeffRef(8) = current_speed->coeffRef(8)/max_element*_max_speed_base;
	}

	// update position
	*current_pose = *current_pose + *current_speed*dt; //+ currAcc*(dt*dt*0.5);
	// update rotation
	double alpha_N = (tf::Vector3((*current_speed)(3),(*current_speed)(4),(*current_speed)(5))).length()*dt;
	tf::Quaternion Q(tf::Vector3((*current_speed)(3),(*current_speed)(4),(*current_speed)(5)).normalized(),alpha_N);
	tf::Quaternion Q2 = Q*current_gripper_q;

	tf::Quaternion Q3(tf::Vector3(0.0,0.0,1.0),(*current_speed)(12)*dt);
	tf::Quaternion Q4 = Q3*current_base_q;

	(*current_pose)(3) = Q2.x();//current_desired_gripper_q.x();//
	(*current_pose)(4) = Q2.y();//current_desired_gripper_q.y();//
	(*current_pose)(5) = Q2.z();//current_desired_gripper_q.z();//
	(*current_pose)(6) = Q2.w();//current_desired_gripper_q.w();//

	(*current_pose)(10) = 0.0;
	(*current_pose)(11) = 0.0;
	(*current_pose)(12) = current_desired_base_q.z();//Q4.z();//
	(*current_pose)(13) = current_desired_base_q.w();//Q4.w();//

}

void GaussianMixtureModel::Trajectory2PoseArray(geometry_msgs::PoseArray* poseArray_in,
                          geometry_msgs::PoseArray* poseArray_transformed)
{
    poseArray_transformed->header.stamp = ros::Time(10.0);
    poseArray_transformed->header.frame_id = "map";
    // add gripper pose
    for (int i = 0;i<poseArray_in->poses.size();i +=3)
    {
        // Eigen::Isometry3d poseBase = trajectoryBase[i]->estimate();
        // Eigen::Isometry3d poseGripper = trajectoryGripper[i]->estimate();
        // Eigen::Isometry3d poseWristLink = poseGripper;
        // // transform gripper poses to wrist poses (end of chain)
        // poseWristLink.translation() = poseWristLink.translation() -  poseGripper.linear()*Eigen::Vector3d(0.18,0.0,0.0);
        // Eigen::Isometry3d poseRelative = poseWristLink;
        // geometry_msgs::Pose poseMsg;
        // geometry_msgs::Point positionMsg;
        // geometry_msgs::Quaternion orientationMsg;
        // positionMsg.x = poseRelative.translation().x();
        // positionMsg.y = poseRelative.translation().y();
        // positionMsg.z = poseRelative.translation().z();
        // poseMsg.position = positionMsg;
        // Eigen::Quaterniond Q(poseRelative.linear());
        // orientationMsg.x = Q.x();
        // orientationMsg.y = Q.y();
        // orientationMsg.z = Q.z();
        // orientationMsg.w = Q.w();
        // poseMsg.orientation = orientationMsg;

        tf::Transform T_gripper_pose;
        tf::poseMsgToTF(poseArray_in->poses[i],T_gripper_pose);

        tf::Transform T;
		T.setOrigin(tf::Vector3(-0.18,0.0,0.0));
		T.setRotation(tf::Quaternion(0,0,0,1));
		T_gripper_pose = T_gripper_pose*T;
        geometry_msgs::Pose poseMsg;
        tf::poseTFToMsg(T_gripper_pose,poseMsg);

        poseArray_transformed->poses.push_back(poseMsg);
        // add torso_linft_link poses
        poseArray_transformed->poses.push_back(poseArray_in->poses[i+1]);
    }
    
    
}





void GaussianMixtureModel::precomputeTrajectory(int nrPoints,std::vector<geometry_msgs::Pose>* gripper_points,
												std::vector<geometry_msgs::Pose>* torso_points,tf::Transform* tf_gripper_pose,
												tf::Transform* tf_base_pose)
{
	geometry_msgs::PoseArray trajectory_pose_array;
	trajectory_pose_array.header.stamp = ros::Time::now();//ros::Time(10.0);
	trajectory_pose_array.header.frame_id = "map";
	geometry_msgs::Pose current_gripper_pose;
	tf::poseTFToMsg(*tf_gripper_pose,current_gripper_pose);
	geometry_msgs::Pose current_base_pose;
	tf::poseTFToMsg(*tf_base_pose,current_base_pose);
	double current_time = 0;
	double dt = 0.0003;//1.0/nrPoints;

	Eigen::VectorXf eigen_current_pose(14);
	eigen_current_pose << tf_gripper_pose->getOrigin().getX(),tf_gripper_pose->getOrigin().getY(), tf_gripper_pose->getOrigin().getZ(),
						   tf_gripper_pose->getRotation().x(),tf_gripper_pose->getRotation().y(),tf_gripper_pose->getRotation().z(),tf_gripper_pose->getRotation().w(),
						   tf_base_pose->getOrigin().getX(),tf_base_pose->getOrigin().getY(), tf_base_pose->getOrigin().getZ(),
						   tf_base_pose->getRotation().x(),tf_base_pose->getRotation().y(),tf_base_pose->getRotation().z(),tf_base_pose->getRotation().w();
	Eigen::VectorXf eigen_current_speed(14);
	eigen_current_speed << 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;

	ROS_INFO("First gripper_pose: (%g,%g,%g)",eigen_current_pose(0),eigen_current_pose(1),eigen_current_pose(2));
	for(int n=0;n<nrPoints;n++)
	{
		current_time += dt;
		integrateModel(current_time, 1.0*dt,&eigen_current_pose,&eigen_current_speed);


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


		gripper_points->push_back(current_gripper_pose);
		torso_points->push_back(current_base_pose);
		trajectory_pose_array.poses.push_back(current_gripper_pose);
		trajectory_pose_array.poses.push_back(current_base_pose);
		trajectory_pose_array.poses.push_back(current_gripper_pose);
	}

	ros::Duration(1.0).sleep();
    Traj_pub_.publish(trajectory_pose_array);

    geometry_msgs::PoseArray poseArray_transformed;
    Trajectory2PoseArray(&trajectory_pose_array, &poseArray_transformed);
    Traj_pub2_.publish(poseArray_transformed);
    ROS_INFO("Trajectory precomputed");
}



template <typename T>
bool GaussianMixtureModel::parseVector(std::ifstream & is, std::vector<T> & pts, const std::string & name)
{
    std::string line;
    getline(is, line);
    if(!is.good()) {
        ROS_ERROR("getline error for %s pts", name.c_str());
        return false;
    }

    std::stringstream ss(line);
    std::string nameStr;
    ss >> nameStr;
    nameStr = string_tools::trim(nameStr);
    if(string_tools::startsWith(nameStr, "\"")) {
        nameStr = nameStr.substr(1);
        if(string_tools::endsWith(nameStr, "\"")) {
            nameStr = nameStr.substr(0, nameStr.size() - 1);
        } else {
            // keep first word, skip rest until "
            while(ss.good()) {
                std::string discard;
                ss >> discard;
                discard = string_tools::trim(discard);
                if(string_tools::endsWith(discard, "\""))
                    break;
            }
        }
    }
    if(!string_tools::startsWith(nameStr, name)) {
        ROS_ERROR("parseVector name mismatch: %s - %s", name.c_str(), nameStr.c_str());
        return false;
    }
    while(ss.good()) {
        T data(0);
        if(ss >> data)
            pts.push_back(data);
    }
    return true;
}






bool GaussianMixtureModel::loadFromFile(std::string & filename)
{
	ROS_INFO("Loading GMM from file...");
	std::ifstream is(filename.c_str());
    if(!is.good()) {
			ROS_INFO("GMM loading Error");
			return false;
		}

		ROS_INFO("GMM loaded");

    std::string line;
    // skip empty lines until we're at a good one
    do {
        getline(is, line);
        if(!is.good()) {
            ROS_ERROR("Data error");
            return false;
        }
        line = string_tools::trim(line);
    } while(line.empty());

    if(!string_tools::startsWith(line, "\"")) {
        ROS_ERROR("Name start error in %s", line.c_str());
        return false;
    }
    if(!string_tools::endsWith(line, "\"")) {
        ROS_ERROR("Name end error in %s", line.c_str());
        return false;
    }
    std::string name = line.substr(1, line.size() - 2);
    ROS_INFO("Data for  %s - parsing data", name.c_str());

    // Read in Priors
    std::vector<double> Priors;
    if(!parseVector(is, Priors, "p")) return false;
    _Priors = Priors;

    // Read in Mu
    line.clear();
    do {
        getline(is, line);
        if(!is.good()) {
            ROS_ERROR("Traj name error");
            return false;
        }
        line = string_tools::trim(line);
    } while(line.empty());

    if(!string_tools::startsWith(line, "\"")) {
        ROS_ERROR("Traj name start error in %s", line.c_str());
        return false;
    }
    if(!string_tools::endsWith(line, "\"")) {
        ROS_ERROR("Traj name end error in %s", line.c_str());
        return false;
    }
    name = line.substr(1, line.size() - 2);
    ROS_INFO("Data for  %s - parsing data", name.c_str());

    std::vector<double> Mu_t;
    if(!parseVector(is, Mu_t, "t")) return false;
    std::vector<double> Mu_x;
    if(!parseVector(is, Mu_x, "x")) return false;
    std::vector<double> Mu_y;
    if(!parseVector(is, Mu_y, "y")) return false;
    if(!(Mu_x.size() == Mu_y.size())) { ROS_ERROR("Size mismatch for yPts"); return false; }
    std::vector<double> Mu_z;
    if(!parseVector(is, Mu_z, "z")) return false;
    if(!(Mu_x.size() == Mu_z.size())) { ROS_ERROR("Size mismatch for zPts"); return false; }
    std::vector<double> Mu_qx;
    if(!parseVector(is, Mu_qx, "qx")) return false;
    if(!(Mu_x.size() == Mu_qx.size())) { ROS_ERROR("Size mismatch for qxPts"); return false; }
    std::vector<double> Mu_qy;
    if(!parseVector(is, Mu_qy, "qy")) return false;
    if(!(Mu_x.size() == Mu_qy.size())) { ROS_ERROR("Size mismatch for qyPts"); return false; }
    std::vector<double> Mu_qz;
    if(!parseVector(is, Mu_qz, "qz")) return false;
    if(!(Mu_x.size() == Mu_qz.size())) { ROS_ERROR("Size mismatch for qzPts"); return false; }
    std::vector<double> Mu_qw;
    if(!parseVector(is, Mu_qw, "qw")) return false;
    if(!(Mu_x.size() == Mu_qw.size())) { ROS_ERROR("Size mismatch for qwPts"); return false; }
    std::vector<double> Mub_x;
    if(!parseVector(is, Mub_x, "x")) return false;
    std::vector<double> Mub_y;
    if(!parseVector(is, Mub_y, "y")) return false;
    if(!(Mub_x.size() == Mub_y.size())) { ROS_ERROR("Size mismatch for yPts"); return false; }
    std::vector<double> Mub_z;
    if(!parseVector(is, Mub_z, "z")) return false;
    if(!(Mub_x.size() == Mub_z.size())) { ROS_ERROR("Size mismatch for zPts"); return false; }
    std::vector<double> Mub_qx;
    if(!parseVector(is, Mub_qx, "qx")) return false;
    if(!(Mub_x.size() == Mub_qx.size())) { ROS_ERROR("Size mismatch for qxPts"); return false; }
    std::vector<double> Mub_qy;
    if(!parseVector(is, Mub_qy, "qy")) return false;
    if(!(Mub_x.size() == Mub_qy.size())) { ROS_ERROR("Size mismatch for qyPts"); return false; }
    std::vector<double> Mub_qz;
    if(!parseVector(is, Mub_qz, "qz")) return false;
    if(!(Mub_x.size() == Mub_qz.size())) { ROS_ERROR("Size mismatch for qzPts"); return false; }
    std::vector<double> Mub_qw;
    if(!parseVector(is, Mub_qw, "qw")) return false;
    if(!(Mub_x.size() == Mub_qw.size())) { ROS_ERROR("Size mismatch for qwPts"); return false; }

    _Mu.clear();
    for(int i=0;i<_nr_modes;i++)
	{
		std::vector<double> Mu_i;
		Mu_i.push_back(Mu_t[i]);
		Mu_i.push_back(Mu_x[i]);
		Mu_i.push_back(Mu_y[i]);
		Mu_i.push_back(Mu_z[i]);
		Mu_i.push_back(Mu_qx[i]);
		Mu_i.push_back(Mu_qy[i]);
		Mu_i.push_back(Mu_qz[i]);
		Mu_i.push_back(Mu_qw[i]);
		Mu_i.push_back(Mub_x[i]);
		Mu_i.push_back(Mub_y[i]);
		Mu_i.push_back(Mub_z[i]);
		Mu_i.push_back(Mub_qx[i]);
		Mu_i.push_back(Mub_qy[i]);
		Mu_i.push_back(Mub_qz[i]);
		Mu_i.push_back(Mub_qw[i]);

		_Mu.push_back(Mu_i);
		Eigen::VectorXf Mu_i_eigen(15);
		Mu_i_eigen << Mu_i[0],Mu_i[1],Mu_i[2],Mu_i[3],Mu_i[4],Mu_i[5],Mu_i[6],Mu_i[7],
					  Mu_i[8],Mu_i[9],Mu_i[10],Mu_i[11],Mu_i[12],Mu_i[13],Mu_i[14];
		_MuEigen.push_back(Mu_i_eigen);
	}

    // Read in Sigma
    _Sigma.clear();
    for(int j=0;j<_nr_modes;j++)
    {
    	line.clear();
	    do {
	        getline(is, line);
	        if(!is.good()) {
	            ROS_ERROR("Traj name error");
	            return false;
	        }
	        line = string_tools::trim(line);
	    } while(line.empty());

	    if(!string_tools::startsWith(line, "\"")) {
	        ROS_ERROR("Traj name start error in %s", line.c_str());
	        return false;
	    }
	    if(!string_tools::endsWith(line, "\"")) {
	        ROS_ERROR("Traj name end error in %s", line.c_str());
	        return false;
	    }
	    name = line.substr(1, line.size() - 2);
	    ROS_INFO("Data for  %s - parsing data", name.c_str());

	    std::vector<double> Sigma_i0;
	    if(!parseVector(is, Sigma_i0, "s")) return false;
	     std::vector<double> Sigma_i1;
	    if(!parseVector(is, Sigma_i1, "s")) return false;
	     std::vector<double> Sigma_i2;
	    if(!parseVector(is, Sigma_i2, "s")) return false;
	     std::vector<double> Sigma_i3;
	    if(!parseVector(is, Sigma_i3, "s")) return false;
	     std::vector<double> Sigma_i4;
	    if(!parseVector(is, Sigma_i4, "s")) return false;
	     std::vector<double> Sigma_i5;
	    if(!parseVector(is, Sigma_i5, "s")) return false;
	     std::vector<double> Sigma_i6;
	    if(!parseVector(is, Sigma_i6, "s")) return false;
	     std::vector<double> Sigma_i7;
	    if(!parseVector(is, Sigma_i7, "s")) return false;
	     std::vector<double> Sigma_i8;
	    if(!parseVector(is, Sigma_i8, "s")) return false;
	     std::vector<double> Sigma_i9;
	    if(!parseVector(is, Sigma_i9, "s")) return false;
	     std::vector<double> Sigma_i10;
	    if(!parseVector(is, Sigma_i10, "s")) return false;
	     std::vector<double> Sigma_i11;
	    if(!parseVector(is, Sigma_i11, "s")) return false;
	     std::vector<double> Sigma_i12;
	    if(!parseVector(is, Sigma_i12, "s")) return false;
	     std::vector<double> Sigma_i13;
	    if(!parseVector(is, Sigma_i13, "s")) return false;
	     std::vector<double> Sigma_i14;
	    if(!parseVector(is, Sigma_i14, "s")) return false;

	    Eigen::MatrixXf Sigma_i(15,15);
	    for(int i=0;i<15;i++)
		{
			Sigma_i(i,0) = Sigma_i0[i];
			Sigma_i(i,1) = Sigma_i1[i];
			Sigma_i(i,2) = Sigma_i2[i];
			Sigma_i(i,3) = Sigma_i3[i];
			Sigma_i(i,4) = Sigma_i4[i];
			Sigma_i(i,5) = Sigma_i5[i];
			Sigma_i(i,6) = Sigma_i6[i];
			Sigma_i(i,7) = Sigma_i7[i];
			Sigma_i(i,8) = Sigma_i8[i];
			Sigma_i(i,9) = Sigma_i9[i];
			Sigma_i(i,10) = Sigma_i10[i];
			Sigma_i(i,11) = Sigma_i11[i];
			Sigma_i(i,12) = Sigma_i12[i];
			Sigma_i(i,13) = Sigma_i13[i];
			Sigma_i(i,14) = Sigma_i14[i];
		}
		_Sigma.push_back(Sigma_i);
    }


    // set goal state
    _goalState.setOrigin(tf::Vector3(_Mu[_nr_modes-1][1],_Mu[_nr_modes-1][2],_Mu[_nr_modes-1][3]));
    _goalState.setRotation(tf::Quaternion(_Mu[_nr_modes-1][4],_Mu[_nr_modes-1][5],_Mu[_nr_modes-1][6],_Mu[_nr_modes-1][7]));
    _goalState.stamp_ = ros::Time(_Mu[_nr_modes-1][0]);

    // set related object parameters
    line.clear();
    do {
        getline(is, line);
        if(!is.good()) {
            ROS_ERROR("Traj name error");
            return false;
        }
        line = string_tools::trim(line);
    } while(line.empty());

    if(!string_tools::startsWith(line, "\"")) {
        ROS_ERROR("Traj name start error in %s", line.c_str());
        return false;
    }
    if(!string_tools::endsWith(line, "\"")) {
        ROS_ERROR("Traj name end error in %s", line.c_str());
        return false;
    }
    name = line.substr(1, line.size() - 2);
    ROS_INFO("Data for  %s - parsing data", name.c_str());

    std::vector<double> object_pose;
    if(!parseVector(is, object_pose, "pose")) return false;
    _related_object_pose.setOrigin(tf::Vector3(object_pose[0],object_pose[1],object_pose[2]));
    _related_object_pose.setRotation(tf::Quaternion(object_pose[3],object_pose[4],object_pose[5],object_pose[6]));
    std::vector<double> object_grasp;
    if(!parseVector(is, object_grasp, "grasp")) return false;
    _related_object_grasp_pose.setOrigin(tf::Vector3(object_grasp[0],object_grasp[1],object_grasp[2]));
    _related_object_grasp_pose.setRotation(tf::Quaternion(object_grasp[3],object_grasp[4],object_grasp[5],object_grasp[6]));
    std::vector<char> object_name;
    ROS_INFO("Test object read1");
    if(!parseVector(is, object_name, "name")) return false;
    ROS_INFO("Test object read2");
	_related_object_name = object_name[0];

    ROS_INFO("Successfully loaded GMM!");
    return true;
}




void GaussianMixtureModel::publishMuMarker()
{
	ROS_INFO("Publishing GMM markers");
	double marker_size = 0.05;
	visualization_msgs::MarkerArray ma;

	for(int i=0;i<_nr_modes;i++)
	{
		visualization_msgs::Marker gmm_marker_gripper;
	    gmm_marker_gripper.header.frame_id = "/map";
	    gmm_marker_gripper.ns = "Gripper_Mu";
	    gmm_marker_gripper.id = i;
	    gmm_marker_gripper.type = visualization_msgs::Marker::SPHERE;
	    gmm_marker_gripper.action = visualization_msgs::Marker::ADD;
	    gmm_marker_gripper.pose.orientation.w = 1.0;
	    gmm_marker_gripper.scale.x = marker_size;
	    gmm_marker_gripper.scale.y = marker_size;
	    gmm_marker_gripper.scale.z = marker_size;
	    gmm_marker_gripper.color.b = 1.0 - ((double)i+1)/(double)_nr_modes;
	    gmm_marker_gripper.color.r = ((double)i+1)/(double)_nr_modes;
	    gmm_marker_gripper.color.a = 1.;
		geometry_msgs::Pose poseMsg;
		poseMsg.position.x = _MuEigen[i](1);
		poseMsg.position.y = _MuEigen[i](2);
		poseMsg.position.z = _MuEigen[i](3);
	    gmm_marker_gripper.pose = poseMsg;

	    visualization_msgs::Marker gmm_marker_base;
	    gmm_marker_base.header.frame_id = "/map";
	    gmm_marker_base.ns = "Base_Mu";
	    gmm_marker_base.id = 100 + i;
	    gmm_marker_base.type = visualization_msgs::Marker::CUBE;
	    gmm_marker_base.action = visualization_msgs::Marker::ADD;
	    gmm_marker_base.pose.orientation.w = 1.0;
	    gmm_marker_base.scale.x = marker_size;
	    gmm_marker_base.scale.y = marker_size;
	    gmm_marker_base.scale.z = marker_size;
	    gmm_marker_base.color.b = 1.0 - ((double)i+1)/(double)_nr_modes;
	    gmm_marker_base.color.r = ((double)i+1)/(double)_nr_modes;
	    gmm_marker_base.color.a = 1.;
	    poseMsg.position.x = _MuEigen[i](8);
		poseMsg.position.y = _MuEigen[i](9);
		poseMsg.position.z = _MuEigen[i](10);
	    gmm_marker_base.pose = poseMsg;

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
	    visualization_msgs::Marker mark;
        mark.header.frame_id = "/map";
        mark.ns = "Base_Mu_Frame";
        mark.id = 300+i;
        mark.type = visualization_msgs::Marker::LINE_LIST;
        mark.action = visualization_msgs::Marker::ADD;
        mark.pose.orientation.w = 1.0;
        mark.scale.x = 0.008;
        mark.color = blue;
        blue.a = 1.0;
        const double poseCoordsLength = 0.15;
        geometry_msgs::Point ptOrig;
        Eigen::Isometry3d basePose;
        basePose.translation() = Eigen::Vector3d(_MuEigen[i](8),_MuEigen[i](9),_MuEigen[i](10));
        Eigen::Quaterniond Q = Eigen::Quaterniond(_MuEigen[i](14),_MuEigen[i](11),_MuEigen[i](12),_MuEigen[i](13));
        basePose.linear() = Q.matrix();
        ptOrig.x = basePose.translation().x();
        ptOrig.y = basePose.translation().y();
        ptOrig.z = basePose.translation().z();
        //ROS_INFO_STREAM("traj pt at " << pt);
        Eigen::Vector3d xCoord(poseCoordsLength, 0.0, 0.0);
        geometry_msgs::Point ptX;
        tf::pointEigenToMsg(basePose * xCoord, ptX);
        mark.points.push_back(ptOrig);
        mark.points.push_back(ptX);
        mark.colors.push_back(red);
        mark.colors.push_back(red);
        Eigen::Vector3d yCoord(0.0, poseCoordsLength, 0.0);
        geometry_msgs::Point ptY;
        tf::pointEigenToMsg(basePose * yCoord, ptY);
        mark.points.push_back(ptOrig);
        mark.points.push_back(ptY);
        mark.colors.push_back(green);
        mark.colors.push_back(green);
        Eigen::Vector3d zCoord(0.0, 0.0, poseCoordsLength);
        geometry_msgs::Point ptZ;
        tf::pointEigenToMsg(basePose * zCoord, ptZ);
        mark.points.push_back(ptOrig);
        mark.points.push_back(ptZ);
        mark.colors.push_back(blue);
        mark.colors.push_back(blue);
        //Gripper
        visualization_msgs::Marker mark_gripper_frame;
        mark_gripper_frame.header.frame_id = "/map";
        mark_gripper_frame.ns = "Gripper_Mu_Frame";
        mark_gripper_frame.id = 200+i;
        mark_gripper_frame.type = visualization_msgs::Marker::LINE_LIST;
        mark_gripper_frame.action = visualization_msgs::Marker::ADD;
        mark_gripper_frame.pose.orientation.w = 1.0;
        mark_gripper_frame.scale.x = 0.008;
        mark_gripper_frame.color = blue;
        geometry_msgs::Point ptOrig2;
        Eigen::Isometry3d gripperPose;
        gripperPose.translation() = Eigen::Vector3d(_MuEigen[i](1),_MuEigen[i](2),_MuEigen[i](3));
        Eigen::Quaterniond Q2 = Eigen::Quaterniond(_MuEigen[i](7),_MuEigen[i](4),_MuEigen[i](5),_MuEigen[i](6));
        gripperPose.linear() = Q2.matrix();
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


	    ma.markers.push_back(gmm_marker_base);
	    ma.markers.push_back(gmm_marker_gripper);
	    ma.markers.push_back(mark);
	    ma.markers.push_back(mark_gripper_frame);
	}

    ros::Duration(1.0).sleep();
    Mu_pub_.publish(ma);
}
