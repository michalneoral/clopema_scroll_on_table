#include "ScrollGarmentForceCB.h"

ScrollGarmentForceCB::ScrollGarmentForceCB(){
	init_forces_ = Eigen::Vector3d(0,0,0);
	init_torques_ = Eigen::Vector3d(0,0,0);
	ready_ = false;
}

void ScrollGarmentForceCB::setInitialize(){
	mutex_wrench_.lock();
	init_forces_ = forces_;
	init_torques_ = torques_;
	mutex_wrench_.unlock();
	computeRotateForce();
}

void ScrollGarmentForceCB::cb_force(const geometry_msgs::WrenchStamped& msg) {
	mutex_wrench_.lock();
	tf::vectorMsgToEigen(msg.wrench.force, forces_);
	tf::vectorMsgToEigen(msg.wrench.torque, torques_);
	mutex_wrench_.unlock();

	if (!ready_){
		ready_=true;
		setInitialize();
	}
	computeRotateForce();
}

void ScrollGarmentForceCB::startSub(std::string name){
	if (name.compare("r1")==0){
		force_topic_name_ = "/r1_force_data_filtered";
		ee_name_ = "/r1_ee";
		sensor_name_ = "/r1_force_sensor";		
	} 
	else if(name.compare("r2")==0){
		force_topic_name_ = "/r2_force_data_filtered";
		ee_name_ = "/r2_ee";
		sensor_name_ = "/r2_force_sensor";		
	} 
	else
	{
		ROS_ERROR("Gripper with setted name doesn't exist!");
	}
	sub_force_ = node_.subscribe(force_topic_name_, 1, &ScrollGarmentForceCB::cb_force, this);
	setLenght();
}

/* @TODO */
void ScrollGarmentForceCB::computeRotateForce(){
	Eigen::Affine3d rot;
	Eigen::Vector3d comForcesVector, comTorquesVector, erForces, erTorques;

	mutex_forces_.lock();
	Eigen::Vector3d prepareForces = (forces_ - init_forces_);
	erForces = forces_;
	mutex_forces_.unlock();
	mutex_torques_.lock();
	Eigen::Vector3d prepareTorques = (torques_ - init_torques_);
	erTorques = torques_;
	mutex_torques_.unlock();

	char buffer [1023];

	double maxForce = 0;
	for (int i=0; i<3; i++){
		if (maxForce < std::abs(erForces[i])) maxForce = std::abs(erForces[i]);
	}

	double maxTorque = 0;
	for (int i=0; i<3; i++){
		if (maxTorque < std::abs(erTorques[i])) maxTorque = std::abs(erTorques[i]);
	}

	if (maxForce >= ABSOLUTE_MAX_FORCE_RAW || maxTorque >= ABSOLUTE_MAX_TORQUE_RAW){
		sprintf(buffer, "The value of force or torque (%.1f N, %.1f Nm) is too much high!", maxForce, maxTorque);
		ROS_WARN_STREAM(buffer);
	}


	Eigen::MatrixXd mat(3,3);
	
	mutex_rotateMatrix_.lock();
	rot = rotateMatrix_;
	mutex_rotateMatrix_.unlock();
	
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			mat(i,j)= rot(i,j);
		}
	}

	comForcesVector = prepareForces.transpose() * mat;
	comTorquesVector = (prepareTorques.transpose() * mat) / lenght_;		
	computedForce_ = comForcesVector(2);
	if(computedForce_ > ABSOLUTE_MAX_FORCE){
		sprintf(buffer, "The value of force (%.1f N) is too much high!", computedForce_);
		ROS_WARN_STREAM(buffer);
	}

	// std::cout << ee_name_ << ": " << computedForce_ << std::endl;

	 // force_info_stream(prepareForces, prepareTorques);
	 // force_info_stream(comForcesVector, comTorquesVector);
}

bool ScrollGarmentForceCB::isForceOk(double force){
	
	if (std::abs(computedForce_) >= ABSOLUTE_MAX_FORCE)
	{
		return false;
	}
	double proc;
	proc = std::abs( (force - std::abs(computedForce_)) / force);
	
	if (proc > RELATIVE_MAX_FORCE){
		return false;
	}
	return true;
}

double ScrollGarmentForceCB::getForce(){
	double rForce;
	mutex_computedForce_.lock();
	rForce = computedForce_;
	mutex_computedForce_.unlock();
	return rForce;
}

void ScrollGarmentForceCB::force_info_stream(Eigen::Vector3d force, Eigen::Vector3d torque) {
	char buffer [1023];
	sprintf(buffer, "%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f", force(0), force(1), force(2), torque(0), torque(1), torque(2));
	ROS_INFO_STREAM(buffer);
}

void ScrollGarmentForceCB::setRotMat(Eigen::Affine3d rot){
	rot(0,3) = 0;
	rot(1,3) = 0;
	rot(2,3) = 0;	
	rotateMatrix_ = rot;
	// std::cout << std::endl << rot.matrix() << std::endl << std::endl;	
}

void ScrollGarmentForceCB::setLenght(){
	ros::Rate rate(100.0);
	geometry_msgs::Pose pos;
	tf::TransformListener listener;
	tf::StampedTransform transform;

	while (ros::ok()){
		try{
			ros::Time latest_transform_time;
			listener.getLatestCommonTime(sensor_name_, ee_name_, latest_transform_time, NULL);
			listener.waitForTransform(sensor_name_, ee_name_, latest_transform_time, ros::Duration(0.1) );
			listener.lookupTransform(sensor_name_, ee_name_, latest_transform_time, transform);
			
			lenght_ = std::abs(transform.getOrigin().z());
			break;
		}
		catch (tf::TransformException ex){
			ros::Duration(0.05).sleep();
		}
		rate.sleep();
	}
}

void ScrollGarmentForceCB::showForces(){
	char buffer [1023];
	mutex_forces_.lock();
	mutex_torques_.lock();
	mutex_computedForce_.lock();
	sprintf(buffer, "%7.2f -> \t%7.2f\t%7.2f\t%7.2f\t%7.2f\t%7.2f", computedForce_, forces_(0), forces_(1), forces_(2), torques_(0), torques_(1), torques_(2));
	mutex_computedForce_.unlock();
	mutex_torques_.unlock();
	mutex_forces_.unlock();
	
	if(std::abs(computedForce_) >= ABSOLUTE_MAX_FORCE){
		std::cout << "\033[1;39mForce " << ee_name_ << buffer << "\033[0m"<< std::endl;
	} else {
		std::cout << "\033[1;40mForce " << ee_name_ << buffer << "\033[0m"<< std::endl;
	}

}