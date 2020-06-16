#include <iostream>

// Eigen headers
#include <Eigen/Dense>
#include <Eigen/Geometry>

//ROS headers
#include <geometry_msgs/Vector3.h>
#include "ros/ros.h"

// Misc. headers
#include <cmath>

// GLOBAL VARIABLES NEEDED FOR FORWARD KINEMATICS -------------------------------
Eigen::Vector2d joint_angles;

// BASIC MATH FUNCTION DEFINITIONS ----------------------------------------------
Eigen::Vector3d params_5bar()
{
	Eigen::Vector3d LL;
	LL << 26, 75.5, 80; //L0, L1, and L2 link lengths (LL) (mm)
	return LL; 
}

Eigen::Vector3d forward_kin(Eigen::Vector2d joint_angles)
{

	Eigen::Vector3d LL;
	LL << 26, 75.5, 80; //L0, L1, and L2 link lengths (LL) (mm)


	float e = (LL(1)*(sin(joint_angles(0)) - sin(joint_angles(1)))) / 
				(2*LL(0) + LL(1)*cos(joint_angles(1)) - LL(1)*cos(joint_angles(0)));

	float f = (LL(1)*LL(0)*(cos(joint_angles(0)) + cos(joint_angles(1)))) / 
				(2*LL(0) + LL(1)*cos(joint_angles(1)) - LL(1)*cos(joint_angles(0)));

	float d = 1 + pow(e,2);
	float g = 2*(e*f - e*LL(1)*cos(joint_angles(0)) + e*LL(0) - LL(1)*sin(joint_angles(0)));
	float h = pow(f,2) - 2*f*(LL(1)*cos(joint_angles(0)) - LL(0)) - 2*LL(1)*LL(0)*cos(joint_angles(0)) + 
			pow(LL(0),2) + pow(LL(1),2) - pow(LL(2),2);

	float yp = (-g + sqrt(pow(g,2) - 4*d*h)) / (2*d);
	float xp = e*yp + f;

	Eigen::Vector3d position;
	position << xp, yp, 0;

	return position;

}
// MESSAGE CALLBACK FUNCTION DEFINITIONS ----------------------------------------

geometry_msgs::Vector3 tempMsg;
void pot_angle_callback(const geometry_msgs::Vector3 msg)
{
	tempMsg = msg;

	//Raw message from Teensy is pot_angles before being zeroed
	//Must zero pot angles on 0 deg and 50 deg posts by hitting button on pot Teensy
	//Coefficient calculated to correct for potentiometer angle (compared to actual angle
	//measured with FARO arm for each pot)
	Eigen::Vector2d zeroed_pot_angles; 
	zeroed_pot_angles << (2.2651/2.353)*tempMsg.x, (2.268/2.366)*tempMsg.y;
	// zeroed_pot_angles << tempMsg.x, tempMsg.y;

	
	// take zeroed pot angles and convert to robot base angle
	// q1_start = 50 deg (0.8727 rad), q2_start = 0 deg (0 rad)  

	Eigen::Vector2d starting_pot_angles;
	starting_pot_angles << 50*(M_PI/180), 0;

	joint_angles = zeroed_pot_angles + starting_pot_angles;

	//**********NEED TO USE CALIBRATION TO MAKE ANGLES CORRECT

}

bool in_out_workspace = false;
int count = 0;
int main(int argc, char *argv[])
{
/********************************************************************************
								INITIALIZE ROS NODE
*********************************************************************************/
	ros::init(argc, argv, "braking_robot");
	ros::NodeHandle nh;


/********************************************************************************
				SET UP PUBLSIHERS, SUBSRIBERS, SERVICES, & CLIENTS
*********************************************************************************/
	// subsribers
	ros::Subscriber pot_angle_sub = nh.subscribe<geometry_msgs::Vector3>("Pot_angles",1,pot_angle_callback);

	// publishers
	ros::Publisher position_pub = nh.advertise<geometry_msgs::Vector3>("Position",1);

	ros::spinOnce();

	double controller_rate = 100.0; //Hz
	double loop_rate = 1.0/controller_rate;
	ros::Rate r(controller_rate);

	while(ros::ok())
	{
		
		Eigen::Vector3d temp_position;
		
		// if (count == 2)
		// {
		// 	temp_position << 1.0, 0.0, 0.0;
		// 	count = 0;
		// } else
		// {
		// 	temp_position << 0.0, 0.0, 0.0;
		// 	//std::cout<<"Got here!"<<std::endl;
		// }
		// count ++;

		temp_position = forward_kin(joint_angles);

		geometry_msgs::Vector3 msg2pub;
		msg2pub.x = temp_position(0);
		msg2pub.y = temp_position(1);
		msg2pub.z = temp_position(2);
		position_pub.publish(msg2pub);


		// std::cout << "joint_angles: q1 = " << joint_angles(0) << std::endl << " q2: " << joint_angles(1) << std::endl;

		ros::spinOnce();
		if ( !r.sleep() ) std::cout<<"Loop Rate not satisfied!" <<std::endl;
	}
	
	
}
