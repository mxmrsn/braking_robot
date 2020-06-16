#include <iostream>

// Eigen headers
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ati_ft_sensor/force_data.h"

//ROS headers
#include <geometry_msgs/Vector3.h>
#include "ros/ros.h"

// Misc. headers
#include <cmath>

enum robot_states
{
  NOT_BRAKING_S,
  BRAKING_S,
  REV_BRAKING_S
};

// GLOBAL VARIABLES NEEDED FOR FORWARD KINEMATICS -------------------------------
bool in_out_workspace = false;
robot_states robot_state = NOT_BRAKING_S; // set robot state as idle on startup
Eigen::Vector2d Force_vector_current;
Eigen::Vector2d Force_unit_vector;
Eigen::Vector2d EndEffector_to_ellipse_center;
Eigen::Vector2d FT_reading_current;
Eigen::Matrix4d FT_reading_tracked;
float Force_mag;



// BASIC MATH FUNCTION DEFINITIONS ----------------------------------------------
float ellipse_check_point(float h, float k, float a, float b, float x, float y)
{
	//checking the equation of ellipse with the given point
	// (h,k): ellipse center, a: major axis, b: minor axis
	float p = (pow((x - h),2) / pow(a,2))
				+ (pow((y - k), 2) / pow(b, 2));

	return p;
}

void update_Force_vector()
{

	FT_reading_tracked(0,0) = FT_reading_tracked(1,0);
	FT_reading_tracked(0,1) = FT_reading_tracked(1,1);

	FT_reading_tracked(1,0) = FT_reading_current(0);
	FT_reading_tracked(1,1) = FT_reading_current(1);

	Force_vector_current(0) = FT_reading_tracked(0,0) - FT_reading_tracked(1,0);
	Force_vector_current(1) = FT_reading_tracked(0,1) - FT_reading_tracked(1,1);

	Force_mag = Force_vector_current.norm();
	Force_unit_vector = Force_vector_current/Force_vector_current.norm();
}

// MESSAGE CALLBACK FUNCTION DEFINITIONS ----------------------------------------
geometry_msgs::Vector3 tempMsg;
void position_callback(const geometry_msgs::Vector3 msg)
{
	tempMsg = msg;

	// ellipse workspace parameters
	float h = 0, k = 100, a = 26, b = 22.5;

	// determine in/out of workspace
	// in_out_workspace = true if check point is OUT of workspace
	// in_out_workspace = false if check point is IN workspace

	if (ellipse_check_point(h,k,a,b,tempMsg.x,tempMsg.y) >= 1)
	{
		in_out_workspace = true;
	}
	else
	{
		in_out_workspace = false;
	}


}

ati_ft_sensor::force_data forceMsg; 
void FT_sensor_callback(const ati_ft_sensor::force_data msg)
{
	forceMsg = msg;

	FT_reading_current(0) = forceMsg.Fx;
	FT_reading_current(1) = forceMsg.Fy;

}


//find_direction over larger interval than recieving messages (duration)


//compare vector to normal of ellipse at given position OR to center of ellipse



int main(int argc, char *argv[])
{
/********************************************************************************
								INITIALIZE ROS NODE
*********************************************************************************/
	ros::init(argc, argv, "braking_robot");
	ros::NodeHandle nh;
	// do setup here 


/********************************************************************************
				SET UP PUBLSIHERS, SUBSRIBERS, SERVICES, & CLIENTS
*********************************************************************************/
	// setup logging using rosparam server
	// bool log_data_ON;
	// ros::param::get("log_data_ON",log_data_ON);

	// subsribers
	ros::Subscriber position_sub = nh.subscribe("Position", 1, position_callback);
	ros::Subscriber FT_sensor_sub = nh.subscribe("force_data_filtered", 1, FT_sensor_callback)	;
	// option to setup subscriber to get status of board

	// publishers
	ros::Publisher dac_pub = nh.advertise<geometry_msgs::Vector3>("MCB1/dac_voltages",1);
	// setup publishers for enable_ros_control and enable_all_motors
	//ros::Publisher enable_ros_control_pub = nh.advertise<std_msgs::Bool>("MCB1/enable_ros_control",1);

	ros::spinOnce();

	double controller_rate = 100.0; //Hz
	double loop_rate = 1.0/controller_rate; //s
	ros::Rate r(controller_rate);

	geometry_msgs::Vector3 mot_msg;

	bool justEnteredBraking = false;
	bool justEnteredNotBraking = true;
	bool justEnteredRevBraking = false;
	bool finishedWaitForMove = false;
	bool using_FT_sensor = false;
	bool update_FT_sensor_vector = false;
	bool force_criteria_met = false;

	ros::Duration time_on(4.0); // time on of 4 seconds
	ros::Duration time_rev_current(0.05); // reverse current for 50 milliseconds
	ros::Duration time_move(5.0); // time move of 5 seconds
	ros::Duration time_FTsensor_update(0.25); //update FT sensor vector over 0.25 seconds

	ros::Time start_time_brake_on;
	ros::Time start_time_reverse_current;
	ros::Time start_time_move;
	ros::Time start_time_update_FT_vector;

	int count = 0;
	while(ros::ok())
	{

		if (using_FT_sensor == 1)
		{
			if (robot_state == NOT_BRAKING_S)
			{
				std::cout<<"Not Braking!"<<std::endl;

				if (in_out_workspace == 1)
				{
					robot_state = BRAKING_S;
				}
				else 
				{
					// std::cout<<in_out_workspace<<std::endl;
					mot_msg.x = 0.0;
					mot_msg.y = 0.0;
					mot_msg.z = 0.0;
				}
			}
			else if (robot_state == BRAKING_S)
			{
				std::cout<<"Braking!"<<std::endl;
				
					//add criteria for when braking
					if(force_criteria_met == 0)
					{
						// set mot_msg
						mot_msg.x = 5.0;
						mot_msg.y = 5.0;
						mot_msg.z = 0.0;	
					} else
					{
						justEnteredRevBraking = true;
						robot_state = REV_BRAKING_S;
					}
				}
			}
			else if (robot_state == REV_BRAKING_S)
			{
				std::cout<<"Reverse!!"<<std::endl;
				if (justEnteredRevBraking == true)
				{
					// start timer
					start_time_reverse_current = ros::Time::now();
					justEnteredRevBraking = false;
				}
				else
				{
					if(ros::Time::now() - start_time_reverse_current < time_rev_current)
					{
							// set mot_msg
							mot_msg.x = -5.0;
							mot_msg.y = -5.0;
							mot_msg.z = 0.0;	

					} else
					{
						robot_state = NOT_BRAKING_S;
					}
				}
			}
		else
		{
			if (robot_state == NOT_BRAKING_S)
			{
				std::cout<<"Not Braking!"<<std::endl;
				if (justEnteredNotBraking == 1) // Coming off of boundary
				{
					start_time_move = ros::Time::now();
					justEnteredNotBraking = false;
				}
				else // Waiting to move into boundary
				{
					if(ros::Time::now() - start_time_move < time_move)
					{
						std::cout<<"Waiting!"<<std::endl;
						mot_msg.x = 0.0;
						mot_msg.y = 0.0;
						mot_msg.z = 0.0;
					}
					else
					{
						finishedWaitForMove = true;
					}
				}

				if (finishedWaitForMove == 1)
				{
					// finishedWaitForMove = false;
					if (in_out_workspace == 1)
					{
						robot_state = BRAKING_S;
						justEnteredBraking = true;
					}
					else 
					{
						mot_msg.x = 0.0;
						mot_msg.y = 0.0;
						mot_msg.z = 0.0;
					}
				}
			}
			else if (robot_state == BRAKING_S)
			{
				std::cout<<"Braking!"<<std::endl;
				if (justEnteredBraking == true)
				{
					// start timer
					 start_time_brake_on = ros::Time::now();
					 justEnteredBraking = false;
				} 
				else // continue braking
				{
					if(ros::Time::now() - start_time_brake_on < time_on)
					{
							// set mot_msg
							mot_msg.x = 5.0;
							mot_msg.y = 5.0;
							mot_msg.z = 0.0;	
					} else
					{
						justEnteredRevBraking = true;
						robot_state = REV_BRAKING_S;
					}
				}
			}
			else if (robot_state == REV_BRAKING_S)
			{
				std::cout<<"Reverse!!"<<std::endl;
				if (justEnteredRevBraking == true)
				{
					// start timer
					start_time_reverse_current = ros::Time::now();
					justEnteredRevBraking = false;
				}
				else
				{
					if(ros::Time::now() - start_time_reverse_current < time_rev_current)
					{
							// set mot_msg
							mot_msg.x = -5.0;
							mot_msg.y = -5.0;
							mot_msg.z = 0.0;	

					} else
					{
						justEnteredNotBraking = true;
						finishedWaitForMove = false;
						robot_state = NOT_BRAKING_S;
					}
				}
			}
		}
		

		// interval
		// -> if interval elapses, oldForceSensor = lastForceSensor; curForceSensor = cure

				if (update_FT_sensor_vector == true)
				{
					// start timer
					start_time_update_FT_vector = ros::Time::now();
					update_FT_sensor_vector = false;
					update_Force_vector();
				}
				else
				{
					if(ros::Time::now() - start_time_reverse_current < time_rev_current)
					{
					}
					else
					{
						update_FT_sensor_vector = true;
					}
				}
		// Def vectors and magnitudes

		//DELAY AT BEGINNING DOES NOT FIX BRAKES ENGAGING AT BEGINNING
		// count ++;
		// // first loop
		// if (count == 1)
		// {
		// 	ros::Time start_time_loop = ros::Time::now();
		// 	ros::Duration loop_begin_delay(0.2); //delay at beginning of loop so brakes do not immediately engage
		// 	while(ros::Time::now() - start_time_loop < loop_begin_delay)
		// 	{
		// 		std::cout << "Delay 0.2 sec" << std::endl;
		// 	}
		// }	

		// //------------------meaure duration of loop between here --------------
		// 	ros::Time start_ = ros::Time::now();

		// if (in_out_workspace == 1)
		// {
			
		// 	//if end effector goes out of workspace, engage brakes for 5 seconds
		// 	ros::Time start_time_brake_on = ros::Time::now();
		// 	ros::Duration time_on(4.0); // time on of 5 seconds
		// 	while(ros::Time::now() - start_time_brake_on < time_on)
		// 	{
		// 		//std::cout << "Brakes ON" << std::endl;
		// 		mot_msg.x = 5.0;
		// 		mot_msg.y = 5.0;
		// 		mot_msg.z = 0.0;

		// 		dac_pub.publish(mot_msg);	
		// 		ros::spinOnce();
		// 	}


		// 	ros::Time start_time_reverse_current = ros::Time::now();
		// 	ros::Duration time_rev_current(0.05); // reverse current for 10 milliseconds
		// 	while(ros::Time::now() - start_time_reverse_current < time_rev_current)
		// 	{
		// 		//std::cout << "Reverse Current" << std::endl;
		// 		mot_msg.x = -5.0;
		// 		mot_msg.y = -5.0;
		// 		mot_msg.z = 0.0;	

		// 		dac_pub.publish(mot_msg);
		// 		ros::spinOnce();
		// 	}

			
		// 	//after 5 seconds, allow user to move end effector back into workspace for 5 seconds
		// 	ros::Time start_time_move = ros::Time::now();
		// 	ros::Duration time_move(5.0); // time move of 5 seconds
		// 	// for (double time_move = 0; time_move < total_time_on; time_move = time_move + loop_rate)
		// 	while(ros::Time::now() - start_time_move < time_move)
		// 	{
		// 		//std::cout << "Brakes OFF. Move into workspace..." << std::endl;
		// 		mot_msg.x = 0.0;
		// 		mot_msg.y = 0.0;
		// 		mot_msg.z = 0.0;	

		// 		dac_pub.publish(mot_msg);
		// 		ros::spinOnce();
		// 	}			

		// } else
		// {
		// 	mot_msg.x = 0.0;
		// 	mot_msg.y = 0.0;
		// 	mot_msg.z = 0.0;
		// }

		// ros::Time end_ = ros::Time::now();
		// 	double execution_time = (end_ - start_).toSec();
		// 	std::cout << "Execution time (s): " << execution_time << std::endl;
		// 	//----------------------------------------------------------------------

		dac_pub.publish(mot_msg);
		ros::spinOnce();
		if ( !r.sleep() ) std::cout<<"Loop Rate not satisfied!" <<std::endl;
	}
	
	
}
