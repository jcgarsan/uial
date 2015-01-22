/* 
 * Copyright (c) 2014 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * This file has been adapted from setVehicleVelocity.cpp
 * authored by Mario Prats and Javier Perez
 *
 * Contributors:
 *     Juan Carlos García
 */ 

#include <iostream>
#include <stdlib.h>
#include <string.h>
#include "../include/uial/uial.h"

#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <cmath>  

//CIRS scene
#define TOPIC  "/dataNavigator"				
#define pressureThreshold 0.2
#define rangeThreshold 0.6

//shipweck scene
//#define TOPIC  "/dataNavigator_G500RAUVI"		
//#define pressureThreshold	-1.0
//#define rangeThreshold 		 1.2

//DEBUG Flags
#define DEBUG_waypoint_sub	0
#define DEBUG_hand_sub 		0
#define DEBUG_leap_sub 		0

using namespace std;



Uial::Uial()
{
	//initializing values
	p0.x = 0; p0.y = 0; p0.z = 0;
	q0.x = 0; q0.y = 0; q0.z = 0; q0.w = 0;
	initPosition.pose.position 		  = p0;
	initPosition.pose.orientation	  = q0;
	currentPosition.pose.position 	  = p0;
	currentPosition.pose.orientation  = q0;
	previousPosition.pose.position 	  = p0;
	previousPosition.pose.orientation = q0;
	sensorRangeAlarm 	= false;
	sensorPressureAlarm = false;
	handIsOpen 			= false;
	rotationMode 		= false;
	selectWaypoint 		= false;
	robotStopped		= false;
	rightHand			= false;
	moving				= false;
	numWaypoint 		= 1;

	listener = new (tf::TransformListener);

	//publisher and subscriber initialization
	vel_pub_ = nh_.advertise<nav_msgs::Odometry>(TOPIC,1);
	hand_sub_ = nh_.subscribe<sensor_msgs::JointState>("leap_tracker/joint_state_out", 1, &Uial::leapHandCallback, this);
	leap_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("leap_tracker/pose_stamped_out", 1, &Uial::leapCallback, this);
	sensorPressure_sub_ = nh_.subscribe<underwater_sensor_msgs::Pressure>("g500/pressure", 1, &Uial::sensorPressureCallback, this);
	sensorRange_sub_ = nh_.subscribe<sensor_msgs::Range>("uwsim/g500/range", 1, &Uial::sensorRangeCallback, this);
	odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("uwsim/girona500_odom_RAUVI", 1, &Uial::odomCallback, this);
	
	robot=new ARM5Arm(nh_, "uwsim/joint_state", "uwsim/joint_state_command");

}

Uial::~Uial()
{
	//Destructor
}


void Uial::odomCallback(const nav_msgs::Odometry::ConstPtr& odomValue)
{
	int i = 0;

	if ((selectWaypoint) and (!robotStopped) and (numWaypoint < 10))
	{
		if ((abs(odomValue->pose.pose.position.x - waypointsList[numWaypoint-1].pose.position.x) >= 0.1) or \
			(abs(odomValue->pose.pose.position.y - waypointsList[numWaypoint-1].pose.position.y) >= 0.1) or \
			(abs(odomValue->pose.pose.position.z - waypointsList[numWaypoint-1].pose.position.z) >= 0.1))
		{
			waypointsList[numWaypoint].pose = odomValue->pose.pose;
			numWaypoint++;
			selectWaypoint = false;
			cout << "New waypoint #: " << numWaypoint << endl;
			if (DEBUG_waypoint_sub)
			{
				cout << "New waypoint pose: \n" << waypointsList[numWaypoint].pose.position << endl;
			}
		}
		else
		{
			cout << "Waypoint similar. Quite close to the last one." << endl;
		}
	}
	if ((numWaypoint <= 10) and (DEBUG_waypoint_sub))
	{
		cout << "Waypoint list: " << endl;
		for (int i=0; i < numWaypoint; i++)
		{
			cout << " - Waypoint #" << i << ": " << waypointsList[i].pose.position.x << ", "\
				 << waypointsList[i].pose.position.y << ", " << waypointsList[i].pose.position.z << endl;
		}
		selectWaypoint = false;
	}
}


void Uial::sensorPressureCallback(const underwater_sensor_msgs::Pressure::ConstPtr& pressureValue)
{
	if (pressureValue->pressure > pressureThreshold)
		sensorPressureAlarm = true;
	else
		sensorPressureAlarm = false;
}

void Uial::sensorRangeCallback(const sensor_msgs::Range::ConstPtr& rangeValue)
{
	if (rangeValue->range < rangeThreshold)
		sensorRangeAlarm = true;
	else
		sensorRangeAlarm = false;
}

void Uial::leapHandCallback(const sensor_msgs::JointState::ConstPtr& jointstate)
{
	handsDetected = jointstate->position[63];
	
	(jointstate->position[64] == 1   ? rightHand = true : rightHand = false);
	(jointstate->position[65] >= 0.9 ? handIsOpen = false : handIsOpen = true);

	if ((rightHand) and (!handIsOpen))  //The right hand is closed
		selectWaypoint = true;

	if (DEBUG_hand_sub)
	{
		(jointstate->position[65] >= 0.9 ? cout << "hand is closed" << endl : cout << "hand is opened" << endl);

	/*	cout << "index.meta  = " << jointstate->position[15] << " / " << jointstate->position[16] \
								 << " / " << jointstate->position[17] << endl;
		cout << "index.prox  = " << jointstate->position[18] << " / " << jointstate->position[19] \
								 << " / " << jointstate->position[20] << endl;
		cout << "index.mid  = " << jointstate->position[21] << " / " << jointstate->position[22] \
								 << " / " << jointstate->position[23] << endl;
		cout << "index.dist  = " << jointstate->position[24] << " / " << jointstate->position[25] \
								 << " / " << jointstate->position[26] << endl;
		cout << endl;*/

	/*	cout << "thumb.dist  = " << jointstate->position[12] << " / " << jointstate->position[13] \
								 << " / " << jointstate->position[14] << endl;
		cout << "index.dist  = " << jointstate->position[24] << " / " << jointstate->position[25] \
								 << " / " << jointstate->position[26] << endl;
		cout << "middle.dist = " << jointstate->position[36] << " / " << jointstate->position[37] \
								 << " / " << jointstate->position[38] << endl;
		cout << "ring.dist   = " << jointstate->position[48] << " / " << jointstate->position[49] \
								 << " / " << jointstate->position[50] << endl;
		cout << "pinky.dist  = " << jointstate->position[60] << " / " << jointstate->position[61] \
								 << " / " << jointstate->position[62] << endl; */

	/*	cout << "hands_detected = " << jointstate->position[63] << endl;
		cout << "right hand? = " << jointstate->position[64] << endl;
		cout << "strength = " << jointstate->position[65] << endl;
		cout << endl; */
	}
}

void Uial::leapCallback(const geometry_msgs::PoseStamped::ConstPtr& posstamped)
{
	int num;
	double roll, pitch, yaw;
	nav_msgs::Odometry odom;
	sensor_msgs::JointState js;
	vpColVector current_joints(5), send_joints(5);

	//Initial user hand position
	if ((initPosition.pose.position.x == 0) and (initPosition.pose.position.y == 0) \
		and (initPosition.pose.position.z == 0))
	{
		//Save initial hand position & orientation
		initPosition.pose = posstamped->pose;
		//Set initial transform data
		transform_init.setOrigin(tf::Vector3(posstamped->pose.position.x, posstamped->pose.position.y, \
								posstamped->pose.position.z));
		q_init = tf::Quaternion(posstamped->pose.orientation.x, posstamped->pose.orientation.y, \
								posstamped->pose.orientation.z, posstamped->pose.orientation.w);
		transform_init.setRotation(q_init.normalize());
		//The starting point in the first waypoint
		waypointsList[0].pose = posstamped->pose;

		cout << "Starting hand...\n" << initPosition.pose << endl;

		cout << "Press Enter to continue... ";
		num = getchar();		
	}
	br.sendTransform(tf::StampedTransform(transform_init, ros::Time::now(), "world", "init_pose"));

	//Keep all with 0. We send velocities, not position.
	odom.pose.pose.position		 = p0;
	odom.pose.pose.orientation	 = q0;
	odom.pose.pose.orientation.w = 1;

	//if the user don't move the hand, the robot keep the position
	if ((posstamped->pose.position.x == previousPosition.pose.position.x) and \
		(posstamped->pose.position.y == previousPosition.pose.position.y) and \
		(posstamped->pose.position.z == previousPosition.pose.position.z))
	{
		robotStopped = true;
		currentPosition.pose.position = p0;
		currentPosition.pose.orientation = q0;
	}
	else
	{
		robotStopped = false;
		//store previous position
		previousPosition.pose.position.x = posstamped->pose.position.x;
		previousPosition.pose.position.y = posstamped->pose.position.y;
		previousPosition.pose.position.z = posstamped->pose.position.z;
		
		//If only the right hand is detected, the user controls the robot navigation
		if ((handsDetected == 1) and rightHand)
		{
			//LeapMotion Y-axis -> Robot Z-axis
			if ((posstamped->pose.position.y >= 80) and (posstamped->pose.position.y <= 120))
				currentPosition.pose.position.y = 0.0;
			else
			{
				if (posstamped->pose.position.y < 80)
					if ((!sensorRangeAlarm) and (posstamped->pose.position.y < 80) and (posstamped->pose.position.y >= 55))
							currentPosition.pose.position.y = 0.3;
					else
					{
						if ((!sensorRangeAlarm) and (posstamped->pose.position.y < 55))
							currentPosition.pose.position.y = 0.6;
						else //sensorRangeAlarm = true
							currentPosition.pose.position.y = 0.0;
					}
				else //User's hand is upper
				{
					if ((!sensorPressureAlarm) and (posstamped->pose.position.y > 120) and (posstamped->pose.position.y <= 170))
						currentPosition.pose.position.y = -0.3;
					else
					{
						if ((!sensorPressureAlarm) and (posstamped->pose.position.y > 170))
							currentPosition.pose.position.y = -0.6;
						else //sensorPressureAlarm = true
							currentPosition.pose.position.y = 0.0;
					}
				}
				if (sensorRangeAlarm)
					cout << "Alarm: robot on seafloor |" << endl;
				if (sensorPressureAlarm)
					cout << "Alarm: robot on surface |" << endl;
			}
			//LeapMotion Z-axis -> Robot X-axis
			if ((posstamped->pose.position.z >= -10) and (posstamped->pose.position.z <= 15))
				currentPosition.pose.position.z = 0.0;
			else
			{
				if ((posstamped->pose.position.z < -10) and (posstamped->pose.position.z <= -35))
					currentPosition.pose.position.z = 0.6;
				else
				{
					if (posstamped->pose.position.z < -10)
						currentPosition.pose.position.z = 0.3;
					else
					{
						if ((posstamped->pose.position.z > 15) and (posstamped->pose.position.z <= 70))
							currentPosition.pose.position.z = -0.3;
						else
							currentPosition.pose.position.z = -0.6;
					}
				}
			}
			//LeapMotion X-axis -> Robot Y-axis
			if ((posstamped->pose.position.x >= -25.0) and (posstamped->pose.position.x <= 25.0))
				currentPosition.pose.position.x = 0.00;
			else
			{
				if ((posstamped->pose.position.x > 25.0) and (posstamped->pose.position.x <= 90.0))
					currentPosition.pose.position.x = 0.3;
				else
				{
					if (posstamped->pose.position.x > 90.0) //and (posstamped->pose.position.x > 70.0))
						currentPosition.pose.position.x = 0.6;
					else
					{
						if ((posstamped->pose.position.x < -25.0) and (posstamped->pose.position.x <= -90.0))
							currentPosition.pose.position.x = -0.6;
						else
							currentPosition.pose.position.x = -0.3;
					}
				}
			}
			//Yaw-orientation
			transform_new.setOrigin(tf::Vector3(posstamped->pose.orientation.x, \
									posstamped->pose.orientation.y, posstamped->pose.orientation.z));
			q_new = tf::Quaternion(posstamped->pose.orientation.x, posstamped->pose.orientation.y, \
									posstamped->pose.orientation.z, posstamped->pose.orientation.w);
			transform_new.setRotation(q_new.normalize());
			br.sendTransform(tf::StampedTransform(transform_new, ros::Time::now(), "world", "new_pose"));

			try
			{
				listener->lookupTransform("/new_pose", "/init_pose", ros::Time(0), transform);
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}

			transform.getBasis().getRPY(roll, pitch, yaw);
			if (roll < -0.5)
			{
				currentPosition.pose.orientation.x = -0.2;
			}
			else
			{
				if ((roll >= -0.5) and (roll <= 0.5))
					currentPosition.pose.orientation.x = 0.0;			
				else
					currentPosition.pose.orientation.x = 0.2;
			}
			
			//Assign the calculated values into the publisher
			odom.twist.twist.linear.x =  currentPosition.pose.position.z;
			odom.twist.twist.linear.y =  currentPosition.pose.position.x;
			odom.twist.twist.linear.z =  currentPosition.pose.position.y;
			odom.twist.twist.angular.x = 0; //roll;
			odom.twist.twist.angular.y = 0; //pitch;
			odom.twist.twist.angular.z = currentPosition.pose.orientation.x; //yaw
			for (int i=0; i<36; i++)
			{
				odom.twist.covariance[i]=0;
				odom.pose.covariance[i]=0;
			}
			vel_pub_.publish(odom);			
		}//((handsDetected == 1) and rightHand)
		
		//If there are two hands, the user controls the end effector
		else if ((handsDetected == 2) and rightHand)
		{
			cout << "two hands detected" << endl;
			//LeapMotion Y-axis -> end effector Z-axis
			if ((posstamped->pose.position.y >= 80) and (posstamped->pose.position.y <= 120))
				currentPosition.pose.position.y = 0.0;
			else
			{
				cout << "Arm Z-axis" << endl;
				if (posstamped->pose.position.y < 80)
					if ((!sensorRangeAlarm) and (posstamped->pose.position.y < 80) and (posstamped->pose.position.y >= 55))
							currentPosition.pose.position.y = 0.3;
					else
					{
						if ((!sensorRangeAlarm) and (posstamped->pose.position.y < 55))
							currentPosition.pose.position.y = 0.6;
						else //sensorRangeAlarm = true
							currentPosition.pose.position.y = 0.0;
					}
				else //User's hand is upper
				{
					if ((posstamped->pose.position.y > 120) and (posstamped->pose.position.y <= 170))
						currentPosition.pose.position.y = -0.3;
					else
					{
						if (posstamped->pose.position.y > 170)
							currentPosition.pose.position.y = -0.6;
					}
				}
				if (sensorRangeAlarm)
					cout << "Alarm: robot on seafloor |" << endl;
			}
			//LeapMotion Z-axis -> end effector X-axis
			if ((posstamped->pose.position.z >= -10) and (posstamped->pose.position.z <= 15))
				currentPosition.pose.position.z = 0.0;
			else
			{
				cout << "Arm X-axis" << endl;
				if ((posstamped->pose.position.z < -10) and (posstamped->pose.position.z <= -35))
					currentPosition.pose.position.z = 0.6;
				else
				{
					if (posstamped->pose.position.z < -10)
						currentPosition.pose.position.z = 0.3;
					else
					{
						if ((posstamped->pose.position.z > 15) and (posstamped->pose.position.z <= 70))
							currentPosition.pose.position.z = -0.3;
						else
							currentPosition.pose.position.z = -0.6;
					}
				}
			}
			//LeapMotion X-axis -> end effector Y-axis
			if ((posstamped->pose.position.x >= -25.0) and (posstamped->pose.position.x <= 25.0))
				currentPosition.pose.position.x = 0.00;
			else
			{
				cout << "Arm Y-axis" << endl;
				if ((posstamped->pose.position.x > 25.0) and (posstamped->pose.position.x <= 90.0))
					currentPosition.pose.position.x = 0.3;
				else
				{
					if (posstamped->pose.position.x > 90.0) //and (posstamped->pose.position.x > 70.0))
						currentPosition.pose.position.x = 0.6;
					else
					{
						if ((posstamped->pose.position.x < -25.0) and (posstamped->pose.position.x <= -90.0))
							currentPosition.pose.position.x = -0.6;
						else
							currentPosition.pose.position.x = -0.3;
					}
				}
			}
			//Yaw-orientation
			transform_new.setOrigin(tf::Vector3(posstamped->pose.orientation.x, \
									posstamped->pose.orientation.y, posstamped->pose.orientation.z));
			q_new = tf::Quaternion(posstamped->pose.orientation.x, posstamped->pose.orientation.y, \
									posstamped->pose.orientation.z, posstamped->pose.orientation.w);
			transform_new.setRotation(q_new.normalize());
			br.sendTransform(tf::StampedTransform(transform_new, ros::Time::now(), "world", "new_pose"));

			try
			{
				listener->lookupTransform("/new_pose", "/init_pose", ros::Time(0), transform);
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}

			transform.getBasis().getRPY(roll, pitch, yaw);
			if (roll < -0.5)
			{
				currentPosition.pose.orientation.x = -0.2;
			}
			else
			{
				if ((roll >= -0.5) and (roll <= 0.5))
					currentPosition.pose.orientation.x = 0.0;			
				else
					currentPosition.pose.orientation.x = 0.2;
			}			
			
			//Calculate the base-end effector matrix
			desired_bMe=bMe;
			desired_bMe[0][3]-= currentPosition.pose.position.x;
			desired_bMe[1][3]-= currentPosition.pose.position.x;
			desired_bMe[2][3]-= currentPosition.pose.position.x;

			//std::cerr<<"Desired bMe"<<std::endl<<desired_bMe<<std::endl;
			next_joints = robot->armIK(desired_bMe);
			//std::cerr<<"Desired joints"<<std::endl<<next_joints<<std::endl;
			
			//If valid joints and reasonable new position ... ask to MOVE
			if ((next_joints[0] > -1.57) && (next_joints[0] < 2.1195) && (next_joints[1] > 0) && \
				(next_joints[1] < 1.58665) && (next_joints[2] > 0) && (next_joints[2] < 2.15294))
			{
				if ((std::abs(desired_bMe[0][3] - bMe[0][3]) < 5) && \
					(std::abs(desired_bMe[1][3] - bMe[1][3]) < 5) && \
					(std::abs(bMe[2][3] - bMe[2][3]) < 5))
					moving = true;
				else
					ROS_INFO("Error: New position too far form the original position.");
			}
			else
				ROS_INFO("Error: Unreachable position.");			
			
			//Send the parameters
			//int res=robot->getJointValues(current_joints);
			if(robot->getJointValues(current_joints))
			{		
				bMe=robot->directKinematics(current_joints);
				//std::cerr<<"Current bMe"<<std::endl<<bMe<<std::endl;
				
				if(moving)
				{
					//Check if it's almost there
					if((std::abs(desired_bMe[0][3] - bMe[0][3]) > 0.01) || \
						(std::abs(desired_bMe[1][3] - bMe[1][3]) > 0.01) || \
						(std::abs(desired_bMe[2][3] - bMe[2][3]) > 0.01))
					{
						ROS_INFO("Info: Movingo to desired position.");
						send_joints[0]=next_joints[0]-current_joints[0];
						send_joints[1]=next_joints[1]-current_joints[1];
						send_joints[2]=next_joints[2]-current_joints[2];
					}
					else
					{
						ROS_INFO("Info: Position reached");
						send_joints[0]=0;
						send_joints[1]=0;
						send_joints[2]=0;
						moving=false;
					}
				}
				else
				{
					ROS_INFO("Info: Joystick is not moving.");
					send_joints[0]=0;
					send_joints[1]=0;
					send_joints[2]=0;		
				}
			}
			else{
				ROS_ERROR("Error: Impossible to access to the arm controler");
				send_joints[0]=0;
				send_joints[1]=0;
				send_joints[2]=0;
			}

			robot->setJointVelocity(send_joints);
		} //if ((handsDetected == 2) and rightHand)
	}


	// DEBUG AREA: print hand position and command to send to UWSim
	if (DEBUG_leap_sub)
	{
		cout << "Absolute hand position: (" << posstamped->pose.position.x << ", " << posstamped->pose.position.y << \
				", " << posstamped->pose.position.z << " : " << posstamped->pose.orientation.x << \
				", " << posstamped->pose.orientation.y << ", " << posstamped->pose.orientation.z << \
				", " << posstamped->pose.orientation.w << ")" << endl;
		if ((currentPosition.pose.position.x == 0) and (currentPosition.pose.position.y == 0) and \
			(currentPosition.pose.position.z == 0) and (currentPosition.pose.orientation.x == 0))
			cout << "Robot stopped: the user's hand is not detected, is in the deadzone or does not move." << endl;
		else
		{
			cout << "Robot movement(s): ";
			if (currentPosition.pose.position.y != 0)
			{
				if (currentPosition.pose.position.y < 0)
					cout << "Z-Axis: up " << currentPosition.pose.position.y << "|";
				else
					cout << "Z-Axis: down " << currentPosition.pose.position.y << "|";
			}
			if (sensorRangeAlarm)
				cout << "Alarm: robot on seafloor |";
			if (sensorPressureAlarm)
				cout << "Alarm: robot on surface |";
			if (currentPosition.pose.position.z != 0)
			{
				if (currentPosition.pose.position.z < 0)
					cout << "X-Axis: back " << currentPosition.pose.position.z << "|";
				else
					cout << "X-Axis: front " << currentPosition.pose.position.z << "|";
			}
			if (currentPosition.pose.position.x != 0)
			{
				if (currentPosition.pose.position.x < 0)
					cout << "Y-Axis: left " << currentPosition.pose.position.x << "|";
				else
					cout << "Y-Axis: right " << currentPosition.pose.position.x << "|";
			}
			if (currentPosition.pose.orientation.x != 0)
			{
				(currentPosition.pose.orientation.x < 0 ? cout << " Yaw:  counterclockwise" << endl : cout << " Yaw:  clockwise" << endl);
				//cout << "roll = " << roll << " | pitch = " << pitch << " | yaw = " << yaw << endl;
			}
			else
				cout << endl;
		}
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uial");
  Uial uial_control;
  ros::spin();
}

