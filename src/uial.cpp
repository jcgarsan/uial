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

//CIRS scene
//#define TOPIC  "/dataNavigator"				
//#define CIRS_pressure 0.2
//#define CIRS_range 0.6

//shipweck scene
#define TOPIC  "/dataNavigator_G500RAUVI"		
#define shipweck_pressure -1.0
#define shipweck_range 1.2

#define DEBUG_hand_sub 0
#define DEBUG_leap_sub 0

using namespace std;



Uial::Uial()
{
	//initializing values
	initPosition[0] = 0;
	initPosition[1] = 0;
	initPosition[2] = 0;
	currentPosition[0] = 0;
	currentPosition[1] = 0;
	currentPosition[2] = 0;
	previousPosition[0] = 0;
	previousPosition[1] = 0;
	previousPosition[2] = 0;
	initOrientation[0] = 0;
	initOrientation[1] = 0;
	initOrientation[2] = 0;
	initOrientation[3] = 0;
	sensorRangeAlarm = false;
	sensorPressureAlarm = false;
	handIsOpen = false;
	rotationMode = false;

	listener = new (tf::TransformListener);

	//publisher and subscriber initialization
	vel_pub_ = nh_.advertise<nav_msgs::Odometry>(TOPIC,1);
	hand_sub_ = nh_.subscribe<sensor_msgs::JointState>("leap_tracker/joint_state_out", 1, &Uial::leapHandCallback, this);
	leap_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("leap_tracker/pose_stamped_out", 1, &Uial::leapCallback, this);
	sensorPressure_sub_ = nh_.subscribe<underwater_sensor_msgs::Pressure>("g500/pressure", 1, &Uial::sensorPressureCallback, this);
	sensorRange_sub_ = nh_.subscribe<sensor_msgs::Range>("uwsim/g500/range", 1, &Uial::sensorRangeCallback, this);
}

Uial::~Uial()
{
	//Destructor
}

void Uial::sensorPressureCallback(const underwater_sensor_msgs::Pressure::ConstPtr& pressureValue)
{
	//if (pressureValue->pressure < CIRS_pressure)
	if (pressureValue->pressure > shipweck_pressure)
		sensorPressureAlarm = true;
	else
		sensorPressureAlarm = false;
}

void Uial::sensorRangeCallback(const sensor_msgs::Range::ConstPtr& rangeValue)
{
	//if (rangeValue->range < CIRS_range)
	if (rangeValue->range < shipweck_range)
		sensorRangeAlarm = true;
	else
		sensorRangeAlarm = false;
}

void Uial::leapHandCallback(const sensor_msgs::JointState::ConstPtr& jointstate)
{
	(jointstate->position[63] >= 0.9 ? handIsOpen = false : handIsOpen = true);

	//index & middle are pointing && ring & pinky are closed
	if ((jointstate->position[24] > 0) and (jointstate->position[36] > 0) \
		and (jointstate->position[48] < 0) and (jointstate->position[60] < 0) \
		and (handIsOpen))
		rotationMode = true;
	else
		rotationMode = false;

	if (DEBUG_hand_sub)
	{
		(jointstate->position[63] >= 0.9 ? cout << "hand is closed" << endl : cout << "hand is opened" << endl);
		(rotationMode == false ? cout << "Not rotationMode" << endl : cout << " rotationMode" << endl);

	/*	cout << "index.meta  = " << jointstate->position[15] << " / " << jointstate->position[16] \
								 << " / " << jointstate->position[17] << endl;
		cout << "index.prox  = " << jointstate->position[18] << " / " << jointstate->position[19] \
								 << " / " << jointstate->position[20] << endl;
		cout << "index.mid  = " << jointstate->position[21] << " / " << jointstate->position[22] \
								 << " / " << jointstate->position[23] << endl;
		cout << "index.dist  = " << jointstate->position[24] << " / " << jointstate->position[25] \
								 << " / " << jointstate->position[26] << endl;
		cout << endl;*/

		cout << "thumb.dist  = " << jointstate->position[12] << " / " << jointstate->position[13] \
								 << " / " << jointstate->position[14] << endl;
		cout << "index.dist  = " << jointstate->position[24] << " / " << jointstate->position[25] \
								 << " / " << jointstate->position[26] << endl;
		cout << "middle.dist = " << jointstate->position[36] << " / " << jointstate->position[37] \
								 << " / " << jointstate->position[38] << endl;
		cout << "ring.dist   = " << jointstate->position[48] << " / " << jointstate->position[49] \
								 << " / " << jointstate->position[50] << endl;
		cout << "pinky.dist  = " << jointstate->position[60] << " / " << jointstate->position[61] \
								 << " / " << jointstate->position[62] << endl; 
		cout << "strength = " << jointstate->position[63] << endl;
		cout << endl;
	}
}

void Uial::leapCallback(const geometry_msgs::PoseStamped::ConstPtr& posstamped)
{
	int num;
	double roll, pitch, yaw;
	
	//Initial user hand position
	if ((initPosition[0] == 0) and (initPosition[1] == 0) and (initPosition[2] == 0))
	{
		initPosition[0] = posstamped->pose.position.x;
		initPosition[1] = posstamped->pose.position.y;
		initPosition[2] = posstamped->pose.position.z;
		//Save initial hand orientation
		initOrientation[0] = posstamped->pose.orientation.x;
		initOrientation[1] = posstamped->pose.orientation.y;
		initOrientation[2] = posstamped->pose.orientation.z;
		initOrientation[3] = posstamped->pose.orientation.w;
		//Set initial transform data
		transform_init.setOrigin(tf::Vector3(initPosition[0], initPosition[1], initPosition[2]));
//		transform_init.setOrigin(tf::Vector3(posstamped->pose.orientation.x, \
								posstamped->pose.orientation.y, posstamped->pose.orientation.z));
		q_init = tf::Quaternion(posstamped->pose.orientation.x, posstamped->pose.orientation.y, \
								posstamped->pose.orientation.z, posstamped->pose.orientation.w);
		transform_init.setRotation(q_init.normalize());

		cout << "Starting hand position: (" << initPosition[0] << "," << initPosition[1] \
			 << "," << initPosition[2] << " :: " << initOrientation[1] << ")" << endl;
		cout << "Press Enter to continue... ";
		num = getchar();		
	}
	br.sendTransform(tf::StampedTransform(transform_init, ros::Time::now(), "world", "init_pose"));

	
	//Keep all with 0. We send velocities, not position.
	nav_msgs::Odometry odom;
	odom.pose.pose.position.x=0.0;
	odom.pose.pose.position.y=0.0;
	odom.pose.pose.position.z=0.0;
	odom.pose.pose.orientation.x=0.0;
	odom.pose.pose.orientation.y=0.0;
	odom.pose.pose.orientation.z=0.0;
	odom.pose.pose.orientation.w=1;

	//if the user don't move the hand, the robot keep the position
	if ((posstamped->pose.position.x == previousPosition[0]) and \
		(posstamped->pose.position.y == previousPosition[1]) and \
		(posstamped->pose.position.z == previousPosition[2]))
	{
		for (int i=0; i<3; i++)
		{
			currentPosition[i] = 0.0;
			currentOrientation[0] = 0.0;
		}
	}
	else
	{
		//store previous position
		previousPosition[0] = posstamped->pose.position.x;
		previousPosition[1] = posstamped->pose.position.y;
		previousPosition[2] = posstamped->pose.position.z;

		//LeapMotion Y-axis -> Robot Z-axis
		if ((posstamped->pose.position.y >= 80) and (posstamped->pose.position.y <= 120))
			currentPosition[1] = 0.0;
		else
		{
			if (posstamped->pose.position.y < 80)
				if ((!sensorRangeAlarm) and (posstamped->pose.position.y < 80) and (posstamped->pose.position.y >= 55))
						currentPosition[1] = 0.3;
				else
				{
					if ((!sensorRangeAlarm) and (posstamped->pose.position.y < 55))
						currentPosition[1] = 0.6;
					else //sensorRangeAlarm = true
						currentPosition[1] = 0.0;
				}
			else //User's hand is upper
			{
				if ((!sensorPressureAlarm) and (posstamped->pose.position.y > 120) and (posstamped->pose.position.y <= 170))
					currentPosition[1] = -0.3;
				else
				{
					if ((!sensorPressureAlarm) and (posstamped->pose.position.y > 170))
						currentPosition[1] = -0.6;
					else //sensorPressureAlarm = true
						currentPosition[1] = 0.0;
				}
			}
/*			if (sensorRangeAlarm)
				cout << "Alarm: robot on seafloor |" << endl;
			if (sensorPressureAlarm)
				cout << "Alarm: robot on surface |" << endl;*/
		}
		//LeapMotion Z-axis -> Robot X-axis
		if ((posstamped->pose.position.z >= -10) and (posstamped->pose.position.z <= 15))
			currentPosition[2] = 0.0;
		else
		{
			if ((posstamped->pose.position.z < -10) and (posstamped->pose.position.z <= -35))
				currentPosition[2] = 0.6;
			else
			{
				if (posstamped->pose.position.z < -10)
					currentPosition[2] = 0.3;
				else
				{
					if ((posstamped->pose.position.z > 15) and (posstamped->pose.position.z <= 70))
						currentPosition[2] = -0.3;
					else
						currentPosition[2] = -0.6;
				}
			}
		}
		//LeapMotion X-axis -> Robot Y-axis
		if ((posstamped->pose.position.x >= -25.0) and (posstamped->pose.position.x <= 25.0))
			currentPosition[0] = 0.00;
		else
		{
			if ((posstamped->pose.position.x > 25.0) and (posstamped->pose.position.x <= 90.0))
				currentPosition[0] = 0.3;
			else
			{
				if (posstamped->pose.position.x > 90.0) //and (posstamped->pose.position.x > 70.0))
					currentPosition[0] = 0.6;
				else
				{
					if ((posstamped->pose.position.x < -25.0) and (posstamped->pose.position.x <= -90.0))
						currentPosition[0] = -0.6;
					else
						currentPosition[0] = -0.3;
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

		try{
			listener->lookupTransform("/new_pose", "/init_pose", ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		transform.getBasis().getRPY(roll, pitch, yaw);
		if (roll < -0.5)
		{
			currentOrientation[0] = -0.2;
		}
		else
		{
			if ((roll >= -0.5) and (roll <= 0.5))
				currentOrientation[0] = 0.0;			
			else
				currentOrientation[0] = 0.2;
		}
	}
	//Assign the calculated values into the publisher
	odom.twist.twist.linear.x =  currentPosition[2]; 
	odom.twist.twist.linear.y =  currentPosition[0]; 
	odom.twist.twist.linear.z =  currentPosition[1];
	odom.twist.twist.angular.x = 0; //roll;
	odom.twist.twist.angular.y = 0; //pitch;
	odom.twist.twist.angular.z = currentOrientation[0]; //yaw
	for (int i=0; i<36; i++)
	{
		odom.twist.covariance[i]=0;
		odom.pose.covariance[i]=0;
	}

	vel_pub_.publish(odom);

	if (DEBUG_leap_sub)
	{
		cout << "Robot movement(s): ";
		if (currentPosition[1] != 0)
		{
			if (currentPosition[1] < 0)
				cout << "Z-Axis: up " << currentPosition[1] << "|";
			else
				cout << "Z-Axis: down " << currentPosition[1] << "|";
		}
		if (sensorRangeAlarm)
			cout << "Alarm: robot on seafloor |";
		if (sensorPressureAlarm)
			cout << "Alarm: robot on surface |";
		if (currentPosition[2] != 0)
		{
			if (currentPosition[2] < 0)
				cout << "X-Axis: back " << currentPosition[2] << "|";
			else
				cout << "X-Axis: front " << currentPosition[2] << "|";
		}
		if (currentPosition[0] != 0)
		{
			if (currentPosition[0] < 0)
				cout << "Y-Axis: left " << currentPosition[0] << "|";
			else
				cout << "Y-Axis: right " << currentPosition[0] << "|";
		}
		if (currentOrientation[0] != 0)
		{
			(currentOrientation[0] < 0 ? cout << " Yaw:  counterclockwise" << endl : cout << " Yaw:  clockwise" << endl);
			//cout << "roll = " << roll << " | pitch = " << pitch << " | yaw = " << yaw << endl;
		}
		else
			cout << endl;
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uial");
  Uial uial_control;
  ros::spin();
}

