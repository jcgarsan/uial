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

#define TOPIC  "/dataNavigator_G500RAUVI"		//shipweck scene
//#define TOPIC  "/dataNavigator"				//CIRS scene

//#define CIRS_pressure 0.2
//#define CIRS_range 0.6
#define shipweck_pressure -1.0
#define shipweck_range 1.2

using namespace std;



Uial::Uial()
{
	//initializing values
	currentPosition[0] = 0;
	currentPosition[1] = 0;
	currentPosition[2] = 0;
	previousPosition[0] = 0;
	previousPosition[1] = 0;
	previousPosition[2] = 0;
	sensorRangeAlarm = false;
	sensorPressureAlarm = false;

	//publisher and subscriber initialization
	vel_pub_ = nh_.advertise<nav_msgs::Odometry>(TOPIC,1);
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

void Uial::leapCallback(const geometry_msgs::PoseStamped::ConstPtr& posstamped)
{
	int num;
	
	//Keep all with 0. We send velocities, not position.
	nav_msgs::Odometry odom;
	odom.pose.pose.position.x=0.0;
	odom.pose.pose.position.y=0.0;
	odom.pose.pose.position.z=0.0;
	odom.pose.pose.orientation.x=0.0;
	odom.pose.pose.orientation.y=0.0;
	odom.pose.pose.orientation.z=0.0;
	odom.pose.pose.orientation.w=1;

	if ((posstamped->pose.position.x == previousPosition[0]) and \
		(posstamped->pose.position.y == previousPosition[1]) and \
		(posstamped->pose.position.z == previousPosition[2]))
	{
		for (int i=0; i<3; i++)
			currentPosition[i] = 0.00;
	}
	else
	{
		//store previous position
		previousPosition[0] = posstamped->pose.position.x;
		previousPosition[1] = posstamped->pose.position.y;
		previousPosition[2] = posstamped->pose.position.z;

		cout << "Robot movement(s): ";
		//LeapMotion Y-axis -> Robot Z-axis
		if ((posstamped->pose.position.y >= 80) and (posstamped->pose.position.y <= 120))		//100 -> 120
			currentPosition[1] = 0.00;
		else
		{
			cout << " Z-Axis: " ;
			if (posstamped->pose.position.y < 80)	//User's hand is lower 80=60, 55=45
				if ((!sensorRangeAlarm) and (posstamped->pose.position.y < 80) and (posstamped->pose.position.y >= 55))
				{
						currentPosition[1] = 0.3;
						cout << " down |";
				}
				else
					if ((!sensorRangeAlarm) and (posstamped->pose.position.y < 55))
					{
						currentPosition[1] = 0.6;
						cout << " down 2x |";
					}
					else //sensorRangeAlarm = true
					{
						currentPosition[1] = 0.0;
					}
			else //User's hand is upper
				if ((!sensorPressureAlarm) and (posstamped->pose.position.y > 120) and (posstamped->pose.position.y <= 170))
				{
					currentPosition[1] = -0.3;
					cout << " up |";
				}
				else
					if ((!sensorPressureAlarm) and (posstamped->pose.position.y > 170))
					{
						currentPosition[1] = -0.6;
						cout << " up 2x |";
					}
					else //sensorPressureAlarm = true
					{
						currentPosition[1] = 0.0;
					}
			if (sensorRangeAlarm)
				cout << "Alarm: robot on seafloor |";
			if (sensorPressureAlarm)
				cout << "Alarm: robot on surface |";
		}
		//LeapMotion Z-axis -> Robot X-axis
		if ((posstamped->pose.position.z >= -10) and (posstamped->pose.position.z <= 15))
			currentPosition[2] = 0.00;
		else
		{
			cout << " X-Axis: " ;
			if ((posstamped->pose.position.z < -10) and (posstamped->pose.position.z <= -35))
			{
				currentPosition[2] = 0.6;
				cout << " front 2x |";
			}
			else
				if (posstamped->pose.position.z < -10)
				{
					currentPosition[2] = 0.3;
					cout << " front |";
				}
				else
					if ((posstamped->pose.position.z > 15) and (posstamped->pose.position.z <= 70))
					{
						currentPosition[2] = -0.3;
						cout << " back |";
					}
					else
					{
							currentPosition[2] = -0.6;
							cout << " back 2x |";
					}
		}
		//LeapMotion X-axis -> Robot Y-axis
		if ((posstamped->pose.position.x >= -25.0) and (posstamped->pose.position.x <= 25.0))
			currentPosition[0] = 0.00;
		else
		{
			cout << " Yaw: " ;
			if (posstamped->pose.position.x > 25.0)
			{
				currentPosition[0] = 0.3;
				cout << " clockwise |";
			}
			else
			{
				currentPosition[0] = -0.3;
				cout << " counterclockwise |";
			}
		}
		cout << endl;
	}
	//Assign the calculated values into the publisher
	odom.twist.twist.linear.x =  currentPosition[2]; 
	odom.twist.twist.linear.y =  0; //Standard Girona500 has not lateral movements
	odom.twist.twist.linear.z =  currentPosition[1];
	odom.twist.twist.angular.x = 0; //roll;
	odom.twist.twist.angular.y = 0; //pitch;
	odom.twist.twist.angular.z = currentPosition[0]; //yaw
	for (int i=0; i<36; i++)
	{
		odom.twist.covariance[i]=0;
		odom.pose.covariance[i]=0;
	}

	vel_pub_.publish(odom);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "uial");
  Uial uial_control;
  ros::spin();
}

