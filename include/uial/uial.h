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

//ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <underwater_sensor_msgs/Pressure.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace std;


class Uial
{

	public:
		Uial();
		~Uial();

		bool sensorPressureAlarm;
		bool sensorRangeAlarm;
		bool handIsOpen;
		bool rotationMode;


	private:
		void leapHandCallback(const sensor_msgs::JointState::ConstPtr& posstamped);
		void leapCallback(const geometry_msgs::PoseStamped::ConstPtr& posstamped);
		void sensorPressureCallback(const underwater_sensor_msgs::Pressure::ConstPtr& pressureValue);
		void sensorRangeCallback(const sensor_msgs::Range::ConstPtr& rangeValue);

		ros::NodeHandle nh_;

		float initPosition[3];
		float currentPosition[3];
		float previousPosition[3];
		float initOrientation[4];
		float currentOrientation[4];

		ros::Publisher vel_pub_;
		ros::Subscriber hand_sub_;
		ros::Subscriber leap_sub_;
		ros::Subscriber sensorPressure_sub_;
		ros::Subscriber sensorRange_sub_;

		tf::Transform transform_init, transform_new;
		tf::StampedTransform transform;
		tf::Quaternion q_init, q_new;
		tf::TransformBroadcaster br;
		tf::TransformListener *listener;
};

