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
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include "std_msgs/Float64MultiArray.h"
#include <underwater_sensor_msgs/Pressure.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <mar_robot_arm5e/ARM5Arm.h>

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
		bool selectWaypoint;
		bool robotStopped;
		bool rightHand;
		bool moving;
		bool robotControl;
	
		int numWaypoint;
		int handsDetected;
		int gripperApperture;
		int gripperRotation;

		geometry_msgs::PoseStamped	waypointsList[10];
		std_msgs::Bool				safetyMeasureAlarm;
		std_msgs::Bool				userControlRequest;
		
		ros::Time lastPress;

		ARM5Arm *robot;
		vpHomogeneousMatrix desired_bMe, bMe;
		vpColVector next_joints;


	private:
		void leapHandCallback(const sensor_msgs::JointState::ConstPtr& posstamped);
		void leapCallback(const geometry_msgs::PoseStamped::ConstPtr& posstamped);
		void sensorPressureCallback(const underwater_sensor_msgs::Pressure::ConstPtr& pressureValue);
		void sensorRangeCallback(const sensor_msgs::Range::ConstPtr& rangeValue);
		void odomCallback(const nav_msgs::Odometry::ConstPtr& odomValue);
		void spacenavButtonsCallback(const sensor_msgs::Joy::ConstPtr& spacenavButtons);
		void spacenavCallback(const geometry_msgs::Twist::ConstPtr& twistValue);
		void joystickCallback(const sensor_msgs::Joy::ConstPtr& joystick);


		ros::NodeHandle nh_;

		geometry_msgs::Point p0;
		geometry_msgs::Quaternion q0;
		geometry_msgs::PoseStamped initPosition;
		geometry_msgs::PoseStamped previousPosition;
		geometry_msgs::PoseStamped currentPosition;

		ros::Publisher  vel_pub_;
		ros::Publisher  acc_pub_;
		ros::Publisher  safety_pub_;
		ros::Publisher	userControlRequest_pub_;

		ros::Subscriber hand_sub_;
		ros::Subscriber leap_sub_;
		ros::Subscriber sensorPressure_sub_;
		ros::Subscriber sensorRange_sub_;
		ros::Subscriber odom_sub_;
		ros::Subscriber spacenav_sub_;
		ros::Subscriber spacenavButtons_sub_;
		ros::Subscriber joystick_sub_;
		ros::Subscriber userControlRequest_sub_;

		tf::Transform transform_init, transform_new;
		tf::StampedTransform transform;
		tf::Quaternion q_init, q_new;
		tf::TransformBroadcaster br;
		tf::TransformListener *listener;
};

