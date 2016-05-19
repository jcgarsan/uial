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
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
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
		
		std_msgs::Int8MultiArray	safetyMeasureAlarm;
		std_msgs::Int8MultiArray	userControlAlarm;
		std_msgs::Bool				userControlRequest;
		std_msgs::Bool				armControlRequest;
		
		ros::Time lastPressUserControl;
		ros::Time lastPressArmControl;

		ARM5Arm *robot;
		vpHomogeneousMatrix desired_bMe, bMe;
		vpColVector next_joints;


	private:
		void leapHandCallback(const sensor_msgs::JointState::ConstPtr& posstamped);
		void leapCallback(const geometry_msgs::PoseStamped::ConstPtr& posstamped);
		void odomCallback(const nav_msgs::Odometry::ConstPtr& odomValue);
		void spacenavButtonsCallback(const sensor_msgs::Joy::ConstPtr& spacenavButtons);
		void spacenavCallback(const geometry_msgs::Twist::ConstPtr& twistValue);
		void joystickCallback(const sensor_msgs::Joy::ConstPtr& joystick);
		void safetyMeasuresCallback(const std_msgs::Int8MultiArray::ConstPtr& msg);
		void userControlCallback(const std_msgs::Int8MultiArray::ConstPtr& msg);


		ros::NodeHandle nh;

		geometry_msgs::Point 		p0;
		geometry_msgs::Quaternion 	q0;
		geometry_msgs::PoseStamped	initPosition;
		geometry_msgs::PoseStamped	previousPosition;
		geometry_msgs::PoseStamped	currentPosition;

		ros::Publisher  pub_vel;
		ros::Publisher  pub_acc;
		ros::Publisher  pub_safety;
		ros::Publisher	pub_userControlRequest;
		ros::Publisher	pub_armControlRequest;

		ros::Subscriber sub_hand;
		ros::Subscriber sub_leap;
		ros::Subscriber sub_odom;
		ros::Subscriber sub_spacenav;
		ros::Subscriber sub_spacenavButtons;
		ros::Subscriber sub_joystick;
		ros::Subscriber sub_userControlRequest;
		ros::Subscriber sub_safetyMeasures;
		ros::Subscriber sub_userControl;

		tf::Transform 				transform_init, transform_new;
		tf::StampedTransform 		transform;
		tf::Quaternion				q_init, q_new;
		tf::TransformBroadcaster	br;
		tf::TransformListener		*listener;
};

