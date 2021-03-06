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
#include <vector>
#include "../include/uial/uial.h"

#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <cmath>  

#define pressureThreshold	0.5
#define rangeThreshold 		1.0
#define num_sensors			2		// 0 = is there an alarm?, 1 = surface, 2 = seafloor

//DEBUG Flags
#define DEBUG_sub_waypoint	0
#define DEBUG_sub_hand 		0
#define DEBUG_sub_leap 		0
#define DEBUG_sub_spacenav	1
#define DEBUG_sub_joystick	0
#define DEBUG_sub_gamepad	1

//Acceleration or velocities
#define accelerations		1


//Device to be used
#define leapMotionDev		0		//SimulatedIAUV.cpp should be changed when LeapMotion is used
#define spaceMouseDev		1
#define joystickDev			0
#define gamepadDev			1

using namespace std;

static const int menuButtonLimits[] = {7, 4, 5, 2, 5};


Uial::Uial()
{
	//initializing values
	p0.x = 0; p0.y = 0; p0.z = 0;
	q0.x = 0; q0.y = 0; q0.z = 0; q0.w = 0;
	initPosition.pose.position			= p0;
	initPosition.pose.orientation		= q0;
	currentPosition.pose.position		= p0;
	currentPosition.pose.orientation	= q0;
	previousPosition.pose.position		= p0;
	previousPosition.pose.orientation	= q0;
	userControlRequest.data				= false;
	armControlRequest.data				= false;
	handIsOpen 							= false;
	rotationMode 						= false;
	selectWaypoint 						= false;
	robotStopped						= false;
	rightHand							= false;
	moving								= false;
	robotControl						= true;		//Selects robot control or arm control
	numWaypoint 						= 1;
	gripperApperture					= 0;
	gripperRotation						= 0;
	userMenuSelection					= 0;

	for (int i=0; i<=num_sensors; i++)
		safetyMeasureAlarm.data.push_back(0);
	for (int i=0; i<2; i++)
		userControlAlarm.data.push_back(0);
	for (int i=0; i<5; i++)		//[OnOff menu, select menu, exec option, menuID, buttonID]
		userMenuData.data.push_back(0);

	listener = new (tf::TransformListener);

	//Publisher initialization
	if (!accelerations)
	        pub_vel = nh.advertise<nav_msgs::Odometry>("dataNavigator", 1);
	else
        	pub_acc = nh.advertise<std_msgs::Float64MultiArray>("g500/thrusters_input", 1);

   	pub_arm = nh.advertise<std_msgs::Float64MultiArray>("g500/arm_input", 1);
	pub_userControlRequest = nh.advertise<std_msgs::Bool>("userControlRequest", 1);
	pub_armControlRequest = nh.advertise<std_msgs::Bool>("armControlRequest", 1);
	pub_userMenuData = nh.advertise<std_msgs::Int8MultiArray>("userMenuData", 1);

	//Subscriber initialization (device to be used)
	if (leapMotionDev)
	{
        	sub_hand = nh.subscribe<sensor_msgs::JointState>("leap_tracker/joint_state_out", 1, &Uial::leapHandCallback, this);
        	sub_leap = nh.subscribe<geometry_msgs::PoseStamped>("leap_tracker/pose_stamped_out", 1, &Uial::leapCallback, this);
	}
	if (spaceMouseDev)
	{
			sub_spacenav 		= nh.subscribe<geometry_msgs::Twist>("spacenav/twist", 1, &Uial::spacenavCallback, this);
			sub_spacenavButtons = nh.subscribe<sensor_msgs::Joy>("spacenav/joy", 1, &Uial::spacenavButtonsCallback, this);
	}
	if (joystickDev)
			sub_joystick = nh.subscribe<sensor_msgs::Joy>("joystick_out", 1, &Uial::joystickCallback, this); 
	if (gamepadDev)
			sub_gamepad = nh.subscribe<sensor_msgs::Joy>("joystick_out", 1, &Uial::gamepadCallback, this); 
	
	//Subscriber initialization (sensors)
	sub_odom = nh.subscribe<nav_msgs::Odometry>("uwsim/girona500_odom_RAUVI", 1, &Uial::odomCallback, this);
	sub_safetyMeasures = nh.subscribe<std_msgs::Int8MultiArray>("safetyMeasuresAlarm", 1, &Uial::safetyMeasuresCallback, this);
	sub_userControl = nh.subscribe<std_msgs::Int8MultiArray>("userControlAlarm", 1, &Uial::userControlCallback, this);
	
	//Arm control
	robot = new ARM5Arm(nh, "uwsim/joint_state", "uwsim/joint_state_command");

	lastPressUserControl = ros::Time::now();
	lastPressArmControl = ros::Time::now();
	lastPressNavButton = ros::Time::now();
	lastPressNavDial = ros::Time::now();
}

Uial::~Uial()
{
	//Destructor
}


void Uial::safetyMeasuresCallback(const std_msgs::Int8MultiArray::ConstPtr& msg)
{
	for (int i=0; i<=num_sensors; i++)
		safetyMeasureAlarm.data[i] = msg->data[i];
}


void Uial::userControlCallback(const std_msgs::Int8MultiArray::ConstPtr& msg)
{
	for (int i=0; i<2; i++)
		userControlAlarm.data[i] = msg->data[i];

	cout << "userControlCallback: [" << (int) userControlAlarm.data[0] << ", " << (int) userControlAlarm.data[1] << "]" << endl;
}


void Uial::spacenavButtonsCallback(const sensor_msgs::Joy::ConstPtr& spacenavButtons)
{
	ros::Time currentPressNavButton = ros::Time::now();
	ros::Duration difTime = currentPressNavButton - lastPressNavButton;
	if (difTime.toSec() > 0.3)
	{
		if ((spacenavButtons->buttons[0] == 0) and (spacenavButtons->buttons[1] == 1)) 
		{	//Display the menu
			userMenuData.data[0] = (userMenuData.data[0] + 1) % 2;
		}
		if ((spacenavButtons->buttons[0] == 1) and (spacenavButtons->buttons[1] == 0)) 
		{	//User selection
			userMenuData.data[2] = 1;
			userMenuData.data[0] = 0;
		}
		else
			userMenuData.data[2] = 0;
		lastPressNavButton = currentPressNavButton;
		pub_userMenuData.publish(userMenuData);
	}
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
			if (DEBUG_sub_waypoint)
			{
				cout << "New waypoint pose: \n" << waypointsList[numWaypoint].pose.position << endl;
			}
		}
		else
		{
			cout << "Waypoint similar. Quite close to the last one." << endl;
		}
	}
	if ((numWaypoint <= 10) and (DEBUG_sub_waypoint))
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


void Uial::leapHandCallback(const sensor_msgs::JointState::ConstPtr& jointstate)
{
	handsDetected = jointstate->position[63];
	
	(jointstate->position[64] == 1   ? rightHand = true : rightHand = false);
	(jointstate->position[65] >= 0.9 ? handIsOpen = false : handIsOpen = true);

	if ((rightHand) and (!handIsOpen))  //The right hand is closed
		selectWaypoint = true;

	if (DEBUG_sub_hand)
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
	double thrusters[5];
	sensor_msgs::JointState js;
	std_msgs::Float64MultiArray thrustersMsg;
	nav_msgs::Odometry odom;
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
	if (handsDetected == 0)
	{
		robotStopped = true;
		moving = false;
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
			if ((posstamped->pose.position.y >= 100) and (posstamped->pose.position.y <= 180))
				currentPosition.pose.position.y = 0.0;
			else
			{
				if (posstamped->pose.position.y < 100)
					if ((!sensorRangeAlarm) and (posstamped->pose.position.y < 100) and (posstamped->pose.position.y >= 55))
					{
							currentPosition.pose.position.y = 0.3;
							thrusters[2] = -0.5;
							thrusters[3] = -0.5;
					}
					else
					{
						if ((!sensorRangeAlarm) and (posstamped->pose.position.y < 55))
						{
							currentPosition.pose.position.y = 0.6;
							thrusters[2] = -0.5;
							thrusters[3] = -0.5;
						}
						else //sensorRangeAlarm = true
							currentPosition.pose.position.y = 0.0;
					}
				else //User's hand is upper
				{
					if ((!sensorPressureAlarm) and (posstamped->pose.position.y > 180) and (posstamped->pose.position.y <= 230))
					{
						currentPosition.pose.position.y = -0.3;
						thrusters[2] = 0.5;
						thrusters[3] = 0.5;
					}
					else
					{
						if ((!sensorPressureAlarm) and (posstamped->pose.position.y > 230))
						{
							currentPosition.pose.position.y = -0.6;
							thrusters[2] = 0.5;
							thrusters[3] = 0.5;
						}
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
				{
					currentPosition.pose.position.z = 0.6;
					thrusters[0] = -0.5;
					thrusters[1] = -0.5;
				}
				else
				{
					if (posstamped->pose.position.z < -10)
					{
						currentPosition.pose.position.z = 0.3;
						thrusters[0] = -0.5;
						thrusters[1] = -0.5;
					}
					else
					{
						if ((posstamped->pose.position.z > 15) and (posstamped->pose.position.z <= 70))
						{
							currentPosition.pose.position.z = -0.3;
							thrusters[0] = 0.5;
							thrusters[1] = 0.5;
						}
						else
						{
							currentPosition.pose.position.z = -0.6;
							thrusters[0] = 0.5;
							thrusters[1] = 0.5;
						}
					}
				}
			}
			//LeapMotion X-axis -> Robot Y-axis
			if ((posstamped->pose.position.x >= -25.0) and (posstamped->pose.position.x <= 25.0))
				currentPosition.pose.position.x = 0.00;
			else
			{
				if ((posstamped->pose.position.x > 25.0) and (posstamped->pose.position.x <= 90.0))
				{
					currentPosition.pose.position.x = 0.3;
					thrusters[4] = 0.6;
				}
				else
				{
					if (posstamped->pose.position.x > 90.0) //and (posstamped->pose.position.x > 70.0))
					{
						currentPosition.pose.position.x = 0.6;
						thrusters[4] = 0.6;
					}
					else
					{
						if ((posstamped->pose.position.x < -25.0) and (posstamped->pose.position.x <= -90.0))
						{
							currentPosition.pose.position.x = -0.6;
							thrusters[4] = -0.5;
						}
						else
						{
							currentPosition.pose.position.x = -0.3;
							thrusters[4] = -0.5;
						}
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
			if (roll < -0.7)
			{
				currentPosition.pose.orientation.x = -0.4;
				thrusters[0] = 0.25;
				thrusters[1] = -0.25;
			}
			else
			{
				if ((roll >= -0.7) and (roll <= 0.7))
					currentPosition.pose.orientation.x = 0.0;			
				else
				{
					currentPosition.pose.orientation.x = 0.4;
					thrusters[0] = -0.25;
					thrusters[1] = 0.25;
				}
			}
		}//((handsDetected == 1) and rightHand)
	}

	if (accelerations)
	{
		for (int i=0; i<5; i++)
		{
			thrustersMsg.data.push_back(thrusters[i]);
			pub_acc.publish(thrustersMsg);
		}
	}
	else
	{
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
		pub_vel.publish(odom);                        
	}	

	// DEBUG AREA: print hand position and command to send to UWSim
	if (DEBUG_sub_leap)
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


void Uial::spacenavCallback(const geometry_msgs::Twist::ConstPtr& twistValue)
{
	ros::Time currentPressNavDial = ros::Time::now();
	ros::Duration difTime = currentPressNavDial - lastPressNavDial;
	int numMenuAvailable = (sizeof(menuButtonLimits)/sizeof(*menuButtonLimits));

	if (difTime.toSec() > 0.2)
	{
		//We rotate the SpaceNav to move the button focus
		if (twistValue->angular.z > 0.3)
		{
			userMenuData.data[4]--;
			if (userMenuData.data[4] < 0)
				userMenuData.data[4] = 0;
		}
		if (twistValue->angular.z < -0.3)
		{
			if (userMenuData.data[4] < (menuButtonLimits[userMenuData.data[3]] - 1))
				userMenuData.data[4]++;
			else
				userMenuData.data[4] = menuButtonLimits[userMenuData.data[3]] - 1;
		}

		//We press the the SpaceNav to select a new menu
		if (twistValue->linear.z < -0.3)
		{
			if (userMenuData.data[4] == menuButtonLimits[userMenuData.data[3]] - 1)
			{	//Back to main menu
				userMenuData.data[1] = 1;
				if (userMenuData.data[3] >= 0)
				{
					userMenuData.data[3] = 0;
					userMenuData.data[4] = 0;	
				}
			}
			else
				if ((userMenuData.data[4] != 4) and (userMenuData.data[4] != 5))
				{
					userMenuData.data[1] = 1;
					userMenuData.data[3] = userMenuData.data[4] + 1;
					userMenuData.data[4] = 0;
				}
		}
		else
			userMenuData.data[1] = 0;

		lastPressNavDial = currentPressNavDial;
		pub_userMenuData.publish(userMenuData);
	}

	// DEBUG AREA: print hand position and command to send to UWSim
	if (DEBUG_sub_spacenav)
	{
		cout << "SpaceNav angular value: " << twistValue->angular.z << endl;
		cout << "SpaceNav linear  value: " << twistValue->linear.z << endl;
		//cout << "Num: " << (int) userMenuData.data[2] << endl;
		cout << "userMenuData: [" << (int) userMenuData.data[0] << ", " << (int) userMenuData.data[1] << ", " \
			 << (int) userMenuData.data[2] << ", " << (int) userMenuData.data[3] << ", " << (int) userMenuData.data[4] << "]" << endl;
	}
}


void Uial::joystickCallback(const sensor_msgs::Joy::ConstPtr& joystick)
{
	int num;
	double roll, pitch, yaw;
	double thrusters[5], armInput[3];
	sensor_msgs::JointState js;
	std_msgs::Float64MultiArray thrustersMsg, armMsg;
	nav_msgs::Odometry odom;
	vpColVector current_joints(5), send_joints(5);

	//Check if the user press the userControlRequest button
	ros::Time currentPressUserControl = ros::Time::now();
	ros::Duration difTimeUserControl = currentPressUserControl - lastPressUserControl;
	if ((difTimeUserControl.toSec() > 0.5) and (joystick->buttons[0] == 1))
	{
		userControlRequest.data = !userControlRequest.data;
		lastPressUserControl = currentPressUserControl;
	}
	pub_userControlRequest.publish(userControlRequest);

	//Check if the user press the armControlRequest button
	ros::Time currentPressArmControl = ros::Time::now();
	ros::Duration difTimeArmControl = currentPressArmControl - lastPressArmControl;
	if ((difTimeArmControl.toSec() > 0.5) and (joystick->buttons[1] == 1))
	{
		armControlRequest.data = !armControlRequest.data;
		lastPressArmControl = currentPressArmControl;
	}
	pub_armControlRequest.publish(armControlRequest);

	//Clear arrays
	for (int i=0; i<5; i++)
		thrusters[i] = 0.0;
	thrustersMsg.data.clear();

	for (int i=0; i<3; i++)
		armInput[0]	= 0.0;

	
	//Check for the joystick movements
	if ((robotControl) and (userControlRequest.data))
	{
		//joystick X-axis -> Robot X-axis
		if ((joystick->axes[0] <= 0.4) and (joystick->axes[0] >= -0.4))
		{
			currentPosition.pose.position.x = 0.0;
			armInput[0] = 0.0;
		}
		else
		{
			if (joystick->axes[0] > 0.4)
			{
				if (joystick->axes[0] < 0.7)
				{
					currentPosition.pose.position.x = 0.3;
					armInput[0]	= 0.05;						
				}
				else  //(joystick->axes[0] >= 0.7)
				{
					currentPosition.pose.position.x = 0.6;
					armInput[0]	= 0.2;
				}
				thrusters[4] = 0.7;
			}
			else
			{
				if (joystick->axes[0] > -0.7)
				{
					currentPosition.pose.position.x = -0.3;
					armInput[0] = -0.05;
				}
				else  //(joystick->axes[0] <= -0.7)
				{
					currentPosition.pose.position.x = -0.6;
					armInput[0]	= -0.2;
				}
				thrusters[4] = -0.7;
			}
		}
		//joystick Y-axis (lever) -> Robot Y-axis
		if ((joystick->axes[3] <= 0.4) and (joystick->axes[3] >= -0.4))
			currentPosition.pose.position.z = 0.0;
		else
		{
			if (joystick->axes[3] > 0.4)
			{
				//Range sensor
				if ((int) safetyMeasureAlarm.data[2] == 0)
				{
					if (joystick->axes[3] < 0.7)
						currentPosition.pose.position.z = 0.3;
					else
						currentPosition.pose.position.z = 0.6;
					thrusters[2] = -0.7;
					thrusters[3] = -0.7;
				}
				else
				{	//Moves the robot up when it is close to the seafloor
					thrusters[2] = 0.7;
					thrusters[3] = 0.7;
				}
			}
			else
			{
				//Pressure sensor
				if ((int) safetyMeasureAlarm.data[1] == 0)
				{
					if (joystick->axes[3] > -0.7)
							currentPosition.pose.position.z = -0.3;
					else
						currentPosition.pose.position.z = -0.6;
					thrusters[2] = 0.7;
					thrusters[3] = 0.7;
				}
				else
				{	//Moves the robot down when it is close on the surface
					thrusters[2] = -0.7;
					thrusters[3] = -0.7;
				}
			}
		}
		//joystick Z-axis -> Robot Z-axis
		if ((joystick->axes[1] <= 0.4) and (joystick->axes[1] >= -0.4))
		{
			currentPosition.pose.position.y = 0.0;
			armInput[2] = 0.0;
		}
		else
		{
			if (joystick->axes[1] > 0.4)
			{
				if (joystick->axes[1] < 0.7)
				{
					currentPosition.pose.position.y = -0.3;
					armInput[2]	= 0.05;
				}
				else
				{
					currentPosition.pose.position.y = -0.6;
					armInput[0]	= 0.2;
				}
				thrusters[0] = 0.7;
				thrusters[1] = 0.7;
			}
			else
			{
				if (joystick->axes[1] > -0.7)
				{
					currentPosition.pose.position.y = 0.3;
					armInput[2]	= 0.05;
				}
				else
				{
					currentPosition.pose.position.y = 0.6;
					armInput[2]	= -0.2;
				}
				thrusters[0] = -0.7;
				thrusters[1] = -0.7;
			}
				
			if ((int) safetyMeasureAlarm.data[1] == 1)
				cout << "Alarm: robot on surface." << endl;
			if ((int) safetyMeasureAlarm.data[2] == 1)
				cout << "Alarm: robot on seafloor." << endl;
		}
		
		//Rotation
		if ((joystick->axes[2] <= 0.5) and (joystick->axes[2] >= -0.5))
		{
			currentPosition.pose.orientation.z = 0.0;
			armInput[1] = 0.0;
		}
		else
		{
			if (joystick->axes[2] > 0.5)
			{
				currentPosition.pose.orientation.z = 0.3;
				thrusters[0] = -0.4;
				thrusters[1] = 0.4;
				armInput[1]	 = 0.1;
			}
			else
			{
				currentPosition.pose.orientation.z = -0.3;
				thrusters[0] = 0.4;
				thrusters[1] = -0.4;
				armInput[1]	 = -0.1;
			}
		}

		if (!armControlRequest.data)
		{
			if (accelerations)
			{
				for (int i=0; i<5; i++)
					thrustersMsg.data.push_back(thrusters[i]);
				pub_acc.publish(thrustersMsg);
			}
			else
			{
				//Assign the calculated values into the publisher
				odom.twist.twist.linear.x =  currentPosition.pose.position.y;
				odom.twist.twist.linear.y =  currentPosition.pose.position.x;
				odom.twist.twist.linear.z =  currentPosition.pose.position.z;
				odom.twist.twist.angular.x = 0; //roll;
				odom.twist.twist.angular.y = 0; //pitch;
				odom.twist.twist.angular.z = currentPosition.pose.orientation.z; //yaw
				for (int i=0; i<36; i++)
				{
					odom.twist.covariance[i]=0;
					odom.pose.covariance[i]=0;
				}
				pub_vel.publish(odom);
			}
		}
		else
		{		
			for (int i=0; i<3; i++)
				armMsg.data.push_back(armInput[i]);
			pub_arm.publish(armMsg);
		}
	}
}


void Uial::gamepadCallback(const sensor_msgs::Joy::ConstPtr& gamepad)
{
	int num;
	double roll, pitch, yaw;
	double thrusters[5], armInput[3];
	sensor_msgs::JointState js;
	std_msgs::Float64MultiArray thrustersMsg, armMsg;
	nav_msgs::Odometry odom;
	vpColVector current_joints(5), send_joints(5);

	//Check if the user press the userControlRequest button
	ros::Time currentPressUserControl = ros::Time::now();
	ros::Duration difTimeUserControl = currentPressUserControl - lastPressUserControl;
	if ((difTimeUserControl.toSec() > 0.5) and (gamepad->buttons[8] == 1))
	{
		userControlRequest.data = !userControlRequest.data;
		lastPressUserControl = currentPressUserControl;
	}
	pub_userControlRequest.publish(userControlRequest);

	//Check if the user press the armControlRequest button
	ros::Time currentPressArmControl = ros::Time::now();
	ros::Duration difTimeArmControl = currentPressArmControl - lastPressArmControl;
	if ((difTimeArmControl.toSec() > 0.5) and (gamepad->buttons[7] == 1))
	{
		armControlRequest.data = !armControlRequest.data;
		lastPressArmControl = currentPressArmControl;
	}
	pub_armControlRequest.publish(armControlRequest);

	//Clear arrays
	for (int i=0; i<5; i++)
		thrusters[i] = 0.0;
	thrustersMsg.data.clear();

	for (int i=0; i<3; i++)
		armInput[0]	= 0.0;

	
	//Check for the gamepad movements
	if (userControlRequest.data)	//VEHICLE
	{
		//gamepad X-axis -> Robot X-axis
		if ((gamepad->axes[0] <= 0.4) and (gamepad->axes[0] >= -0.4))
			currentPosition.pose.position.x = 0.0;
		else
		{
			if (gamepad->axes[0] > 0.4)
			{
				if (gamepad->axes[0] < 0.7)
					currentPosition.pose.position.x = 0.3;
				else  //(gamepad->axes[0] >= 0.7)
					currentPosition.pose.position.x = 0.6;
				thrusters[4] = 0.7;
			}
			else
			{
				if (gamepad->axes[0] > -0.7)
					currentPosition.pose.position.x = -0.3;
				else  //(gamepad->axes[0] <= -0.7)
					currentPosition.pose.position.x = -0.6;
				thrusters[4] = -0.7;
			}
		}
		//gamepad frontal left lower push button (go down)
		if (gamepad->axes[2] <= -0.5)
			currentPosition.pose.position.z = 0.0;
		else
		{
			//Range sensor
			if ((int) safetyMeasureAlarm.data[2] == 0)
			{
				if (gamepad->axes[2] > 0.5)
					currentPosition.pose.position.z = 0.6;
				else if (gamepad->axes[2] > 0.0)
					currentPosition.pose.position.z = 0.3;
				thrusters[2] = -0.7;
				thrusters[3] = -0.7;
			}
			else
			{	//Moves the robot up when it is close to the seafloor
				thrusters[2] = 0.7;
				thrusters[3] = 0.7;
			}
		}
		//gamepad frontal right lower push button (go up)
		if (gamepad->axes[5] <= -0.5)
			currentPosition.pose.position.z = 0.0;
		else
		{
			//Pressure sensor
			if ((int) safetyMeasureAlarm.data[1] == 0)
			{
				if (gamepad->axes[5] > 0.5)
						currentPosition.pose.position.z = -0.6;
				else if (gamepad->axes[5] > 0.0)
					currentPosition.pose.position.z = -0.3;
				thrusters[2] = 0.7;
				thrusters[3] = 0.7;
			}
			else
			{	//Moves the robot down when it is close on the surface
				thrusters[2] = -0.7;
				thrusters[3] = -0.7;
			}
		}
		//gamepad Z-axis -> Robot Z-axis
		if ((gamepad->axes[1] <= 0.4) and (gamepad->axes[1] >= -0.4))
			currentPosition.pose.position.y = 0.0;
		else
		{
			if (gamepad->axes[1] > 0.4)
			{
				if (gamepad->axes[1] < 0.7)
					currentPosition.pose.position.y = -0.3;
				else
					currentPosition.pose.position.y = -0.6;
				thrusters[0] = 0.7;
				thrusters[1] = 0.7;
			}
			else
			{
				if (gamepad->axes[1] > -0.7)
					currentPosition.pose.position.y = 0.3;
				else
					currentPosition.pose.position.y = 0.6;
				thrusters[0] = -0.7;
				thrusters[1] = -0.7;
			}
				
			if ((int) safetyMeasureAlarm.data[1] == 1)
				cout << "Alarm: robot on surface." << endl;
			if ((int) safetyMeasureAlarm.data[2] == 1)
				cout << "Alarm: robot on seafloor." << endl;
		}
		
		//Robot rotation using bottom front buttons
		if ((gamepad->buttons[4] == 0) and (gamepad->buttons[5] == 0))
			currentPosition.pose.orientation.z = 0.0;
		else
		{
			if (gamepad->buttons[4] == 1)
			{
				currentPosition.pose.orientation.z = -0.3;
				thrusters[0] = 0.4;
				thrusters[1] = -0.4;
			}
			if (gamepad->buttons[5] == 1)
			{
				currentPosition.pose.orientation.z = 0.3;
				thrusters[0] = -0.4;
				thrusters[1] = 0.4;
			}
		}

		if (accelerations)
		{
			for (int i=0; i<5; i++)
				thrustersMsg.data.push_back(thrusters[i]);
			pub_acc.publish(thrustersMsg);
		}
		else
		{
			//Assign the calculated values into the publisher
			odom.twist.twist.linear.x =  currentPosition.pose.position.y;
			odom.twist.twist.linear.y =  currentPosition.pose.position.x;
			odom.twist.twist.linear.z =  currentPosition.pose.position.z;
			odom.twist.twist.angular.x = 0; //roll;
			odom.twist.twist.angular.y = 0; //pitch;
			odom.twist.twist.angular.z = currentPosition.pose.orientation.z; //yaw
			for (int i=0; i<36; i++)
			{
				odom.twist.covariance[i]=0;
				odom.pose.covariance[i]=0;
			}
			pub_vel.publish(odom);
		}
	}

	if (armControlRequest.data)		//ARM
	{
		if ((gamepad->axes[3] <= 0.4) and (gamepad->axes[3] >= -0.4))
			armInput[0] = 0.0;
		else
		{
			if (gamepad->axes[3] > 0.4)
			{
				if (gamepad->axes[3] < 0.7)
					armInput[0]	= 0.05;
				else  //(gamepad->axes[3] >= 0.7)
					armInput[0]	= 0.2;
			}
			else
			{
				if (gamepad->axes[3] > -0.7)
					armInput[0]	= -0.05;
				else  //(gamepad->axes[3] <= -0.7)
					armInput[0]	= -0.2;
			}
		}
		if ((gamepad->axes[4] <= 0.4) and (gamepad->axes[4] >= -0.4))
			armInput[2]	= 0.0;
		else
		{
			if (gamepad->axes[4] > 0.4)
			{
				if (gamepad->axes[4] < 0.7)
					armInput[2]	= 0.05;
				else  //(gamepad->axes[4] >= 0.7)
					armInput[2]	= 0.2;
			}
			else
			{
				if (gamepad->axes[4] > -0.7)
					armInput[2]	= -0.05;
				else  //(gamepad->axes[4] <= -0.7)
					armInput[2]	= -0.2;
			}
		}
		//Button1 & Button2 controls the arm rotation
		armInput[1]	= 0.0;
		if (gamepad->buttons[1] == 1) //Right
			armInput[1]	= 0.2;
		if (gamepad->buttons[2] == 1) //Left
			armInput[1]	= -0.2;


		if (armControlRequest.data)
		{		
			for (int i=0; i<3; i++)
				armMsg.data.push_back(armInput[i]);
			pub_arm.publish(armMsg);
		}
	}


	// DEBUG AREA: print hand position and command to send to UWSim
	if (DEBUG_sub_gamepad)
	{
		cout << "Accelerations activated: " << accelerations << endl;
		cout << "Gamepad values for vehicle: (" << gamepad->axes[0] << ", " << gamepad->axes[1] <<  \
		" :: " << gamepad->axes[2] << ", " << gamepad->axes[5] << ")" << endl;
		cout << "Gamepad values for arm: (" << gamepad->axes[3] << ", " << gamepad->axes[4]  << ")" << endl;
		cout << "thrusters values: (" << thrusters[0] << ", " << thrusters[1] << ", " << thrusters[2] <<\
				", " << thrusters[3] << ", " << thrusters[4] << ")" << endl;
		cout << "arm values: (" << armInput[0] << ", " << armInput[1] << ", " << armInput[2] << ")" << endl;
		cout << "sensorPressureAlarm: " << (int) safetyMeasureAlarm.data[1] << ". sensorRangeAlarm: " << (int) safetyMeasureAlarm.data[2] << endl;
		cout << "safetyMeasureAlarm: [";
		for (int i=0; i<=num_sensors; i++)
			cout << (int) safetyMeasureAlarm.data[i] << ",";
		cout << "]" << endl;
		cout << "userControlAlarm: [" << (int) userControlAlarm.data[0] << ", " << (int) userControlAlarm.data[1] << "]" << endl;
		cout << "userControlRequest: " << (int) userControlRequest.data << endl;
		cout << "armControlRequest: " << (int) armControlRequest.data << endl;
	}
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "uial");
  Uial uial_control;
  ros::spin();
}

