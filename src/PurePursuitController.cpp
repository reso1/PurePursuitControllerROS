/******************************************************************************
 * Copyright (C) 2014 by Todd Tang                                          *
 * todd.j.tang@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include "curio_navigation/PurePursuitController.h"

#include <cmath>

#include <geometry_msgs/Twist.h>

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

PurePursuitController::PurePursuitController(const ros::NodeHandle& nh) 
	: _nodeHandle(nh)
	, _nextWayPoint(-1) 
{
	getParameters();

	initMessages();

	_pathSubscriber = _nodeHandle.subscribe(
		_pathTopicName, _queueDepth,
		&PurePursuitController::pathCallback, this);

	_odometrySubscriber = _nodeHandle.subscribe(
		_odometryTopicName, _queueDepth,
		&PurePursuitController::odometryCallback, this);
		
	_cmdVelocityPublisher = _nodeHandle.advertise<geometry_msgs::Twist>(
		_cmdVelocityTopicName, _queueDepth);

	_targetWayPointPublisher = _nodeHandle.advertise<visualization_msgs::Marker>(
		_targetWayPointPubTopicName, _queueDepth);

	_timer = _nodeHandle.createTimer(ros::Duration(1.0/_control_frequency),
		&PurePursuitController::timerCallback, this);
	
}

PurePursuitController::~PurePursuitController() 
{

}

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

void PurePursuitController::spin() 
{	
	ros::spin();
}


void PurePursuitController::pathCallback(const nav_msgs::Path& msg) 
{
	if (_nextWayPoint == -1)
	{
		_refPathFrameId = msg.header.frame_id;
		_currentReferencePath = msg;
		_refPathLength =  _currentReferencePath.poses.size();
		ROS_INFO_STREAM("Received reference path of size " << _refPathLength);
		_nextWayPoint = getClosestWayPoint();
		ROS_INFO_STREAM("Start Pure-Pursuit from WayPoint #"<< _nextWayPoint);
		_pathEndPose = _currentReferencePath.poses.back().pose;
	}
}


void PurePursuitController::odometryCallback(const nav_msgs::Odometry& msg) 
{	
	_currentPose.header = msg.header;
	_currentPose.pose = msg.pose.pose;
	_currentVelocity = msg.twist.twist;
}


void PurePursuitController::timerCallback(const ros::TimerEvent& event)
{
	if (_nextWayPoint == -1) {
		ROS_WARN_ONCE("No reference path received, will keep waiting...");
		return;
	}
	
	geometry_msgs::Point currentPosition = getCurrentPose().pose.position;
	tf::Vector3 v1(currentPosition.x, currentPosition.y, currentPosition.z);
	tf::Vector3 v2(_pathEndPose.position.x, _pathEndPose.position.y, _pathEndPose.position.z);
	
	if (tf::tfDistance(v1, v2) < 1.0f) {
		ROS_INFO_STREAM_ONCE("Path Following Finished...shutdown");
		ros::shutdown();
	}

	geometry_msgs::Twist cmdVelocity;

	if (step(cmdVelocity)) {
		_cmdVelocityPublisher.publish(cmdVelocity);
	}
}


bool PurePursuitController::step(geometry_msgs::Twist& twist) 
{
	twist.linear.x = 0.0;
	twist.linear.y = 0.0;
	twist.linear.z = 0.0;

	twist.angular.x = 0.0;
	twist.angular.y = 0.0;
	twist.angular.z = 0.0;

	_nextWayPoint = getNextWayPoint();

	if (_nextWayPoint >= 0) 
	{
		geometry_msgs::PoseStamped target = _currentReferencePath.poses[_nextWayPoint];

		_target_marker_msg.pose = target.pose;
		_targetWayPointPublisher.publish(_target_marker_msg);

		// ld: delta-distance from current pose to goal pose
		double l_t = getLookAheadThreshold();

		geometry_msgs::PoseStamped origin = getCurrentPose();
		double dy = target.pose.position.y - origin.pose.position.y;
		double dx = target.pose.position.x - origin.pose.position.x;
		// alpha: delta-angle from current pose to goal pose
		double alpha = atan2(dy, dx) - tf::getYaw(origin.pose.orientation);

		double angularVelocity = 0.0;

		if (std::abs(std::sin(alpha)) >= _epsilon) 
		{
			// r = ld / (2 * sin(alpha))
			double radius = 0.5 * (l_t / std::sin(alpha));
			
			// try to reach the desired linear velocity
			double linearVelocity = _velocity;

			// only steering when delta_angle is larger than error_threshold _epsilon
			if (std::abs(radius) >= _epsilon)
				// omega = v / r
				angularVelocity = linearVelocity / radius;

			twist.linear.x = linearVelocity;
			twist.angular.z = angularVelocity;
			
		}

		return true;
	}

	return false;
}


geometry_msgs::PoseStamped PurePursuitController::getCurrentPose() const 
{
	geometry_msgs::PoseStamped transformedPose;
	
	try {
		_tfListener.transformPose(_refPathFrameId, _currentPose, transformedPose);
	}
	catch (tf::TransformException& exception) {
		ROS_ERROR_STREAM("PurePursuitController::getCurrentPose: " << 
		exception.what());
	}

	// get current vehicle pose in ReferencePath frame
	return transformedPose;
}

double PurePursuitController::getDistanceToPose(
	const geometry_msgs::PoseStamped& pose) const 
{
	geometry_msgs::PoseStamped origin = getCurrentPose();
	geometry_msgs::PoseStamped transformedPose;

	try {
		_tfListener.transformPose(_refPathFrameId, pose, transformedPose);
	}
	catch (tf::TransformException& exception) {
		ROS_ERROR_STREAM("PurePursuitController::getDistanceToPose: " << 
		exception.what());
		
		return -1.0;
	}

	tf::Vector3 v1(origin.pose.position.x,
					origin.pose.position.y,
					origin.pose.position.z);
	tf::Vector3 v2(transformedPose.pose.position.x,
					transformedPose.pose.position.y,
					transformedPose.pose.position.z);

	// Distance from current vehicle pose to current waypoint in ReferencePath frame
	return tf::tfDistance(v1, v2);
}

double PurePursuitController::getAngleToPose(
	const geometry_msgs::PoseStamped& pose) const 
{
	geometry_msgs::PoseStamped origin = getCurrentPose();
	geometry_msgs::PoseStamped transformedPose;

	try {
		_tfListener.transformPose(_refPathFrameId, pose, transformedPose);
	}
	catch (tf::TransformException& exception) {
		ROS_ERROR_STREAM("PurePursuitController::getAngleToPose: " << 
		exception.what());
		
		return -1.0;
	}

	tf::Vector3 v1(origin.pose.position.x,
					origin.pose.position.y,
					origin.pose.position.z);
	tf::Vector3 v2(transformedPose.pose.position.x,
					transformedPose.pose.position.y,
					transformedPose.pose.position.z);

	// Delta Angle from current vehilce pose to current waypoint in ReferencePath frame
	return tf::tfAngle(v1, v2);
}


double PurePursuitController::getLookAheadThreshold() const 
{
	return _lookAheadRatio * _currentVelocity.linear.x + _lookAheadConstant;
}


int PurePursuitController::getNextWayPoint() 
{   
	int wayPoint = _nextWayPoint;
	if (!_currentReferencePath.poses.empty()) {
		if (wayPoint >= 0) {
			double thisDistance = getDistanceToPose(_currentReferencePath.poses[wayPoint]);
	
			while (wayPoint < _refPathLength-1)
			{
				double nextDistance = getDistanceToPose(_currentReferencePath.poses[wayPoint+1]);
				if (thisDistance < nextDistance)
					break;
				wayPoint = wayPoint + 1;
				thisDistance = nextDistance;
			}

			while (getLookAheadThreshold() > getDistanceToPose(_currentReferencePath.poses[wayPoint]))
			{
				if (wayPoint + 1 >= _refPathLength)
					break;

				wayPoint = wayPoint + 1;
			}

			return wayPoint;
		}
		else
			return 0;

	}

	return -1;
}


int PurePursuitController::getClosestWayPoint() const {
	if (!_currentReferencePath.poses.empty()) {
		int closestWaypoint = 0;
		double minDistance = getDistanceToPose(_currentReferencePath.poses.front());
		
		for (int i = closestWaypoint+1; i < _refPathLength; ++i) {
			double distance = getDistanceToPose(_currentReferencePath.poses[i]);
		
			if (distance < minDistance) {
				closestWaypoint = i;
				minDistance = distance;
			}
		}

		return closestWaypoint;
	}

	return -1;
}


void PurePursuitController::getParameters() 
{
	std::string param_ns = "pure_pursuit_controller";

	_nodeHandle.param<std::string>(param_ns + "ref_path_topic_name", 
		_pathTopicName, "/reference_path");
	
	_nodeHandle.param<std::string>(param_ns + "odom_topic_name",
		_odometryTopicName, "/odometry/filtered_map");
	
	_nodeHandle.param<std::string>(param_ns + "cmd_vel_topic_name",
		_cmdVelocityTopicName, "/ackermann_drive_controller/cmd_vel");

	_nodeHandle.param<std::string>("interp_waypoint_topic_name",
		_targetWayPointPubTopicName, "/interpolated_waypoint_pose");
	
	_nodeHandle.param<std::string>("pose_frame_id", _poseFrameId, "base_link");

	_nodeHandle.param<int>(param_ns + "queue_depth", _queueDepth, 100);
	
	_nodeHandle.param<double>(param_ns + "control_frequency", _control_frequency, 20.0);
	
	_nodeHandle.param<double>(param_ns + "target_velocity", _velocity, 0.2);
	
	_nodeHandle.param<double>(param_ns + "look_ahead_ratio", _lookAheadRatio, 0.1);

	_nodeHandle.param<double>(param_ns + "look_ahead_constant", _lookAheadConstant, 1.0);
	
	_nodeHandle.param<double>(param_ns + "epsilon", _epsilon, 1e-6);
}


void PurePursuitController::initMessages()
{
	// targer marker message
	_target_marker_msg.header.frame_id = "map";
	_target_marker_msg.type = visualization_msgs::Marker::SPHERE;
	_target_marker_msg.action = visualization_msgs::Marker::ADD;
	_target_marker_msg.scale.x = 0.2;
	_target_marker_msg.scale.y = 0.2;
	_target_marker_msg.scale.z = 0.2;
	_target_marker_msg.color.r = 0.5;
	_target_marker_msg.color.g = 0;
	_target_marker_msg.color.b = 1;
	_target_marker_msg.color.a = 1;
}