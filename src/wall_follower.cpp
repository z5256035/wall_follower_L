// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim
//
// Modified by Claude Sammut for COMP3431
// Use this code as the basis for a wall follower

#include "wall_follower/wall_follower.hpp"

#include <memory>
#include <fstream>


using namespace std::chrono_literals;

WallFollower::WallFollower()
: Node("wall_follower_node")
{
	/************************************************************
	** Initialise variables
	************************************************************/
	for (int i = 0; i < 12; i++)
		scan_data_[i] = 0.0;

	robot_pose_ = 0.0;
	near_start = false;

	/************************************************************
	** Initialise ROS publishers and subscribers
	************************************************************/
	auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

	// Initialise publishers
	cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

	// Initialise subscribers
	scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"scan", \
		rclcpp::SensorDataQoS(), \
		std::bind(
			&WallFollower::scan_callback, \
			this, \
			std::placeholders::_1));
	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom", qos, std::bind(&WallFollower::odom_callback, this, std::placeholders::_1));

	/************************************************************
	** Initialise ROS timers
	************************************************************/
	update_timer_ = this->create_wall_timer(10ms, std::bind(&WallFollower::update_callback, this));

	RCLCPP_INFO(this->get_logger(), "Wall follower node has been initialised - Gurveer");
	RCLCPP_INFO(this->get_logger(), "Wall follower node has been initialised");
}

WallFollower::~WallFollower()
{
	RCLCPP_INFO(this->get_logger(), "Wall follower node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/

#define START_RANGE	0.2

void WallFollower::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	static bool first = true;
	static bool start_moving = true;

	tf2::Quaternion q(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	robot_pose_ = yaw;

	double current_x =  msg->pose.pose.position.x;
	double current_y =  msg->pose.pose.position.y;
	if (first)
	{
        RCLCPP_INFO(this->get_logger(), "hi there");
		start_x = current_x;
		start_y = current_y;
		first = false;
        std::ofstream f;
        f.open("./out_wall.txt");
        f << start_x << ", " << start_y << ", " << yaw << std::endl;
        f.close();

	}
	else if (start_moving)
	{
		if (fabs(current_x - start_x) > START_RANGE || fabs(current_y - start_y) > START_RANGE)
			start_moving = false;
	}
	else if (fabs(current_x - start_x) < START_RANGE && fabs(current_y - start_y) < START_RANGE)
	{
		fprintf(stderr, "Near start Woohoo!!\n");
		near_start = true;
		first = true;
		start_moving = true;
	}
}

#define BEAM_WIDTH 5

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	uint16_t scan_angle[12] = {0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330};

	double closest = msg->range_max;
	for (int angle = 360-BEAM_WIDTH; angle < 360; angle++)
		if (msg->ranges.at(angle) < closest && msg->ranges.at(angle) != 0)
			closest = msg->ranges.at(angle);
	for (int angle = 0; angle < BEAM_WIDTH; angle++)
		if (msg->ranges.at(angle) < closest && msg->ranges.at(angle) != 0)
			closest = msg->ranges.at(angle);
	scan_data_[0] = closest;

	for (int i = 1; i < 12; i++)
	{
		closest = msg->range_max;
		for (int angle = scan_angle[i]-BEAM_WIDTH; angle < scan_angle[i]+BEAM_WIDTH; angle++)
			if (msg->ranges.at(angle) < closest  && msg->ranges.at(angle) != 0)
				closest = msg->ranges.at(angle);
		scan_data_[i] = closest;
	}
}

void WallFollower::update_cmd_vel(double linear, double angular)
{
	geometry_msgs::msg::Twist cmd_vel;
	cmd_vel.linear.x = linear;
	cmd_vel.angular.z = angular;

	cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/

bool pl_near;

int maxCount = 0;
int currentCount = 0;
double linearSpeed = 0.0;
double angularSpeed = 0.0;

bool close_to_left = false;
bool left_outer_turn = false;

void WallFollower::update_callback()
{
    // if (near_start) update_cmd_vel(0.0, 0.0);
    // else if (scan_data_[FRONT] < 0.35) update_cmd_vel(0, -0.8);
    // else if (scan_data_[LEFT_FRONT] > 0.7) {
    // // if (scan_data_[LEFT_BACK] < 0.1) update_cmd_vel(0.05, 0.0);
    // update_cmd_vel(0.05, 0.3);
    // }
    // else if (scan_data_[LEFT_FRONT] < 0.3) update_cmd_vel(0.05, -0.3);
    // else if (scan_data_[FRONT] > 0.6) update_cmd_vel(0.15, 0.0);
    // // else if (scan_data_[LEFT_FRONT] > 0.4) update_cmd_vel(0.1, 0.5);
    // else if (scan_data_[FRONT] < 0.25) update_cmd_vel(0.01, -0.5);
    // else if (scan_data_[LEFT] < 0.1) update_cmd_vel(0.01, -0.5);
    // // else if (scan_data_[RIGHT] < 0.4) update_cmd_vel(0.05, 0.35);
    // // else if (scan_data_[LEFT] > 0.4) update_cmd_vel(0.07, 0.3);
    // // else if (scan_data_[LEFT_BACK] > 0.4) update_cmd_vel(0.07, 0.4);
    // else update_cmd_vel(0.1,0.0);

    /*
    #define FRONT 0
    #define FRONT_LEFT 1
    #define LEFT_FRONT 2
    #define LEFT 3
    #define LEFT_BACK 4
    #define BACK_LEFT 5
    #define BACK 6
    #define BACK_RIGHT 7
    #define RIGHT_BACK 8
    #define RIGHT 9
    #define RIGHT_FRONT 10
    #define FRONT_RIGHT 11
    FRONT_LEFT is 30 degrees to the left from the front
    LEFT_FRONT is 30 degrees to the front from the left (i.e 60 degrees)
    */
    // RCLCPP_INFO(this->get_logger(), "FRONT %lf", scan_data_[FRONT]);
    // RCLCPP_INFO(this->get_logger(), "FRONT_LEFT %lf", scan_data_[FRONT_LEFT]);
    // RCLCPP_INFO(this->get_logger(), "FRONT_RIGHT %lf", scan_data_[FRONT_RIGHT]);

    // RCLCPP_INFO(this->get_logger(), "LEFT %lf", scan_data_[LEFT]);
    // RCLCPP_INFO(this->get_logger(), "LEFT_FRONT %lf", scan_data_[LEFT_FRONT]);
    // RCLCPP_INFO(this->get_logger(), "LEFT_BACK %lf", scan_data_[LEFT_BACK]);

    // RCLCPP_INFO(this->get_logger(), "RIGHT %lf", scan_data_[RIGHT]);
    // RCLCPP_INFO(this->get_logger(), "RIGHT_FRONT %lf", scan_data_[RIGHT_FRONT]);
    // RCLCPP_INFO(this->get_logger(), "RIGHT_BACK %lf", scan_data_[RIGHT_BACK]);

    // RCLCPP_INFO(this->get_logger(), "BACK %lf", scan_data_[BACK]);
    // RCLCPP_INFO(this->get_logger(), "BACK_LEFT %lf", scan_data_[BACK_LEFT]);
    // RCLCPP_INFO(this->get_logger(), "BACK_RIGHT %lf", scan_data_[BACK_RIGHT]);

    double lower_radius = 0.2;
    double upper_radius = 0.35;
    // If at the start, stop.
    if (near_start) {
        RCLCPP_INFO(this->get_logger(), "Near start, stopping...");
        // std::ofstream f;
        // f.open("./out_wall.txt");
        // f << start_x << ", " << start_y << ", " << yaw << std::endl;
        // f.close();
        //std::cout << "Near start, stopping..." << std::endl;
        update_cmd_vel(0.0, 0.0);
        return;
    }

    if (close_to_left) {
        if (scan_data_[FRONT] < 0.45
        || scan_data_[FRONT_LEFT] < 0.45
        || scan_data_[FRONT_RIGHT] < 0.45) {
            // RCLCPP_INFO(this->get_logger(), "Too close to front");
            close_to_left = false;
            return;
        }

		if (scan_data_[LEFT_FRONT] < lower_radius) {
			// RCLCPP_INFO(this->get_logger(), "Turning right on the spot...");
			update_cmd_vel(0.0, -0.1);
			return;
		}

        if (scan_data_[LEFT] < lower_radius) {
        // RCLCPP_INFO(this->get_logger(), "Panning out...");
        update_cmd_vel(0.07, -0.1);
        return;
        } 
		
		if (scan_data_[LEFT] > upper_radius) {
        close_to_left = false;
        return;
        }

        double left_front = scan_data_[LEFT_FRONT];
        double left_back = scan_data_[LEFT_BACK];

        if ((left_front - left_back) < -0.02) {
			// RCLCPP_INFO(this->get_logger(), "Left back is too big, panning out...");
			update_cmd_vel(0.07, -0.1);
			return;
        }
		
		if (scan_data_[LEFT] > upper_radius && scan_data_[LEFT_BACK] < upper_radius) {
            close_to_left = false;
            return;
        } 
		
		if ((left_back - left_front) < -0.02) {
			// RCLCPP_INFO(this->get_logger(), "Left front is too big, panning in...");
			update_cmd_vel(0.07, 0.1);
			return;
        }


        close_to_left = false;
        return;
    }

    if (left_outer_turn) {
        if (scan_data_[FRONT] < 0.3
        || scan_data_[FRONT_LEFT] < 0.3
        || scan_data_[FRONT_RIGHT] < 0.3) {
            // RCLCPP_INFO(this->get_logger(), "Too close to front");
            left_outer_turn = false;
            return;
        }

        if (scan_data_[LEFT] > upper_radius || scan_data_[LEFT_FRONT] > upper_radius) {
            // RCLCPP_INFO(this->get_logger(), "No wall on left, panning in...");
            update_cmd_vel(0.07, 0.3);
            return;
        }

        left_outer_turn = false;
        return;
    }
    // if (currentCount < maxCount) {
    // update_cmd_vel(linearSpeed, angularSpeed);
    // currentCount++;
    // return;
    // }

    // If too close to the back then move forward
    if (scan_data_[BACK] < 0.2
    || (scan_data_[BACK_LEFT] < 0.23
    || scan_data_[BACK_RIGHT] < 0.2)) {
        // RCLCPP_INFO(this->get_logger(), "BACK %lf", scan_data_[BACK]);
        // RCLCPP_INFO(this->get_logger(), "BACK_LEFT %lf", scan_data_[BACK_LEFT]);
        // RCLCPP_INFO(this->get_logger(), "BACK_RIGHT %lf", scan_data_[BACK_RIGHT]);
        // RCLCPP_INFO(this->get_logger(), "Too close to back, moving forward...");
        update_cmd_vel(0.03, 0.0);
        return;
    }

    // If too close to the front then turn right
    if (scan_data_[FRONT] < 0.45
    || (scan_data_[FRONT_LEFT] < 0.45
    || scan_data_[FRONT_RIGHT] < 0.45)) {
        // RCLCPP_INFO(this->get_logger(), "FRONT %lf", scan_data_[FRONT]);
        // RCLCPP_INFO(this->get_logger(), "FRONT_LEFT %lf", scan_data_[FRONT_LEFT]);
        // RCLCPP_INFO(this->get_logger(), "FRONT_RIGHT %lf", scan_data_[FRONT_RIGHT]);
        close_to_left = false;
        left_outer_turn = false;
        // RCLCPP_INFO(this->get_logger(), "Too close to front, Turning right...");
        update_cmd_vel(0.0, -0.2);
        return;
    }


    // // If too close to the back then come forward, slowly
    // if (scan_data_[BACK] < lower_radius
    //    || scan_data_[BACK_LEFT] < lower_radius
    //    || scan_data_[BACK_RIGHT] < lower_radius) {
    //     RCLCPP_INFO(this->get_logger(), "Too close to back, forwarding...");
    //     //std::cout << "Too close to back, forwarding..." << std::endl;
    // update_cmd_vel(0.03, 0.0);
    // return;
    // }

    if (scan_data_[LEFT] < lower_radius) {
        // RCLCPP_INFO(this->get_logger(), "Too close to left, panning out...");
        update_cmd_vel(0.03, -0.1);
        close_to_left = true;
        return;
    }



    // If there is a wall in BACK_LEFT but not
    // immediate LEFT, then do a 90 to that wall
    if (scan_data_[LEFT] > upper_radius
    && scan_data_[LEFT_BACK] < 0.5) {
        // RCLCPP_INFO(this->get_logger(), "No wall on left, turning left...");
        left_outer_turn = true;
        return;
    }

    // If too far from left wall, pan in
    if (scan_data_[LEFT_FRONT] > upper_radius
    || scan_data_[LEFT] > upper_radius
    || scan_data_[LEFT_BACK] > upper_radius) {
        // RCLCPP_INFO(this->get_logger(), "LEFT %lf", scan_data_[LEFT]);
        // RCLCPP_INFO(this->get_logger(), "LEFT_FRONT %lf", scan_data_[LEFT_FRONT]);
        // RCLCPP_INFO(this->get_logger(), "LEFT_BACK %lf", scan_data_[LEFT_BACK]);

        // RCLCPP_INFO(this->get_logger(), "Too far from left, panning in...");
        update_cmd_vel(0.07, 0.1);
        return;
    }

    // No obstruction, walk forward
    // RCLCPP_INFO(this->get_logger(), "No obstruction, forwarding...");
    update_cmd_vel(0.25, 0.0);
    return;
}



/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WallFollower>());
	rclcpp::shutdown();

	return 0;
}
