// // Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
// // Author: Claude Sammut
// // Last Modified: 2024.10.14

// // Use this code as the basis for a wall follower

// #include "wall_follower/wall_follower.hpp"

// #include <memory>


// using namespace std::chrono_literals;

// WallFollower::WallFollower()
// : Node("wall_follower_node")
// {
// 	/************************************************************
// 	** Initialise variables
// 	************************************************************/
// 	for (int i = 0; i < 12; i++)
// 		scan_data_[i] = 0.0;

// 	robot_pose_ = 0.0;
// 	near_start = false;

// 	/************************************************************
// 	** Initialise ROS publishers and subscribers
// 	************************************************************/
// 	auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

// 	// Initialise publishers
// 	cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

// 	// Initialise subscribers
// 	scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
// 		"scan", \
// 		rclcpp::SensorDataQoS(), \
// 		std::bind(
// 			&WallFollower::scan_callback, \
// 			this, \
// 			std::placeholders::_1));
// 	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
// 		"odom", qos, std::bind(&WallFollower::odom_callback, this, std::placeholders::_1));

// 	/************************************************************
// 	** Initialise ROS timers
// 	************************************************************/
// 	update_timer_ = this->create_wall_timer(10ms, std::bind(&WallFollower::update_callback, this));

// 	RCLCPP_INFO(this->get_logger(), "Wall follower node has been initialised");
// }

// WallFollower::~WallFollower()
// {
// 	RCLCPP_INFO(this->get_logger(), "Wall follower node has been terminated");
// }

// /********************************************************************************
// ** Callback functions for ROS subscribers
// ********************************************************************************/

// #define START_RANGE	0.2

// void WallFollower::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
// {
// 	static bool first = true;
// 	static bool start_moving = true;

// 	tf2::Quaternion q(
// 		msg->pose.pose.orientation.x,
// 		msg->pose.pose.orientation.y,
// 		msg->pose.pose.orientation.z,
// 		msg->pose.pose.orientation.w);
// 	tf2::Matrix3x3 m(q);
// 	double roll, pitch, yaw;
// 	m.getRPY(roll, pitch, yaw);

// 	robot_pose_ = yaw;

// 	double current_x =  msg->pose.pose.position.x;
// 	double current_y =  msg->pose.pose.position.y;
// 	if (first)
// 	{
// 		start_x = current_x;
// 		start_y = current_y;
// 		first = false;
// 	}
// 	else if (start_moving)
// 	{
// 		if (fabs(current_x - start_x) > START_RANGE || fabs(current_y - start_y) > START_RANGE)
// 			start_moving = false;
// 	}
// 	else if (fabs(current_x - start_x) < START_RANGE && fabs(current_y - start_y) < START_RANGE)
// 	{
// 		fprintf(stderr, "Near start!!\n");
// 		near_start = true;
// 		first = true;
// 		start_moving = true;
// 	}
// }

// #define BEAM_WIDTH 15

// void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
// {
// 	uint16_t scan_angle[12] = {0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330};

// 	double closest = msg->range_max;
// 	for (int angle = 360-BEAM_WIDTH; angle < 360; angle++)
// 		if (msg->ranges.at(angle) < closest)
// 			closest = msg->ranges.at(angle);
// 	for (int angle = 0; angle < BEAM_WIDTH; angle++)
// 		if (msg->ranges.at(angle) < closest)
// 			closest = msg->ranges.at(angle);
// 	scan_data_[0] = closest;

// 	for (int i = 1; i < 12; i++)
// 	{
// 		closest = msg->range_max;
// 		for (int angle = scan_angle[i]-BEAM_WIDTH; angle < scan_angle[i]+BEAM_WIDTH; angle++)
// 			if (msg->ranges.at(angle) < closest)
// 				closest = msg->ranges.at(angle);
// 		scan_data_[i] = closest;
// 	}
// }

// void WallFollower::update_cmd_vel(double linear, double angular)
// {
// 	geometry_msgs::msg::Twist cmd_vel;
// 	cmd_vel.linear.x = linear;
// 	cmd_vel.angular.z = angular;

// 	cmd_vel_pub_->publish(cmd_vel);
// }

// /********************************************************************************
// ** Update functions
// ********************************************************************************/

// bool pl_near;


// void WallFollower::update_callback()
// {
// 	// if (near_start) {update_cmd_vel(0.0, 0.0); exit(0);}
// 	// else if (scan_data_[LEFT_FRONT] > 0.9) update_cmd_vel(0.2, 1.5);
// 	// else if (scan_data_[FRONT] < 0.7) update_cmd_vel(0.0, -1.5);
// 	// else if (scan_data_[FRONT_LEFT] < 0.6) update_cmd_vel(0.3, -1.5);
// 	// else if (scan_data_[FRONT_RIGHT] < 0.6) update_cmd_vel(0.3, 1.5);
// 	// else if (scan_data_[LEFT_FRONT] > 0.6) update_cmd_vel(0.3, 1.5);
// 	// else update_cmd_vel(0.3, 0.0);
// }



// /*******************************************************************************
// ** Main
// *******************************************************************************/
// int main(int argc, char ** argv)
// {
// 	rclcpp::init(argc, argv);
// 	rclcpp::spin(std::make_shared<WallFollower>());
// 	rclcpp::shutdown();

// 	return 0;
// }


// Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
// Author: Claude Sammut
// Modified: 2025-10-16 (PID version)

#include "wall_follower/wall_follower.hpp"
#include <memory>
#include <sstream>
#include <algorithm>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

WallFollower::WallFollower()
: Node("wall_follower_node")
{
	/************************************************************
	** Initialise variables
	************************************************************/
	for (int i = 0; i < 12; i++)
		scan_data_[i] = 0.0;

    /************************************************************
    ** ROS setup
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

	RCLCPP_INFO(this->get_logger(), "Wall follower node has been initialised");
}

WallFollower::~WallFollower()
{
    RCLCPP_INFO(this->get_logger(), "Wall follower node terminated");
}

/********************************************************************************
** Odometry
********************************************************************************/

#define START_RANGE	0.4

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

    double current_x = msg->pose.pose.position.x;

	double current_y =  msg->pose.pose.position.y;
	if (first)
	{
		start_x = current_x;
		start_y = current_y;
		first = false;
	}
	else if (start_moving)
	{
		if (fabs(current_x - start_x) > START_RANGE || fabs(current_y - start_y) > START_RANGE)
			start_moving = false;
	}
	else if (fabs(current_x - start_x) < START_RANGE && fabs(current_y - start_y) < START_RANGE)
	{
		fprintf(stderr, "Near start!!\n");
		// near_start = true;
		first = true;
		start_moving = true;
	}
	
	
}

/********************************************************************************
** Laser Scan
********************************************************************************/
#define BEAM_WIDTH 15

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	uint16_t scan_angle[12] = {0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330};

	double closest = msg->range_max;
	for (int angle = 360-BEAM_WIDTH; angle < 360; angle++)
		if (msg->ranges.at(angle) < closest && msg->ranges.at(angle) > 0)
			closest = msg->ranges.at(angle);
	for (int angle = 0; angle < BEAM_WIDTH; angle++)
		if (msg->ranges.at(angle) < closest && msg->ranges.at(angle) > 0)
			closest = msg->ranges.at(angle);
	scan_data_[0] = closest;

	for (int i = 1; i < 12; i++)
	{
		closest = msg->range_max;
		for (int angle = scan_angle[i]-BEAM_WIDTH; angle < scan_angle[i]+BEAM_WIDTH; angle++)
			if (msg->ranges.at(angle) < closest && msg->ranges.at(angle) > 0)
				closest = msg->ranges.at(angle);
		scan_data_[i] = closest;
	}
}

/********************************************************************************
** Velocity publisher
********************************************************************************/
void WallFollower::update_cmd_vel(double linear, double angular)
{
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    cmd_vel_pub_->publish(cmd);
}

/********************************************************************************
** Main PID logic
********************************************************************************/
void WallFollower::update_callback()
{
	/*Default:
		if (near_start) {update_cmd_vel(0.0, 0.0); exit(0);}
		else if (scan_data_[LEFT_FRONT] > 0.9) update_cmd_vel(0.2, 1.5);
		else if (scan_data_[FRONT] < 0.7) update_cmd_vel(0.0, -1.5);
		else if (scan_data_[FRONT_LEFT] < 0.6) update_cmd_vel(0.3, -1.5);
		else if (scan_data_[FRONT_RIGHT] < 0.6) update_cmd_vel(0.3, 1.5);
		else if (scan_data_[LEFT_FRONT] > 0.6) update_cmd_vel(0.3, 1.5);
		else update_cmd_vel(0.3, 0.0);
	*/

	// if (near_start) {update_cmd_vel(0.0, 0.0); exit(0);} 
	// else if (scan_data_[LEFT_FRONT] >  0.34) update_cmd_vel(2, 1.2); 1
	// else if (scan_data_[FRONT] < 0.32) update_cmd_vel(0.2, -1.5); 2
	// else if (scan_data_[FRONT_LEFT] < 0.2) update_cmd_vel(0.3, -1.5); 3
	// else if (scan_data_[FRONT_RIGHT] < 0.2) update_cmd_vel(0.3, 1.5); 4
	// else if (scan_data_[LEFT_FRONT] > 0.2) update_cmd_vel(1, 1.5); 5
	// else update_cmd_vel(0.3, 0.0); 6

	// double x = 0.4;

    // --- Read relevant beams ---
    double front       = scan_data_[FRONT];        // 0°
    double front_left  = scan_data_[FRONT_LEFT];   // 30°
    double left_front  = scan_data_[LEFT_FRONT];   // 60°
    double left        = scan_data_[LEFT];         // 90°
    double right_front = scan_data_[RIGHT_FRONT];  // 300°

	// if (near_start) {update_cmd_vel(0.0, 0.0); exit(0);}
	// else if (scan_data_[LEFT_FRONT] > 0.68) {update_cmd_vel(1.2 - x, 2.2 );fprintf(stderr, "1\n");}
	// else if (scan_data_[FRONT] < 0.48) {update_cmd_vel(0, -1.5);fprintf(stderr, "2\n");}
	// // else if (scan_data_[LEFT] < 0.4) {update_cmd_vel(1, -1.5);fprintf(stderr, "3\n");}
	// else if (scan_data_[FRONT_LEFT] < 0.46) {update_cmd_vel(1 - x, -0.5);fprintf(stderr, "4\n");}
	// else if (scan_data_[RIGHT_FRONT] < 0.4) {update_cmd_vel(1 - x, 0.5);fprintf(stderr, "5\n");}
	// else if (scan_data_[LEFT_FRONT] > 0.4) {update_cmd_vel(1.2 - x, 2.2);fprintf(stderr, "6\n");}
	// else {update_cmd_vel(0.3, 0);fprintf(stderr, "7\n");}

	bool test = false;

	if (test) {
		// This should do a full rotation in 4 seconds: If it completes it in x seconds:
		// time = x;
		// factor = alpha;
		// 2 * 3.1415 * alpha = 1 * time
		// alpha = time / (2pi) <--- get the factor relating cmdvel angular velocity to real angular velocity
		// so realw = alpha * w; Bigger alpha means it takes longer time than mathematically
		// If we want to do a quarter turn in x time:
		// realW = pi/4 / x
		update_cmd_vel(0, 3.1415/2);
		return;
	}
	

	if (near_start) {update_cmd_vel(0.0, 0.0); exit(0);} 
	else if ((scan_data_[FRONT_LEFT] < 0.18 || scan_data_[FRONT] < 0.18 || scan_data_[FRONT_RIGHT] < 0.18)) {
		// Too close to something in front so reverse
		// update_cmd_vel(-0.2, 0.05);
		fprintf(stderr, "Reverse\n");
	}
	else if (scan_data_[LEFT_FRONT] >  0.7) {
		// Left wall disappeared so do a harsher turn
		update_cmd_vel(LINEAR_VELOCITY+0.05, 0.3);
		
		fprintf(stderr, "Left wall not found\n");
	}
	else if (scan_data_[FRONT] < 0.35
		// || scan_data_[FRONT_LEFT] < 0.22 
		|| scan_data_[FRONT_RIGHT] < 0.22
	) {
		// Turn right quickly
		update_cmd_vel(-0.3, -2); 
		fprintf(stderr, "Wall in Front close\n");

		// else {
		// 	// Turn right gradually
		// 	update_cmd_vel(LINEAR_VELOCITY -0.9); 
		// 	fprintf(stderr, "Wall in Front far");
		// }
	}
	
	else if (scan_data_[FRONT_LEFT] < 0.36) {
		// Too close to wall on the left or skewed towards it 
		update_cmd_vel(LINEAR_VELOCITY, -0.8); 
		fprintf(stderr, "Skew away from left\n");

	}
	else if (scan_data_[FRONT_RIGHT] < 0.23) {
		// Too close to wall on the right or skewed towards it 
		update_cmd_vel(LINEAR_VELOCITY, 1.5); 
		fprintf(stderr, "Skew away from right\n");

	} 
	else if (scan_data_[FRONT_LEFT] > 0.7) {
		// Too far from wall on the left or skewed away it 
		update_cmd_vel(LINEAR_VELOCITY, 0.5); 
		fprintf(stderr, "Skew towards Left\n");
		
	}
	// else if (scan_data_[LEFT_FRONT] > 0.2) {
	// 	update_cmd_vel(1, 1.5); 
	// }
	else {
		update_cmd_vel(LINEAR_VELOCITY, 0.0);
		fprintf(stderr, "Move forward\n");

	}


	// fprintf(stderr, "scan FRONT: %f\n", scan_data_[FRONT]);
	// fprintf(stderr, "scan FRONT LEFT: %f\n", scan_data_[FRONT_LEFT]);
	// fprintf(stderr, "scan LEFT FRONT: %f\n", scan_data_[LEFT_FRONT]);
	// fprintf(stderr, "scan RIGHT FRONT: %f\n", scan_data_[RIGHT_FRONT]);
	
}




/*******************************************************************************
** Main
********************************************************************************/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollower>());
    rclcpp::shutdown();
    return 0;
}
