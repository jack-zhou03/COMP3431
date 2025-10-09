// Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
// Author: Claude Sammut
// Last Modified: 2024.10.14

// Use this code as the basis for a wall follower

#include "wall_follower/wall_follower.hpp"

#include <memory>


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
		near_start = true;
		first = true;
		start_moving = true;
	}
}

// Half of a scan section
// Compass directions requires 22.5 but round up to 23 for integers
#define BEAM_WIDTH 23

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	const sections = 360 / (2 * BEAM_WIDTH);
	uint16_t scan_angle[sections];
	for (int i = 0; i < sections; ++i) {
		scan_angle[i] = i * (BEAM_WIDTH * 2);
	}

	double closest = msg->range_max;
	for (int angle = 360-BEAM_WIDTH; angle < 360; angle++)
		if (msg->ranges.at(angle) < closest)
			closest = msg->ranges.at(angle);
	for (int angle = 0; angle < BEAM_WIDTH; angle++)
		if (msg->ranges.at(angle) < closest)
			closest = msg->ranges.at(angle);
	scan_data_[0] = closest;

	for (int i = 1; i < 12; i++)
	{
		closest = msg->range_max;
		for (int angle = scan_angle[i]-BEAM_WIDTH; angle < scan_angle[i]+BEAM_WIDTH; angle++)
			if (msg->ranges.at(angle) < closest)
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


void WallFollower::update_callback()
{
	// Scan data is in metres
	// Distance from centre to corner of robot is about 0.17 m
	const double robotLength = 0.17;
	// Maze is separated into 'boxes.' Side length of a box is about 0.5 m
	const double boxSize = 0.5;
	// Keep robot in middle of box: Currently evaluates to an 8 cm gap
	const double nominalDistance = (boxSize / 2); 
	// Percentage tolerance used for too close distance to wall
	const double distanceTolerance = 0.9;
	// Distance away from the wall before an adjustment is needed (about 5cm)
	const double tooClose = (nominalDistance) * distanceTolerance;
	const double tooFar = (nominalDistance) / distanceTolerance;
	// Percentage tolerance for making sure the robot is straight with the wall
	const double straightTolerance = 0.9;
	// Wall in front distance (about 3 cm)
	const double wallInFront = tooClose * distanceTolerance;


	

	// Try the scanning in compass directions
	// +AngularVelocity is CCW (Left), -AngularVelocity is CW (Right)
	// If going back to the start then the run should finish
	if (near_start) {update_cmd_vel(0.0, 0.0); exit(0);}
	// If the robot is too far away from the wall on the left then turn back towards it
	else if (scan_data[LEFT] > tooFar ) update_cmd_vel(0.2, ANGULAR_VELOCITY);
	// If the robot is too close to the wall on the left or right veer the other direction
	else if (scan_data[FRONT_LEFT] < tooClose) update_cmd_vel(0.2, -ANGULAR_VELOCITY);
	else if (scan_data[FRONT_RIGHT] < tooClose) update_cmd_vel(0.2, ANGULAR_VELOCITY);
	// If the robot is too close to the wall in front then turn right
	else if (scan_data[FRONT] < wallInFront) update_cmd_vel(0.0, -ANGULAR_VELOCITY);
	// Straighten robot within a tolerance: Front left is closer than back left means it's pointing towards the wall and vice versa
	else if (scan_data[FRONT_LEFT] * straightTolerance < scan_data[BACK_LEFT]) update_cmd_vel(0.2, -ANGULAR_VELOCITY);
	else if (scan_data[BACK_LEFT] * straightTolerance > scan_data[FRONT_LEFT]) update_cmd_vel(0.2, ANGULAR_VELOCITY);
	


	// else if (scan_data_[LEFT_FRONT] > 0.9) update_cmd_vel(0.2, ANGULAR_VELOCITY);
	// else if (scan_data_[FRONT] < 0.7) update_cmd_vel(0.0, -ANGULAR_VELOCITY);
	// else if (scan_data_[FRONT_LEFT] < 0.6) update_cmd_vel(LINEAR_VELOCITY, -ANGULAR_VELOCITY);
	// else if (scan_data_[FRONT_RIGHT] < 0.6) update_cmd_vel(LINEAR_VELOCITY, ANGULAR_VELOCITY);
	// else if (scan_data_[LEFT_FRONT] > 0.6) update_cmd_vel(LINEAR_VELOCITY, ANGULAR_VELOCITY);
	else update_cmd_vel(LINEAR_VELOCITY, 0.0);
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
