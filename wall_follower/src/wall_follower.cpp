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
    for (int i = 0; i < 12; i++) scan_data_[i] = 0.0;
    robot_pose_ = 0.0;
    near_start = false;

    // --- PID and target distance ---
    desired_left_dist_ = 0.55;
    kp_ = 2.4;
    ki_ = 0.0;
    kd_ = 1.0;
    integral_ = 0.0;
    prev_error_ = 0.0;
    prev_time_ = std::chrono::steady_clock::now();

    /************************************************************
    ** ROS setup
    ************************************************************/
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(),
        std::bind(&WallFollower::scan_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", qos, std::bind(&WallFollower::odom_callback, this, std::placeholders::_1));
    update_timer_ = this->create_wall_timer(10ms, std::bind(&WallFollower::update_callback, this));

    RCLCPP_INFO(this->get_logger(), "Wall follower node (multi-beam PID) initialized");
}

WallFollower::~WallFollower()
{
    RCLCPP_INFO(this->get_logger(), "Wall follower node terminated");
}

/********************************************************************************
** Odometry
********************************************************************************/
#define START_RANGE 0.2

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
    double current_y = msg->pose.pose.position.y;

    if (first) {
        start_x = current_x;
        start_y = current_y;
        first = false;
    } else if (start_moving) {
        if (fabs(current_x - start_x) > START_RANGE || fabs(current_y - start_y) > START_RANGE)
            start_moving = false;
    } else if (fabs(current_x - start_x) < START_RANGE && fabs(current_y - start_y) < START_RANGE) {
        near_start = true;
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
    for (int i = 0; i < 12; i++) {
        double closest = msg->range_max;
        int low = std::max(0, (int)scan_angle[i] - BEAM_WIDTH);
        int high = std::min((int)msg->ranges.size() - 1, (int)scan_angle[i] + BEAM_WIDTH);
        for (int angle = low; angle <= high; angle++) {
            double r = msg->ranges.at(angle);
            if (r > 0.0 && r < closest) closest = r;
        }
        scan_data_[i] = (closest == msg->range_max ? msg->range_max : closest);
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
    if (near_start) {
        update_cmd_vel(0.0, 0.0);
        return;
    }

    using clock = std::chrono::steady_clock;
    auto now = clock::now();
    double dt = std::chrono::duration<double>(now - prev_time_).count();
    if (dt <= 0.0) dt = 1e-3;
    prev_time_ = now;

    // --- Read relevant beams ---
    double front       = scan_data_[FRONT];        // 0°
    double front_left  = scan_data_[FRONT_LEFT];   // 30°
    double left_front  = scan_data_[LEFT_FRONT];   // 60°
    double left        = scan_data_[LEFT];         // 90°
    double right_front = scan_data_[RIGHT_FRONT];  // 300°

    // --- Filter left distance (low-pass) ---
    static double left_filt = 0.0;
    const double ALPHA = 0.7;
    if (left_filt == 0.0) left_filt = left;
    left_filt = ALPHA * left_filt + (1 - ALPHA) * left;
    left = left_filt;

    // --- PID core ---
    double error = (left - desired_left_dist_);
    integral_ += error * dt;
    integral_ = std::clamp(integral_, -1.0, 1.0);
    double derivative = (error - prev_error_) / dt;
    prev_error_ = error;

    // --- adaptive scaling ---
    double abs_err = std::fabs(error);
    double ang_scale = std::clamp(abs_err / 0.4, 0.5, 1.0);
    double lin_scale = std::clamp(1.0 + abs_err, 0.4, 1.0);

    double ang = (kp_ * error + ki_ * integral_ + kd_ * derivative) * ang_scale;
    double lin = 0.35 * lin_scale;

    // --- strong correction if too far ---
    if (left > 0.8) {
        lin = 0.35;
        ang = 0.6;  // steer left gently
    }

    // --- safety if too close ---
    if (left < 0.25) {
        lin = 0.0;
        ang = -1.0;  // sharp right turn
    }

    // --- obstacle avoidance (front + right-front) ---
    if (front < 0.55) {
        lin = 0.0;
        // pick safer direction: if right side has space, turn right;
        // otherwise left
        ang = (right_front > left_front) ? -1.2 : 1.2;
    } else if (front < 0.9) {
        lin *= 0.5;  // slow down near front obstacle
    }

    // --- corner approach (left front + front) ---
    bool approaching_corner = (front < 0.9 && left_front < 0.45);
    bool in_corner = (front < 0.55);
    if (in_corner) {
        lin = 0.0;
        ang = -1.2;  // commit to right turn
    }
    else if (approaching_corner) {
        lin = 0.25;
        ang = std::min(ang, -0.8);
    }

    // --- right-front avoidance ---
    if (right_front < 0.45) {
        // push away from right obstacle
        lin *= 0.8;
        ang += 0.8 * (0.45 - right_front);
    }

    // --- slow-turn damping ---
    if (std::fabs(ang) > 0.8)
        lin *= 0.5;

    // --- clamp & publish ---
    ang = std::clamp(ang, -1.5, 1.5);
    lin = std::clamp(lin, 0.0, 0.6);
    update_cmd_vel(lin, ang);

    fprintf(stderr, "[PID] L=%.2f E=%.2f A=%.2f Lin=%.2f F=%.2f RF=%.2f\n",
            left, error, ang, lin, front, right_front);
}







/********************************************************************************
** Main
********************************************************************************/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollower>());
    rclcpp::shutdown();
    return 0;
}
