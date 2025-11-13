#!/usr/bin/env python3

# Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
# Author: Claude Sammut
# Last Modified: 2024.10.14

# ROS 2 program to subscribe to update a MarkerArray subscribing to 
# PointStamed topic, assuming it's published by a vision node.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import csv


import wall_follower.landmark
from wall_follower.landmark import marker_type, max_markers, Landmark

from nav_msgs.msg import Odometry
import math

class PointTransformer(Node):

	def __init__(self):
		super().__init__('point_transformer')
		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
		self.point_subscriber = self.create_subscription(PointStamped, '/marker_position', self.point_callback, 10)
		self.marker_publisher_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

		self.marker_position = []
		self.marker_array = MarkerArray()			
		self.marker_array.markers = []
		self.start_recorded = False
		self.start_x = 0.0
		self.start_y = 0.0
		self.start_heading = 0.0

		self.odom_sub = self.create_subscription(
			Odometry,
			'/odom',
			self.odom_callback,
			10)
		for i in range(max_markers):
			self.marker_position.append(Landmark(i, self.marker_array.markers))
		


	# def saveLandmarks(self):
	# 	with open('landmarks.csv', 'w', newline='') as csvfile:
	# 		lm_writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
	# 		for lm in self.marker_position:
	# 			if lm.top_marker is not None:
	# 				# lm_writer.writerow(lm.toRow())
	# 				lm_writer.writerow([lm.top_marker.pose.position.x,
    #                 lm.top_marker.pose.position.y,
    #                 marker_type[lm.mtype]])
	def saveLandmarks(self):
		with open('landmarks.csv', 'w', newline='') as csvfile:
			lm_writer = csv.writer(csvfile, delimiter=', ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
			lm_writer.writerow([self.start_x, self.start_y, self.start_heading])
			for lm in self.marker_position:
				if lm.top_marker is not None:
					lm_writer.writerow([
						lm.top_marker.pose.position.x,
						lm.top_marker.pose.position.y,
						marker_type[lm.mtype]   
					])




	def point_callback(self, msg):
		try:
			# Lookup the transform from the camera_rgb_optical_frame to the map frame
			transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
		except tf2_ros.LookupException as e:
			self.get_logger().error('Transform lookup failed: %s' % str(e))
			return

		which_marker = int(msg.point.z)
		m = marker_type[which_marker]
		msg.point.z = 0.0

		# Transform the point from camera_rgb_optical_frame to map frame
		map_point = tf2_geometry_msgs.do_transform_point(msg, transform)

		# Print the transformed point in the map frame
#		self.get_logger().info(f'Mapped {m} marker to /map frame: x={map_point.point.x}, y={map_point.point.y}, z={map_point.point.z}')

		self.marker_position[which_marker].update_position(map_point.point)
		self.marker_publisher_.publish(self.marker_array)
	def odom_callback(self, msg: Odometry):
		if not self.start_recorded:
			self.start_x = msg.pose.pose.position.x
			self.start_y = msg.pose.pose.position.y
			q = msg.pose.pose.orientation
			siny_cosp = 2 * (q.w * q.z + q.x * q.y)
			cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
			self.start_heading = math.atan2(siny_cosp, cosy_cosp)

			self.start_recorded = True
			self.get_logger().info(
				f"Recorded start pose: x={self.start_x:.2f}, y={self.start_y:.2f}, heading={self.start_heading:.2f}"
			)


def main(args=None):
	rclpy.init(args=args)
	node = PointTransformer()

	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		node.saveLandmarks()
		exit()

	rclpy.shutdown()

if __name__ == '__main__':
	main()

