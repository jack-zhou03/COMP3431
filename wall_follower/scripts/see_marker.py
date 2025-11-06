#!/usr/bin/env python3

# Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
# Author: Claude Sammut
# Last Modified: 2025.09.28

# ROS 2 program to subscribe to real-time streaming 
# video from TurtleBot3 Pi camera and find coloured landmarks.

# Colour segementation and connected components example added by Claude sSammut
	
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import math
import numpy as np
from sensor_msgs.msg import Image # Image is the message type
from sensor_msgs.msg import CompressedImage # CompressedImage message type
from sensor_msgs.msg import LaserScan # Laser scan message type
from sensor_msgs.msg import CameraInfo # Need to know camera frame
from geometry_msgs.msg import Point, PointStamped

import wall_follower.landmark
from wall_follower.landmark import marker_type


field_of_view_h = 62.2
field_of_view_v = 48.8
focal_length = 3.04
pixel_size = 105     # need to test the real pixel size in mm
# pixel_size = 0.19 
# pixel_size = 892.86
# 121.7105
real_object_size = 100.0 
distance_numerator = real_object_size * focal_length * pixel_size

CALIBRATION_MODE = False
CALIBRATION_DISTANCE = 850.0 # in mm, means 1 meter (1000)
class SeeMarker(Node):
	"""
	Create an ImageSubscriber class, which is a subclass of the Node class.
	"""
	def __init__(self):
		"""
		Class constructor to set up the node
		"""
		# Initiate the Node class's constructor and give it a name
		super().__init__('see_marker')

		self.range = 0
		self.prev_h = 0
		self.prev_r = 0
			
		# Create the subscriber. This subscriber will receive an Image
		# from the video_frames topic. The queue size is 10 messages.
		self.subscription = self.create_subscription(
			CompressedImage,
			'/camera/image_raw/compressed', 	# Change to "compressed" for real robot
			self.listener_callback, 
			10)
		self.subscription # prevent unused variable warning
			
		# Used to convert between ROS and OpenCV images
		self.br = CvBridge()

		self.point_publisher = self.create_publisher(PointStamped, '/marker_position', 10)

	def listener_callback(self, data):
		"""
		Callback function with debugging.
		"""
		# self.get_logger().info(f'Received data: format={data.format}, size={len(data.data)} bytes')
		
		if len(data.data) == 0:
			# self.get_logger().error('Data is empty!')
			return
		
		# try:
		# 	current_frame = self.br.compressed_imgmsg_to_cv2(data)
		# 	self.get_logger().info(f'Decoded successfully: shape={current_frame.shape}')
		# except Exception as e:
		# 	self.get_logger().error(f'Decode failed: {e}')
		# 	return
		"""
		Callback function.
		"""
		# Display the message on the console
		# self.get_logger().info('Receiving video frame')

		# Convert ROS Image message to OpenCV image

		# uncompressed the data ////////////////////////////////////////////////
		# self.get_logger().info('Receiving compressed video frame')
		current_frame = self.br.compressed_imgmsg_to_cv2(data)
		height, width = current_frame.shape[:2]
		self.get_logger().info(f'Camera resolution: {width}x{height}')
		
		# The following code is a simple example of colour segmentation
		# and connected components analysis
		
		# Convert BGR image to HSV
		hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
		# cv2.imshow("camera",current_frame)
		
		# cv2.waitKey(1)
		# Find pink blob
		combined_result = np.zeros_like(current_frame)
		pink_result, pink_blob = segment(current_frame, hsv_frame, "pink", self)
		if pink_blob:
			(pink_x, pink_y, pink_h, p_d, p_a) = pink_blob
			combined_result = cv2.bitwise_or(combined_result, pink_result)
			for c in ["blue", "green", "yellow"]:
				color_result, blob = segment(current_frame, hsv_frame, c, self)
				if blob:
					(c_x, c_y, c_h, c_d, c_a) = blob
					combined_result = cv2.bitwise_or(combined_result, color_result)
					# Check to see if the blobs are verically aligned
					if abs(pink_x - c_x) > pink_h:
						print(f'pink_x = {pink_x}, pink_y = {pink_y}, h = {pink_h}')
						continue

					marker_at = PointStamped()
					marker_at.header.stamp = self.get_clock().now().to_msg()
					marker_at.header.frame_id = 'camera_link'

					"""
					The next few lines are a hack. Since the markers are always on the ground,
					we don't use the "z" coordinate for position, so instead we use it to
					store the parker type. Ugly, but it saves creating a new message type.
					"""
					if c_y < pink_y:	# +y is down
						print(c, "/ pink", f'{c_d:.2f}, {c_a:.2f}')
						marker_at.point.z = float(marker_type.index(c + '/pink'))
					else:
						print("pink / ", c, f'{p_d:.2f}, {p_a:.2f}')
						marker_at.point.z = float(marker_type.index('pink/' + c))
					
					x, y = polar_to_cartesian(c_d, c_a)
					self.get_logger().info(f'Polar to Cartesian: x={x:.2f} , y={y:.2f}')
					real_x_y = math.sqrt(x**2 + y**2)
					self.get_logger().info(f'real_x_y: {real_x_y:.2f} ')

					# x = x / 0.00174
					# y = y / 0.00174
					marker_at.point.x = x
					marker_at.point.y = y

					distance = math.sqrt(x**2 + y**2)
					self.get_logger().info(f'distance to marker: {distance:.2f} mm')

					print(f'Camera coordinates: {x}, {y}')
					self.point_publisher.publish(marker_at)
					self.get_logger().info('Published Point: x=%f, y=%f, z=%f' %
						(marker_at.point.x, marker_at.point.y, marker_at.point.z))

		# Display camera image
		cv2.imshow("camera", current_frame)
		cv2.imshow("result", combined_result)
		cv2.setMouseCallback("camera", mouse_callback, hsv_frame)
		cv2.waitKey(1)
		
# pink
# Clicked pixel at (125,93) HSV = (164,151,230)
# Clicked pixel at (125,100) HSV = (164,154,229)
# Clicked pixel at (128,107) HSV = (164,157,231)
# Clicked pixel at (141,107) HSV = (165,158,224)
# Clicked pixel at (150,96) HSV = (167,152,207)
# Clicked pixel at (122,92) HSV = (164,150,231)
# Clicked pixel at (86,107) HSV = (164,149,230)

# blue
# Clicked pixel at (121,43) HSV = (99,237,207)
# Clicked pixel at (115,43) HSV = (99,235,208)
# Clicked pixel at (127,38) HSV = (99,232,202)
# Clicked pixel at (129,28) HSV = (100,232,198)
# Clicked pixel at (124,61) HSV = (99,231,202)
# Clicked pixel at (65,65) HSV = (100,211,207)

# yellow
# Clicked pixel at (38,37) HSV = (28,255,244)
# Clicked pixel at (68,19) HSV = (27,254,204)
# Clicked pixel at (90,22) HSV = (25,255,183)
# Clicked pixel at (63,46) HSV = (28,255,223)
# Clicked pixel at (84,52) HSV = (27,255,203)
# Clicked pixel at (92,19) HSV = (25,249,177)

# green
# Clicked pixel at (79,75) HSV = (83,244,137)
# Clicked pixel at (102,74) HSV = (81,206,89)
# Clicked pixel at (98,94) HSV = (80,205,103)
# Clicked pixel at (76,97) HSV = (81,231,146)
# colours = {
# 	"pink":	 	((150,110,130), (170, 170, 255)),
# 	"blue":		((85,140,100), (115, 250, 255)),
# 	"green":	((75,190,80), (85, 245, 150)),
# 	"yellow":	((20,210,170), (32, 255, 245))
# }

colours = {
	"pink":	 	((140,70,100), (170, 255, 255)),
	"blue":		((100,100,100), (130, 255, 255)),
	"green":	((50,70,70), (99, 255, 255)),
	"yellow":	((10,100,150), (40, 255, 255))
}

# colours = {
# 	"pink":	 	((140,0,0), (170, 255, 255)),
# 	"blue":		((100,0,0), (130, 255, 255)),
# 	"green":	((40,0,0), (80, 255, 255)),
# 	"yellow":	((25,0,0), (32, 255, 255))
# }

# colours = {
# 	"pink":   ((140, 60, 80), (170, 170, 255)),  # more strict
# 	"blue":		((100,0,0), (130, 255, 255)),
#  	"green":	((40,0,0), (80, 255, 255)),
# 	"yellow": ((20, 90, 90), (32, 255, 245))
# }

def mouse_callback(event,x,y,flags,param):
	if event == cv2.EVENT_LBUTTONDOWN:  # click the left mouse button
			hsv_frame = param
			h, s, v = hsv_frame[y, x]
			print(f"Clicked pixel at ({x},{y}) HSV = ({h},{s},{v})")

def segment(current_frame, hsv_frame, colour, node):
	"""
	Mask out everything except the specified colour
	Connect pixels to form a blob
	"""

	(lower, upper) = colours[colour]

	# Mask out everything except pixels of colour parameter
	mask = cv2.inRange(hsv_frame, lower, upper)
	result = cv2.bitwise_and(current_frame, current_frame, mask=mask)

	# Run 4-way connected components, with statistics
	blobs = cv2.connectedComponentsWithStats(mask, 4, cv2.CV_32S)

	# Display masked image
	# cv2.imshow("result", result)

	# Print statistics for each blob (connected component)
	return result, get_stats(blobs, colour, node)


def get_stats(blobs, colour, node):
	"""
	Print statistics for each blob (connected component)
	of the specified colour
	Return the centroid and height of the largest blob, if there is one.
	"""

	(numLabels, labels, stats, centroids) = blobs
	if numLabels == 0:
		return None

	largest = 0
	rval = None
	# centre = 320 # 640/2
	centre = 80 # 160/2
	MIN_HEIGHT = 10 *0.25
	MIN_AREA = 50 *0.25
	for i in range(1, numLabels):
		x = stats[i, cv2.CC_STAT_LEFT]
		y = stats[i, cv2.CC_STAT_TOP]
		w = stats[i, cv2.CC_STAT_WIDTH]
		h = stats[i, cv2.CC_STAT_HEIGHT]
		area = stats[i, cv2.CC_STAT_AREA]
		(cx, cy) = centroids[i]
		# print(colour, x, y, w, h, area, cx, cy)
		# node.get_logger().info(f'height of pixels = {h}')
		if h < MIN_HEIGHT:
			# node.get_logger().info(f'{colour}: h={h} too small, skipping')
			continue
    
		if area < MIN_AREA:
			# node.get_logger().info(f'{colour}: area={area} too small, skipping')
			continue
		if area > largest:
			largest = area
			# distance = 35.772 * pow(h, -0.859) # obtained experimentally
			if CALIBRATION_MODE:   
				calculated_pixel_size = (CALIBRATION_DISTANCE * h) / (real_object_size * focal_length)
				node.get_logger().info(f'Calibrated pixel size = {calculated_pixel_size:.4f} pixels/mm')
			distance = distance_numerator / h # https://www.baeldung.com/cs/cv-compute-distance-from-object-video
			"""
			The condition below accounts for when the marker is at the edge of the screen 
			and is only paritally visible. The assumption is that if the aspect ratio < 0.8
			then it is only partially visisble.
			We need two cases for when the marker is on the left or right edge.
			"""
			# aspect_ratio = h/w
			aspect_ratio = w/h
			node.get_logger().info(f'{colour} LARGEST: h={h}, distance={distance:.2f}, aspect_ratio={aspect_ratio:.2f}')
			# I think we need do some change for the aspect ratio as well
			if aspect_ratio < 0.8: 
				if cx < centre:
					cx += h-w
				else:
					cx -= h-w
			# angle = (centre - cx) * field_of_view_h / 640
			angle = (centre - cx) * field_of_view_h / 160
			if angle < 0:
				angle += 360
			rval = (cx, cy, h, distance, angle)

	if (CALIBRATION_MODE):
		print("\nHeight h: {h}, Measure the distance: ")
		input("Press key to continue: ")

	return rval


def polar_to_cartesian(distance, angle):
	# Convert angle from degrees to radians
	angle_rad = math.radians(angle)

	# Calculate x and y coordinates
	x = distance * math.cos(angle_rad)
	y = distance * math.sin(angle_rad)

	return x, y


def main(args=None):
	
	# Initialize the rclpy library
	rclpy.init(args=args)
	
	# Create the node
	see_marker = SeeMarker()
	
	# Spin the node so the callback function is called.
	try:
		rclpy.spin(see_marker)
	except KeyboardInterrupt:
		exit()

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	see_marker.destroy_node()
	
	# Shutdown the ROS client library for Python
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()