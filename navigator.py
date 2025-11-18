#!/usr/bin/env python3


from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

"""
Basic security route patrol demo. In this demonstration, the expectation
is that there are security cameras mounted on the robots recording or being
watched live by security staff.
"""


def main() -> None:
    rclpy.init()

    navigator = BasicNavigator()

    # Security route, probably read in from a file for a real application
    # from either a map or drive and repeat.
    security_route = [
        [2.9515, 0.08425],
        [-0.01724, -1.0147],
        [-0.01797, -3.1092],
        [0.8829, -3.0146]
    ]

    # Set our demo's initial pose
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 0.02986
    # initial_pose.pose.position.y = -0.06687
    # initial_pose.pose.orientation.z = -0.001723
    # initial_pose.pose.orientation.w =  0.9999
    # navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    # Do security route until dead
    while rclpy.ok():
        # Send our route
        route_poses = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0
        for pt in security_route:
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            route_poses.append(deepcopy(pose))
        go_through_poses_task = navigator.goThroughPoses(route_poses)

        # Do something during our route (e.x. AI detection on camera images for anomalies)
        # Simply print ETA for the demonstration
        i = 0
        while not navigator.isTaskComplete():
            i += 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'Estimated time to complete current route: '
                    + '{:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )

                # Some failure mode, must stop since the robot is clearly stuck
                if Duration.from_msg(feedback.navigation_time) > Duration(
                    seconds=180.0
                ):
                    print('Navigation has exceeded timeout of 180s, canceling request.')
                    navigator.cancelTask()

        # If at end of route, reverse the route to restart
        security_route.reverse()

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Route complete! Restarting...')
        elif result == TaskResult.CANCELED:
            print('Security route was canceled, exiting.')
            exit(1)
        elif result == TaskResult.FAILED:
            (error_code, error_msg) = navigator.getTaskError()
            print(f'Security route failed!:{error_code}:{error_msg}')
            print('Restarting from other side...')

    exit(0)


if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# import math, csv, os, time

# class MarkerNavigator(Node):
#     def __init__(self):
#         super().__init__('marker_navigator')
#         self.navigator = BasicNavigator()

#         # --- Load CSV of markers ---
#         csv_path = '/home/pi/turtlebot3_ws/landmarks_new.csv'
#         if not os.path.exists(csv_path):
#             self.get_logger().error(f"❌ CSV file not found: {csv_path}")
#             return

#         self.get_logger().info("Waiting for Nav2 to become active...")
#         self.navigator.waitUntilNav2Active()
#         self.get_logger().info("✅ Nav2 is active. Starting waypoint navigation...")

#         waypoints = self.load_markers(csv_path)
#         self.navigate_waypoints(waypoints)
#         self.return_to_start(waypoints[0])

#     # --------------------------------------------------------------------
#     def load_markers(self, csv_path):
#         waypoints = []
#         with open(csv_path, 'r') as f:
#             reader = csv.reader(f)
#             for row in reader:
#                 try:
#                     x, y = float(row[0]), float(row[1])
#                     theta = float(row[2]) if len(row) > 2 else 0.0
#                     waypoints.append((x, y, theta))
#                 except ValueError:
#                     self.get_logger().warn(f"Skipping invalid row: {row}")
#         self.get_logger().info(f"📍 Loaded {len(waypoints)} markers.")
#         return waypoints

#     # --------------------------------------------------------------------
#     def navigate_waypoints(self, waypoints):
#         for i, (x, y, theta) in enumerate(waypoints, start=1):
#             pose = self.create_pose(x, y, theta)
#             self.get_logger().info(f"➡️ Navigating to marker {i}/{len(waypoints)}: ({x:.2f}, {y:.2f})")

#             self.navigator.goToPose(pose)
#             self.monitor_progress(i)

#     # --------------------------------------------------------------------
#     def monitor_progress(self, idx):
#         """Poll feedback and check for completion."""
#         start_time = time.time()
#         TIMEOUT = 180  # seconds per waypoint
#         while not self.navigator.isTaskComplete():
#             feedback = self.navigator.getFeedback()
#             if feedback:
#                 eta = feedback.estimated_time_remaining.sec
#                 self.get_logger().info(f"⏱ ETA to waypoint {idx}: {eta} sec")
#             if time.time() - start_time > TIMEOUT:
#                 self.get_logger().warn(f"⚠️ Timeout at waypoint {idx}. Cancelling goal.")
#                 self.navigator.cancelTask()
#                 break
#             time.sleep(0.5)

#         result = self.navigator.getResult()
#         if result == TaskResult.SUCCEEDED:
#             self.get_logger().info(f"✅ Reached marker {idx}")
#         elif result == TaskResult.CANCELED:
#             self.get_logger().warn(f"⚠️ Navigation to marker {idx} canceled.")
#         elif result == TaskResult.FAILED:
#             self.get_logger().error(f"❌ Navigation to marker {idx} failed.")

#     # --------------------------------------------------------------------
#     def return_to_start(self, start):
#         x, y, theta = start
#         pose = self.create_pose(x, y, theta)
#         self.get_logger().info("🏁 Returning to start position...")
#         self.navigator.goToPose(pose)
#         self.monitor_progress("start")
#         self.get_logger().info("✅ Navigation complete!")

#     # --------------------------------------------------------------------
#     def create_pose(self, x, y, theta):
#         pose = PoseStamped()
#         pose.header.frame_id = 'map'
#         pose.header.stamp = self.navigator.get_clock().now().to_msg()

#         pose.pose.position.x = x
#         pose.pose.position.y = y

#         # Full quaternion (yaw-only rotation)
#         pose.pose.orientation.x = 0.0
#         pose.pose.orientation.y = 0.0
#         pose.pose.orientation.z = math.sin(theta / 2.0)
#         pose.pose.orientation.w = math.cos(theta / 2.0)
#         return pose

# # ------------------------------------------------------------------------

# def main(args=None):
#     rclpy.init(args=args)
#     node = MarkerNavigator()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
