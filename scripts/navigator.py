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
        # [-0.06242923379727182,-3.10064952800541],
        # [-0.08372162803232902,-1.0074568385397238],
        # [2.9708988171937776,0.13127214449794022],
        [0.7699593453880917,-2.9495048140375753],
        [0.0,-0.1]
    ]

    # Set our demo's initial pose
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 0.0
    # initial_pose.pose.position.y = 0.0
    # initial_pose.pose.orientation.z = 0.0
    # initial_pose.pose.orientation.w = 1.0
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

        # # go back to the start 

        # start_pose = PoseStamped()
        # start_pose.header.frame_id = 'map'
        # start_pose.header.stamp = navigator.get_clock().now().to_msg()
        # start_pose.pose.position.x = 0.0
        # start_pose.pose.position.y = 0.0
        # start_pose.pose.orientation.w = 1.0
        # navigator.goToPose(start_pose)
        # while not navigator.isTaskComplete():
        #     i+=1
        #     feedback = navigator.getFeedback()
        #     if feedback and i % 5 == 0:
        #         print(
        #             'Estimated time to return to start: '
        #             + '{:.0f}'.format(
        #                 Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
        #                 / 1e9
        #             )
        #             + ' seconds.'
        #         )
        
        # result = navigator.getResult()
        # if result == TaskResult.SUCCEEDED:
        #     print('Returned to start position, restarting route...')
        # elif result == TaskResult.CANCELED:
        #     print('Return to start was canceled, exiting.')
        #     exit(1)
        # elif result == TaskResult.FAILED:
        #     (error_code, error_msg) = navigator.getTaskError()
        #     print(f'Return to start failed!:{error_code}:{error_msg}')
        #     print('Exiting.')

    exit(0)


if __name__ == '__main__':
    main()

# from geometry_msgs.msg import PoseStamped
# from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# import rclpy
# from rclpy.duration import Duration

# """
# Basic navigation demo to go to pose.
# """


# def main():
#     rclpy.init()

#     coordinates_order = [
#         # [2.95, 0.0835],
#         # [-0.01724, -1.0147],
#         # [-0.01797, -3.1092],
#         # [0.8829, -3.0146],
#         # [0.0,0.0]
#         [-0.06242923379727182,-3.10064952800541],
#         [-0.08372162803232902,-1.0074568385397238],
#         [2.9708988171937776,0.13127214449794022],
#         [0.7699593453880917,-2.9495048140375753],
#         [0.0,0.0]
#     ]
#     navigator = BasicNavigator()


#     # Activate navigation, if not autostarted. This should be called after setInitialPose()
#     # or this will initialize at the origin of the map and update the costmap with bogus readings.
#     # If autostart, you should `waitUntilNav2Active()` instead.
#     # navigator.lifecycleStartup()

#     # Wait for navigation to fully activate, since autostarting nav2
#     navigator.waitUntilNav2Active()

#     # If desired, you can change or load the map as well
#     # navigator.changeMap('/path/to/map.yaml')

#     # You may use the navigator to clear or obtain costmaps
#     # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
#     # global_costmap = navigator.getGlobalCostmap()
#     # local_costmap = navigator.getLocalCostmap()


#     while rclpy.ok():
#         # Go to our demos first goal pose
#         for cord in coordinates_order:
#             goal_pose = PoseStamped()
#             goal_pose.header.frame_id = 'map'
#             goal_pose.header.stamp = navigator.get_clock().now().to_msg()
#             goal_pose.pose.position.x = cord[0]
#             goal_pose.pose.position.y = cord[1]
#             goal_pose.pose.orientation.w = 1.0

#             # sanity check a valid path exists
#             # path = navigator.getPath(initial_pose, goal_pose)

#             navigator.goToPose(goal_pose)

#             i = 0
#             while not navigator.isTaskComplete():
#                 ################################################
#                 #
#                 # Implement some code here for your application!
#                 #
#                 ################################################

#                 # Do something with the feedback
#                 i = i + 1
#                 feedback = navigator.getFeedback()
#                 if feedback and i % 5 == 0:
#                     print('Estimated time of arrival: ' + '{0:.0f}'.format(
#                         Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
#                         + ' seconds.')

                

#             # Do something depending on the return code
#             result = navigator.getResult()
#             if result == TaskResult.SUCCEEDED:
#                 print('Goal succeeded!')
#             elif result == TaskResult.CANCELED:
#                 print('Goal was canceled!')
#             elif result == TaskResult.FAILED:
#                 print('Goal failed!')
#             else:
#                 print('Goal has an invalid return status!')

#         exit(0)


# if __name__ == '__main__':
#     main()



