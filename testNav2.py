
#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# from geometry_msgs.msg import PoseStamped
# from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# import rclpy


"""
Basic navigation demo to follow a given path after smoothing
"""

from wall_follower.wall_follower.landmark import max_markers
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration


def main():
    rclpy.init()

    filename = "./exampleLandmarks.csv"


    with open(filename) as file:
        landmarks = [line.rstrip() for line in file]
        
    i = 0
    # rows = max_markers + 1
    rows = 6 + 1
    cols = 3
    # Initialize a 2D array with zeros
    landmarkPositions = [[0 for _ in range(cols)] for _ in range(rows)]
    for line in landmarks:
        formatLine = line.replace(",", "")
        print(formatLine)
        landmarkPositions[i] = formatLine.split(' ')
        i += 1
        
    print(landmarkPositions)

    navigator = BasicNavigator()

    # get each pose
    poseList = []
    for i in landmarkPositions.length + 1:
        newPose = PoseStamped()
        newPose.header.frame_id = 'map'
        newPose.header.stamp = navigator.get_clock().now().to_msg()
        if (i == landmarkPositions.length):
            newPose.pose.position.x = landmarkPositions[0][0]
            newPose.pose.position.y = landmarkPositions[0][1]
        else:
            newPose.pose.position.x = landmarkPositions[i][0]
            newPose.pose.position.y = landmarkPositions[i][1]
        newPose.pose.orientation.w = 1.0
        poseList[i] = newPose
    
    # get each path
    pathList = []
    for i in poseList.length - 1:
        pathStart = poseList[i]
        pathEnd = poseList[i + 1]

        pathList[i] = navigator.smoothPath(navigator.getPath(pathStart, pathEnd))
    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    while rclpy.ok():
        # Follow path
        
        for path in pathList:
            navigator.followPath(path)

            i = 0
            while not navigator.isTaskComplete():
                

                # Do something with the feedback
                i += 1
                feedback = navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print('Estimated distance remaining to goal position: ' +
                        '{0:.3f}'.format(feedback.distance_to_goal) +
                        '\nCurrent speed of the robot: ' +
                        '{0:.3f}'.format(feedback.speed))

            # Do something depending on the return code
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')

        # end of path list
        navigator.lifecycleShutdown()
        rclpy.shutdown()
        
    exit(0)


if __name__ == '__main__':
    main()