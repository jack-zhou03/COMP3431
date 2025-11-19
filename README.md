# COMP3431
Wall follower and vision code

# Starting the robot 

1. connect to the wifi 
2. ping the robot to check for connection
3. tmux 
    - ssh 
    - bring up 
4. separate terminal check ros2 topic list 
5. separate terminal start camera and rqt 
6. wall follower code

# NAV2
Saving map command:
ros2 run nav2_map_server map_saver_cli -f test

To get the coordinates on rviz, open rviz and put waypoint markers 
ros2 topic echo waypoint_ ndoes or list
If you scroll, you will see it has an attribute for marker type thats numbered for according to the marker added on rviz
note that not every marker is labeled so make sure to check before

run this with config inside wall follower folder 
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=/home/pi/turtlebot3_ws/test.yaml params_file:=/home/pi/turtlebot3_ws/src/wall_follower/config/waffle_pi.yaml

