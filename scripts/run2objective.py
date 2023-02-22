#!/usr/bin/env python3


import rospy
from std_msgs.msg import Int16,Float32,Float64,Int32,Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from time import time 

current_position = [0,0]

goal = [0,0]
global last_goal
last_goal = [0,0]

goal_reached = False
global last_goal_reached 
last_goal_reached = False 

start_time = 0
delta_time = [0,0]
finished_time = 0
distance2goal = [0,0]
#check current distance to goal 

#check time between start and finish 

#log info in txt 

def goal_reached_callback(msg):
    global goal_reached 
    goal_reached = msg.data

def position_callback(position_msg):
    global goal
    goal = position_msg.data

def odom_callback(odom_msg):
    global current_position

    current_position[0] = odom_msg.pose.pose.position.x
    current_position[1] = odom_msg.pose.pose.position.y

def calculate_distance(cur_pos,goal):
    delta[0] = goal[0] - cur_pos[0]
    delta[1] = goal[1] - cur_pos[1]
    return delta 


if __name__ == '__main__':
    rospy.init_node('iron_cup_node')
    rate = rospy.Rate(10)
    rospy.Subscriber("control/position/goal/reached",Bool,goal_reached_callback)
    rospy.Subscriber("/control/position/x", Float64, position_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)

    while not rospy.is_shutdown():
      
        if(goal!= last_goal):
            last_goal = goal
            start_time = time()
           

        if(goal_reached > last_goal_reached):
            finished_time = time()
            delta_time = finished_time - start_time
            print(f"D| t{delta_time}")
            delta_time = 0 

        print(goal_reached)
        print(last_goal_reached)
        
        last_goal_reached = goal_reached

        rate.sleep()