#!/usr/bin/env python3

import rospy
import os
import actionlib
import tf
import sys
from nav_msgs.msg import Odometry
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import json
from geometry_msgs.msg import PoseArray, Pose, PoseWithCovarianceStamped
import dynamic_reconfigure.client

waypoints = PoseArray()
change_params = []

#==============
initial_pose = PoseWithCovarianceStamped()
#==============

def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose = pose

    return goal_pose

#=====================
def load_initfile(fileName):
    global initial_pose
    with open(fileName, "r") as f:
        initPoseDict = json.load(f)
        initial_pose.header.frame_id = initPoseDict["frame_id"]
        initial_pose.pose.pose.position.x = initPoseDict["Pos_x"]
        initial_pose.pose.pose.position.y = initPoseDict["Pos_y"]
        initial_pose.pose.pose.position.z = initPoseDict["Pos_z"]
        initial_pose.pose.pose.orientation.x = initPoseDict["Ori_x"]
        initial_pose.pose.pose.orientation.y = initPoseDict["Ori_y"]
        initial_pose.pose.pose.orientation.z = initPoseDict["Ori_z"]
        initial_pose.pose.pose.orientation.w = initPoseDict["Ori_w"]
        initial_pose.pose.covariance = initPoseDict["cov"]
        
    print("initial_pose_loaded")
        
#=====================

def load_file(fileName):
    with open(fileName, "r") as f:
        waypointFile = json.load(f)
        for line in waypointFile:
            pose = Pose()
            pose.position.x = line[0][0]
            pose.position.y = line[0][1]
            pose.position.z = line[0][2]
            pose.orientation.x = line[1][0]
            pose.orientation.y = line[1][1]
            pose.orientation.z = line[1][2]
            pose.orientation.w = line[1][3]
            waypoints.poses.append(pose)
            if len(line) >= 3:
                change_params.append(line[2])
            else:
                change_params.append(None)
        waypoints.header.frame_id = "map"
    #print(waypoints)

def angle_dif(target, current):
    diff = target - current
    if (diff > math.pi):
        diff -= 2 * math.pi
    elif(diff < -math.pi):
        diff += 2 * math.pi
    return abs(diff)

if __name__ == '__main__':
    print("start_wizurg_navigation")
    rospy.init_node('wizurg_navigation')
    pub = rospy.Publisher('waypoints', PoseArray, queue_size=10)
    yo_tolerance = rospy.get_param('move_base/DWAPlannerROS/yaw_goal_tolerance')
    xy_tolerance = rospy.get_param('move_base/DWAPlannerROS/xy_goal_tolerance')
    if (len(sys.argv) < 2):
        print("Usage " + sys.argv[0] + " fileName")
        quit()
        
    #==initial_waypoint.jsonがあるなら初期位置をパブリッシュ.============
    init_pose_file = "initial_" + sys.argv[1]
    if os.path.isfile(init_pose_file):
        initpub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
        load_initfile(init_pose_file)
        print("published initial_pose")
    #==================================================================
        

    load_file(sys.argv[1])
    listener = tf.TransformListener()

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(4.0))
    dynamic_client = dynamic_reconfigure.client.Client("amcl", timeout=30)

    #=============
    count=0
    firstLoop=True
    #===========
    while not rospy.is_shutdown():
        if os.path.isfile(init_pose_file) and firstLoop:
            initpub.publish(initial_pose)

        for i in range(len(waypoints.poses)):
            pose = waypoints.poses[i]
            pub.publish(waypoints)
            goal = goal_pose(pose)
            client.send_goal(goal)
            if not change_params[i] == None:
                dynamic_client.update_configuration(change_params[i])

            #========自己位置とwaypointが近づくまでループ=========
            while not rospy.is_shutdown():
                now = rospy.Time()
                listener.waitForTransform("map", "base_link", now, rospy.Duration(4.0))

                position, quaternion = listener.lookupTransform("map", "base_link", now)
                euler = tf.transformations.euler_from_quaternion((quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
                goal_euler = tf.transformations.euler_from_quaternion((goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w))
                #print("rad = "+str(angle_dif(goal_euler[2], euler[2])))
                client.wait_for_result(rospy.Duration(0.1))
                
                
#===========================================
                if (math.sqrt(((position[0]-goal.target_pose.pose.position.x)/(xy_tolerance*5))**2 + ((position[1]-goal.target_pose.pose.position.y)/(xy_tolerance*5))**2 + (angle_dif(goal_euler[2], euler[2])/(yo_tolerance*2)) ) <= 1):
                    count=count+1
                    #print("Node Goes next!!")
                elif(client.get_result()):
                    count=count+1
                    #print("Client next!!")
                if(count>3):
                    print("reached point_" + str(i))
                    print("Next Point!!")
                    count=0
                    break
                    
        if firstLoop:
            firstLoop = False             
#==========複数マップモードのとき、繰り返さず終了===================================
        if (len(sys.argv) > 2):
            if (sys.argv[2] == "once"):
                break
#=============================================

