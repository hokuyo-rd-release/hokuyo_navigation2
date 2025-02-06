#!/usr/bin/env python3


import actionlib
import tf
import math
import dynamic_reconfigure.client


import rospy
#==================
import os
import sys
import json
#==================
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray, Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16

waypoints = PoseArray()
#change_params = []

is_insert = -1
pub = None
num = None

amcl_pose = PoseWithCovarianceStamped()
joy_button = 1

#	waypointファイルの読み込み.
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
            #if len(line) >= 3:
            #    change_params.append(line[2])
            #else:
            #    change_params.append(None)
        waypoints.header.frame_id = "map"

#	rvizで番号の表示 publish(visualization_msgs).
def rewriteMarker():
    global num
    marker_data = Marker()
    marker_data.header.frame_id = "map"
    marker_data.header.stamp = rospy.Time.now()
    marker_data.ns = "basic_shapes"
    marker_data.action = Marker.DELETEALL
    num.publish(marker_data)

    marker_data.action = Marker.ADD
    counter = 0
    marker_data.color.a = 1.0
    marker_data.scale.z = 2
    marker_data.lifetime = rospy.Duration()
    marker_data.type = Marker.TEXT_VIEW_FACING
    for pose in waypoints.poses:
        marker_data.id = counter

        marker_data.pose.position.x = pose.position.x
        marker_data.pose.position.y = pose.position.y
        marker_data.pose.orientation.z = pose.orientation.z
        marker_data.pose.orientation.w = pose.orientation.w
        marker_data.text = str(counter)

        num.publish(marker_data)
        counter +=1

#	jsonファイルへの書き込み.
def updateWaypointjson():
    file=open(sys.argv[1], 'w')
    file.write("[\n")
    for i, pose in enumerate(waypoints.poses):
        file.write("    [[{0},{1},0.0],[0.0,0.0,{2},{3}]]".format(pose.position.x,pose.position.y,pose.orientation.z,pose.orientation.w))
        if i == len(waypoints.poses)-1:
        	file.write("\n")
        else:
        	file.write(",\n")
    file.write("]")
    file.close()
    
    print("\nwaypont_writed!!\n")

#	プロンプト表示.
def printWaypoints():
    print("[")
    for pose in waypoints.poses:
        print("    [[{0},{1},0.0],[0.0,0.0,{2},{3}]],".format(pose.position.x,pose.position.y,pose.orientation.z,pose.orientation.w))
    print("]")


def fileCallback(data):
#pos = data.goal.target_pose.pose
    global pub
    global waypoints
    waypoints.poses.append(data.target_pose.pose)
    printWaypoints()
    updateWaypointjson()
    rewriteMarker()
    pub.publish(waypoints)


#	ウェイポイントの削除.
def removeCallback(removeId):
    global pub
    global waypoints
    waypoints.poses.pop(removeId)
    #printWaypoints()
    updateWaypointjson()
    rewriteMarker()
    pub.publish(waypoints)


#	ウェイポイントの追加.
def insertWaypoint(data):
    global is_insert
    if is_insert != -1:
        waypoints.poses.insert(is_insert, data.goal.target_pose.pose)
    is_insert = -1


def insertCallback(insertId):
    global pub
    global waypoints
    global is_insert
    is_insert = insertId
    print ("\nplease insert waypoint_" + str(is_insert) + " on rviz...\n")
    while is_insert != -1:
        rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, insertWaypoint)
    #printWaypoints()
    updateWaypointjson()
    rewriteMarker()
    pub.publish(waypoints)


def listener():
    global pub
    global num
        
    rospy.init_node('goal_sub', anonymous=True)
    
    pub = rospy.Publisher('waypoints', PoseArray, queue_size=100)
    num = rospy.Publisher('waypointnumber', Marker, queue_size=100)
    waypoints.header.frame_id = "map"

    load_file(sys.argv[1])
    firstLoop=True
    inputFrag=False #入力された数値が範囲内かどうかのフラグ.
    edit_mode: int = 0
    waypoint_ID: int = -1
    
    while not rospy.is_shutdown():
        if firstLoop:
            rospy.sleep(1.0)
            printWaypoints()
            rewriteMarker()
            pub.publish(waypoints)
            print("waypont_loaded")
            firstLoop = False
        else:
            #	操作内容の入力.
            inputFrag=False
            while not inputFrag:
                print("\n\nselect next operation...\n 1) \"remove waypoint\" \n 2) \"insert waypoint\" \n 3) \"replace waypoint\" \n")
                edit_mode = int(input())

                #入力内容が正しいか判断.
                inputFrag = (0 < edit_mode) and (edit_mode <= 3)
                if not inputFrag:
                    print("operation " + str(edit_mode) + " is not an option\nplease select again")
           
            #	操作するwaypoint番号の入力.
            inputFrag=False            
            while not inputFrag:
                print("\ninput waypoint_ID...\n")
                waypoint_ID = int(input())

                #入力内容が正しいか判断.
                if edit_mode == 2:
                    inputFrag = (0 <= waypoint_ID) and (waypoint_ID < len(waypoints.poses) +1)
                else:
                    inputFrag = (0 <= waypoint_ID) and (waypoint_ID < len(waypoints.poses))
                if not inputFrag:
                    print("waypoint_ID " + str(waypoint_ID) + " is out of range\nplease input again")
                
            if edit_mode == 1:
                removeCallback(waypoint_ID)
            elif edit_mode == 2:
                insertCallback(waypoint_ID)
            elif edit_mode == 3:
                insertCallback(waypoint_ID)
                removeCallback(waypoint_ID + 1)
            else:
                print("operation " + str(edit_mode) + " is not an option")
            
            edit_mode = 0
            waypoint_ID = -1
            

if __name__ == '__main__':
    print("start_wizurg_waypoint_editor")
#==============================
    if (len(sys.argv) < 2):
        print("Usage " + sys.argv[0] + " fileName")
        print(sys.argv[1])
        quit()
#==============================
    listener()
