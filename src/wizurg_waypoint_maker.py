#!/usr/bin/env python3
# amcl_pose: posestampedwithcovariance から、estimated_pose: posestamped に変更
import rospy
#==================
import sys
import json
#==================
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray, Pose, PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16

waypoints = PoseArray()
way_distance = 5
way_rad_distance = 1.57
is_insert = -1
pub = None
num = None

lio_loc_pose = PoseStamped()
joy_button = 1


#	rvizで矢印の表示 publish(visualization_msgs).
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
    marker_data.scale.z = 5
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
    
    print("waypont_writed")

#	プロンプト表示.
def printWaypoints():
    print("[")
    for pose in waypoints.poses:
        print("    [[{0},{1},0.0],[0.0,0.0,{2},{3}]],".format(pose.position.x,pose.position.y,pose.orientation.z,pose.orientation.w))
    print("]")


#	rvizで入力されたゴールをwaypointに追加.
def goalCallback(data):
    global pub
    global waypoints
    waypoints.poses.append(data.goal.target_pose.pose)
    printWaypoints()
    updateWaypointjson()
    rewriteMarker()
    pub.publish(waypoints)
    print("waypont_added")


#   amclの自己位置を更新.
def amclCallback(data):
    global lio_loc_pose
    lio_loc_pose = data 
    
    
#   amclの自己位置をwaypointに追加.
def amclWaypointAppend():
    global pub
    global waypoints
    global lio_loc_pose
    waypoints.poses.append(lio_loc_pose.pose) # 2/10 髙橋変更 ()
    printWaypoints()
    updateWaypointjson()
    rewriteMarker()
    pub.publish(waypoints)

def amclTimer(event):
    amclWaypointAppend()


#   joycon入力時.
def joyCallback(joy_msg):
    if joy_msg.buttons[joy_button] == 1:
        print("put")
        amclWaypointAppend()


#   rvizで入力されたinitial_poseをjsonファイルに出力
def initPoseCallback(data):

    initPoseDict = {}
    initPoseDict["frame_id"] = data.header.frame_id
    initPoseDict["Pos_x"] = data.pose.pose.position.x
    initPoseDict["Pos_y"] = data.pose.pose.position.y
    initPoseDict["Pos_z"] = data.pose.pose.position.z
    initPoseDict["Ori_x"] = data.pose.pose.orientation.x
    initPoseDict["Ori_y"] = data.pose.pose.orientation.y
    initPoseDict["Ori_z"] = data.pose.pose.orientation.z
    initPoseDict["Ori_w"] = data.pose.pose.orientation.w
    initPoseDict["cov"] = data.pose.covariance
    
    with open(("initial_" + sys.argv[1]), 'w') as file:
        json.dump(initPoseDict, file)
    
    print("Initial_pose saved")
    

def fileCallback(data):
#pos = data.goal.target_pose.pose
    global pub
    global waypoints
    waypoints.poses.append(data.target_pose.pose)
    printWaypoints()
    updateWaypointjson()
    rewriteMarker()
    pub.publish(waypoints)


def removeCallback(removeId):
    global pub
    global waypoints
    waypoints.poses.pop(removeId.data)
    printWaypoints()
    updateWaypointjson()
    rewriteMarker()
    pub.publish(waypoints)


def insertWaypoint(data):
    global is_insert
    if is_insert != -1:
        waypoints.poses.insert(is_insert, data.goal.target_pose.pose)
        waypoints.poses.pop()
    is_insert = -1


def insertCallback(insertId):
    global pub
    global waypoints
    global is_insert
    is_insert = insertId.data
    print ("insert" + str(is_insert))
    while is_insert != -1:
        rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, insertWaypoint)
    printWaypoints()
    updateWaypointjson()
    rewriteMarker()
    pub.publish(waypoints)


def listener():
    global pub
    global num
    global joy_button
    rospy.init_node('goal_sub', anonymous=True)
    
    rospy.Subscriber("/estimated_pose", PoseStamped, amclCallback)
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, initPoseCallback)
    rospy.Subscriber('/joy', Joy, joyCallback)
    
    rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, goalCallback)
    rospy.Subscriber("/file", MoveBaseGoal, fileCallback)
    rospy.Subscriber("/remove", Int16, removeCallback)
    rospy.Subscriber("/insert", Int16, insertCallback)

    rospy.Timer(rospy.Duration(10), amclTimer) 

    pub = rospy.Publisher('waypoints', PoseArray, queue_size=10)
    num = rospy.Publisher('waypointnumber', Marker, queue_size=10)
    joy_button = rospy.get_param("/low_button", joy_button)
    waypoints.header.frame_id = "map"

    # 引数で繰り返しmodeの指定がある場合.
    if (len(sys.argv) > 2):
       print("未実装")

    rospy.spin()

if __name__ == '__main__':
    print("start_wizurg_waypoint_maker")
#==============================
    if (len(sys.argv) < 2):
        print("Usage " + sys.argv[0] + " fileName")
        print(sys.argv[1])
        quit()
#==============================
    listener()
