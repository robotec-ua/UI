#!/usr/bin/env python

import rospy
import actionlib
import os
import subprocess
import csv
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Point32
from std_msgs.msg import Empty, String, Bool

ui_message_pub = rospy.Publisher('/ui_message', String, queue_size=1)
poseArray_publisher = rospy.Publisher('/WPs_topic', PoseArray, queue_size=1)

move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

output_file_path = '/home/ubuntu/edu_ws/src/BotShare_UI/botshare_nodes/saved_path/way_points.csv'
frame_id = rospy.get_param('~goal_frame_id', 'map')

waypoints = []
WPs = []
current_wp = 0

def convert_PoseWithCovArray_to_PoseArray(waypoints):
    poses = PoseArray()
    poses.header.frame_id = frame_id
    poses.poses = [pose.pose.pose for pose in waypoints]
    return poses


def WP_req_callback(data):
    del waypoints[:]
    with open(output_file_path, 'r') as file:
        reader = csv.reader(file, delimiter=',')
        for line in reader:
            current_pose = PoseWithCovarianceStamped()
            current_pose.pose.pose.position.x = float(line[0])
            current_pose.pose.pose.position.y = float(line[1])
            current_pose.pose.pose.position.z = float(line[2])
            current_pose.pose.pose.orientation.x = float(line[3])
            current_pose.pose.pose.orientation.y = float(line[4])
            current_pose.pose.pose.orientation.z = float(line[5])
            current_pose.pose.pose.orientation.w = float(line[6])
            waypoints.append(current_pose)
    poseArray_publisher.publish(
        convert_PoseWithCovArray_to_PoseArray(waypoints))


def way_point_callback(data):
    print("way_point_callback")
    line = str(data.pose.pose.position.x) + ',' + str(data.pose.pose.position.y) + ',' + str(data.pose.pose.position.z) + ',' + str(data.pose.pose.orientation.x) + \
        ',' + str(data.pose.pose.orientation.y) + ',' + \
        str(data.pose.pose.orientation.z) + ',' + \
        str(data.pose.pose.orientation.w)

    if(line not in WPs):
        WPs.append(line)

    if (len(WPs) == 1):
        ui_message_pub.publish("1 waypoint added to the route")
    else:
        ui_message_pub.publish(str(len(WPs)) + " waypoints added to the route")


def save_wp_func():
    print("save_wp_callback ")

    if(len(WPs) > 0):
        with open(output_file_path, 'a') as file:
            for WP in WPs:
                file.write(WP + '\n')
        ui_message_pub.publish(str(len(WPs)) + " waypoints saved")
    else:
        ui_message_pub.publish("Please, set at least 1 wayPoint")


def clear_way_point_func():
    global current_wp

    del WPs[:]
    with open(output_file_path, 'w') as file:
        file.write('')

    print("Waypoints cleared")
    ui_message_pub.publish("Waypoints cleared")    
    current_wp = 0

def read_wp():

    del waypoints[:]
    with open(output_file_path, 'r') as file:
        reader = csv.reader(file, delimiter=',')
        for line in reader:
            current_pose = PoseWithCovarianceStamped()
            current_pose.pose.pose.position.x = float(line[0])
            current_pose.pose.pose.position.y = float(line[1])
            current_pose.pose.pose.position.z = float(line[2])
            current_pose.pose.pose.orientation.x = float(line[3])
            current_pose.pose.pose.orientation.y = float(line[4])
            current_pose.pose.pose.orientation.z = float(line[5])
            current_pose.pose.pose.orientation.w = float(line[6])
            waypoints.append(current_pose)

    if waypoints == []:
        rospy.loginfo('The waypoint queue has been reset.')
        ui_message_pub.publish("The waypoint queue is empty.")
        return

def follow_func():
    global current_wp

    current_wp = 0
    read_wp()
    for index, waypoint in enumerate(waypoints):
        ui_message_pub.publish("Following to "+str(index+1)+" waypoint...")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.pose.position = waypoint.pose.pose.position
        goal.target_pose.pose.orientation = waypoint.pose.pose.orientation
        move_base_client.send_goal(goal)
        move_base_client.wait_for_result()        
        run_buzzer_start = subprocess.Popen("python /home/ubuntu/buzzer_3.py", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        current_wp = index+1
    ui_message_pub.publish("Current route was successfully completed")


def next_wp_func():  
    global current_wp  
    read_wp()
    current_wp +=1
    if(current_wp > len(waypoints)):  
        current_wp = 1
    ui_message_pub.publish("Following to "+str(current_wp)+" waypoint...")
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame_id
    goal.target_pose.pose.position = waypoints[current_wp-1].pose.pose.position
    goal.target_pose.pose.orientation = waypoints[current_wp-1].pose.pose.orientation
    move_base_client.send_goal(goal)
    move_base_client.wait_for_result()        
    run_buzzer_start = subprocess.Popen("python /home/ubuntu/buzzer_3.py", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        
    if(current_wp == 1):
        ui_message_pub.publish(str(current_wp) + "st point successfully reached")
    elif(current_wp == 2):
        ui_message_pub.publish(str(current_wp) + "nd point successfully reached")
    elif(current_wp == 3):
        ui_message_pub.publish(str(current_wp) + "rd point successfully reached")
    else:
        ui_message_pub.publish(str(current_wp) + "th point successfully reached")


def previous_wp_func(): 
    global current_wp
    read_wp()
    current_wp -=1
    if(current_wp < 1):  
        current_wp = len(waypoints)

    ui_message_pub.publish("Following to "+str(current_wp)+" waypoint...")
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame_id
    goal.target_pose.pose.position = waypoints[current_wp-1].pose.pose.position
    goal.target_pose.pose.orientation = waypoints[current_wp-1].pose.pose.orientation
    move_base_client.send_goal(goal)
    move_base_client.wait_for_result()        
    run_buzzer_start = subprocess.Popen("python /home/ubuntu/buzzer_3.py", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        
    if(current_wp == 1):
        ui_message_pub.publish(str(current_wp) + "st point successfully reached")
    elif(current_wp == 2):
        ui_message_pub.publish(str(current_wp) + "nd point successfully reached")
    elif(current_wp == 3):
        ui_message_pub.publish(str(current_wp) + "rd point successfully reached")
    else:
        ui_message_pub.publish(str(current_wp) + "th point successfully reached")

def home_func():
    
    ui_message_pub.publish("Following to home position")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame_id
    goal.target_pose.pose.position.x = 0
    goal.target_pose.pose.position.y = 0
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = 0
    goal.target_pose.pose.orientation.w = 1
    move_base_client.send_goal(goal)
    move_base_client.wait_for_result()

    ui_message_pub.publish("Home position successfully reached")

def ui_operation_callback(data):
    if(data.data == "follow"):
        follow_func()

    elif(data.data == "save_wp"):
        save_wp_func()

    elif(data.data == "clear"):
        clear_way_point_func()

    elif(data.data == "previous_wp"):
        previous_wp_func()

    elif(data.data == "next_wp"):
        next_wp_func()

    elif(data.data == "home"):
        home_func()


def init():

    # Initialization of node
    rospy.init_node('Follow_WP')
    rospy.loginfo(
        "____________________Start Follow_WP ______________________")
    rospy.Subscriber("/ui_operation", String, ui_operation_callback)

    rospy.Subscriber("/new_way_point",
                     PoseWithCovarianceStamped, way_point_callback)
    rospy.Subscriber("/WP_req", Empty, WP_req_callback)
    rospy.loginfo('Connecting to move_base...')
    move_base_client.wait_for_server()
    rospy.loginfo('Connected to move_base.')
    WP_req_callback(1)
    rospy.spin()


# ------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
