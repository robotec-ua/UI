#!/usr/bin/env python3
import rospy
import roslaunch
from std_msgs.msg import String, Bool
import os
import subprocess
import time
from pathlib import Path 

pose_path = '/home/ubuntu/edu_ws/src/BotShare_UI/botshare_nodes/saved_path/way_points.csv'

launch_gmapping = None
launch_wp = None

ui_pub = rospy.Publisher('ui_message', String, queue_size=1)

clear_message = "clear"
dock_message = Bool()
dock_message.data = True
undock_message = Bool()
undock_message.data = True

run_motors_start = subprocess.Popen("python /home/ubuntu/enable_motors.py", stdout=subprocess.PIPE,
                                       shell=True, preexec_fn=os.setsid)

run_buzzer_start = subprocess.Popen("python /home/ubuntu/buzzer.py", stdout=subprocess.PIPE,
                                       shell=True, preexec_fn=os.setsid)
def ui_callback(data):
    global launch_gmapping
    global launch_wp
    
    if(data.data == "build_map"):
        
        print("Start building map... Please wait for service")
        ui_pub.publish("Start building map... Please wait for service")

        os.system("rosnode kill /follow_waypoints")
        os.system("rosnode kill /amcl")
        os.system("rosnode kill /move_base")
        os.system("rosnode kill /map_server")

        with open(pose_path, 'w') as file:
            file.write('')

        time.sleep(3)

        print("SLAM")
        launch_gmapping = subprocess.Popen("roslaunch linorobot slam.launch", stdout=subprocess.PIPE,
                                           shell=True, preexec_fn=os.setsid)
        time.sleep(6)

        print("Move the robot along the perimeter of the room and in the center using the buttons ")
        ui_pub.publish("Move the robot along the perimeter of the room and in the center using the buttons ")

    elif(data.data == "save_map"):

        print("Saving map...")
        ui_pub.publish("Saving map...")

        command = "rosrun map_server map_saver -f " + str(Path.home()) + "/edu_ws/src/linorobot/maps/my_map"
        os.system(command)
        
        time.sleep(2)

        print("Map saved, please return robot in start position")
        ui_pub.publish("Map saved please, return robot in start position")

        time.sleep(1)
        
        print("Wait for robot localization... ")
        ui_pub.publish("Wait for robot localization... ")

        os.system("rosnode kill /slam_gmapping")

        time.sleep(8)
        launch_amcl = subprocess.Popen("roslaunch linorobot navigate.launch", stdout=subprocess.PIPE,
                                       shell=True, preexec_fn=os.setsid)
                                       
        time.sleep(5)        
        launch_wp = subprocess.Popen("roslaunch follow_waypoints follow_waypoints.launch", stdout=subprocess.PIPE,
                                     shell=True)

        time.sleep(5)
        run_buzzer = subprocess.Popen("python /home/ubuntu/buzzer.py", stdout=subprocess.PIPE,
                                       shell=True, preexec_fn=os.setsid) 

        print("You can set points or follow the route")
        ui_pub.publish("You can set points or follow the route")
 

def listener():
    rospy.init_node('ui_operation', anonymous=True)
    rospy.Subscriber("ui_operation", String, ui_callback)
    rospy.spin()


if __name__ == '__main__':
    listener()