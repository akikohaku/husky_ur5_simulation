#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
from time import sleep
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from actionlib.action_client import GoalManager
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf_conversions import transformations
from math import pi  
import tf

robot = moveit_commander.RobotCommander()
arm = moveit_commander.MoveGroupCommander('manipulator')
end_effector_link = arm.get_end_effector_link()

class Robot:
    def __init__(self):
        self.tf_listener = tf.TransformListener()    
        #创建了监听器，它通过线路接收tf转换，并将其缓冲10s。若用C++写，需设置等待10s缓冲。
        try:
            self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(1.0))    
            #猜测：等待Duration=1s，判断map是否转换为base_link
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return

 
    def get_pos(self):
        
        (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))

        return(trans)

class Camera:
    def __init__(self):
        self.tf_listener = tf.TransformListener()    
        #创建了监听器，它通过线路接收tf转换，并将其缓冲10s。若用C++写，需设置等待10s缓冲。
        try:
            self.tf_listener.waitForTransform('/map', '/camera_link', rospy.Time(), rospy.Duration(1.0))    
            #猜测：等待Duration=1s，判断map是否转换为base_link
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return

 
    def get_pos(self):
        
        (trans, rot) = self.tf_listener.lookupTransform('/map', '/camera_link', rospy.Time(0))

        return(trans)




def main(): #进行初始化操作
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('huskuy_ur5_controller', anonymous=True)
    robotpos=Robot()
    camerapos=Camera()
    sleep(2)
    reach_goal(2.4,0.8,0.6,0,1.5,1.5,0.9,robotpos,camerapos)
def demo():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('huskuy_ur5_controller', anonymous=True)
    uping()
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 0.9
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    arm_move_to_goal(pose_goal)
    sleep(7)
    goal0 = MoveBaseGoal()
    goal0.target_pose.pose.position.x = 1.84500110149
    goal0.target_pose.pose.position.y = -0.883078575134
    goal0.target_pose.pose.orientation.z = 0.0
    goal0.target_pose.pose.orientation.w = 0.951839761956
    husky_move_to_goal(goal0)

def homing():
    move_group = arm
    move_group.set_named_target('home')
    move_group.go()
    rospy.sleep(1)

def uping():
    move_group = arm
    move_group.set_named_target('up')
    move_group.go()
    rospy.sleep(1)

def arm_move_to_goal(pose_goal):
    move_group = arm
        
    move_group.set_start_state_to_current_state()
    move_group.set_pose_target(pose_goal)
        
    plan = move_group.go(wait=True)        
    move_group.stop()
    move_group.clear_pose_targets()

def husky_move_to_goal(goal):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()
    client.send_goal(goal)
    wait = client.wait_for_result(rospy.Duration.from_sec(60.0))

def reach_goal(x,y,z,rx,ry,rz,w,robotpos,camerapos):
    now_pos=robotpos.get_pos()
    print(now_pos)
    if pow(x-now_pos[0],2)+pow(y-now_pos[1],2)+pow(z+0.25-now_pos[2],2)>0.9 :
        print("start move")
        goal0 = MoveBaseGoal()
        goal0.target_pose.pose.position.x = x
        goal0.target_pose.pose.position.y = y
        goal0.target_pose.pose.position.z = 0
        goal0.target_pose.pose.orientation.w = w
        husky_move_to_goal(goal0)
        print("start arm")
        now_pos=camerapos.get_pos()
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = w
        pose_goal.orientation.x = rx
        pose_goal.orientation.y = ry
        pose_goal.orientation.z = rz
        pose_goal.position.x = x-now_pos[0]
        pose_goal.position.y = y-now_pos[1]
        pose_goal.position.z = z-now_pos[2]
        arm_move_to_goal(pose_goal)
    else:
        print("start arm")
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = w
        pose_goal.position.x = x-now_pos[0]
        pose_goal.position.y = y-now_pos[1]
        pose_goal.position.z = z-now_pos[2]
        pose_goal.orientation.x = rx
        pose_goal.orientation.y = ry
        pose_goal.orientation.z = rz
        print("arm target",pose_goal)
        arm_move_to_goal(pose_goal)
    print("now pos",camerapos.get_pos())

if __name__=="__main__":
    main()



