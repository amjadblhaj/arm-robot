#!/usr/bin/env python3
from math import atan, cos, sin, pi ,pow 
from std_msgs.msg import Float64
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
import sys

def callback(pose_msg): 
    rospy.loginfo("Received pose:".format(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z, pose_msg.orientation.w))
    x=pose_msg.position.x
    y=pose_msg.position.y 
    z=pose_msg.position.z
    r=pose_msg.orientation.w
     
    if x==0:
     x=0.0001

    j1 = float( atan(y/x))
    if x< 0:
     j1= pi + j1
      
    j4 = r*(pi/180)
    j5 = 0.495 - z
    if 0.8 <= (pow(pow(x,2)+pow(y,2),0.5)):
      j2=0
      j3= (x/cos(j1))-0.8
    else:
      j3=0
      j2=(x/cos(j1))-0.8 

    
    goal_positions=[float(j1), float(j2), float(j3) , float(j4), float(j5) ]
    contoller_name='/arm_controller/command'
    trajectory_publihser = rospy.Publisher(contoller_name,JointTrajectory, queue_size=10)                      
    arm_joints = ['joint_1','joint_2','joint_3','joint_4','joint_5']
    print(goal_positions)
    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = arm_joints
    trajectory_msg.points.append(JointTrajectoryPoint())
    trajectory_msg.points[0].positions = goal_positions
    trajectory_msg.points[0].velocities = [0.0 for i in arm_joints]
    trajectory_msg.points[0].accelerations = [0.0 for i in arm_joints]
    trajectory_msg.points[0].time_from_start = rospy.Duration(3)
    rospy.sleep(1)
    trajectory_publihser.publish(trajectory_msg)
    
def perform_trajectory():
  
    rospy.init_node('perform_trajectory' , anonymous=True)  
    rospy.Subscriber('myrobot',Pose,callback)
    rospy.spin()  


if __name__ == '__main__':
     perform_trajectory()
     
       