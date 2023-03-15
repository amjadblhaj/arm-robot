#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys

def perform_trajectory():
    rospy.init_node('gripper_trajectory_publisher')
    contoller_name='/gripper_controller/command'
    trajectory_publihser = rospy.Publisher(contoller_name,JointTrajectory, queue_size=10)
    argv = sys.argv[1:]                         
    gripper_joints = ['joint_6','joint_7']
    goal_positions = [ float(argv[0]) , float(argv[1]) ]
 
    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = gripper_joints
    trajectory_msg.points.append(JointTrajectoryPoint())
    trajectory_msg.points[0].positions = goal_positions
    trajectory_msg.points[0].velocities = [0.0 for i in gripper_joints]
    trajectory_msg.points[0].accelerations = [0.0 for i in gripper_joints]
    trajectory_msg.points[0].time_from_start = rospy.Duration(3)
    rospy.sleep(1)
    trajectory_publihser.publish(trajectory_msg)


if __name__ == '__main__':
    perform_trajectory()