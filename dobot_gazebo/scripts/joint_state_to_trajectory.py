#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointStateToTrajectory:
    def __init__(self):
        rospy.init_node('joint_state_to_trajectory')
        
        self.trajectory_pub = rospy.Publisher('/set_joint_trajectory', JointTrajectory, queue_size=10)
        
        self.joint_state_sub = rospy.Subscriber('/move_group/fake_controller_joint_states', JointState, self.joint_state_callback)
        
        rospy.loginfo("JointState to JointTrajectory converter started")
        
    def joint_state_callback(self, msg):
        trajectory = JointTrajectory()
        trajectory.header = msg.header
        trajectory.joint_names = msg.name
        
        point = JointTrajectoryPoint()
        point.positions = msg.position
        point.time_from_start = rospy.Duration(0.1)  # 100ms to reach target
        
        trajectory.points = [point]
        
        self.trajectory_pub.publish(trajectory)
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        converter = JointStateToTrajectory()
        converter.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Converter node interrupted")