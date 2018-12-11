#!/usr/bin/env python
import rospy
import rospkg
import numpy as np

from duckietown_msgs.msg import FSMState, BoolStamped, WheelsCmdStamped
from std_msgs.msg import String, Int16, Float32, Int16MultiArray

from duckietown_msgs.srv import SetFSMState

from Graph import DuckieGraph, GraphSearch

class PursuitNavControl(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        
        # State Variables
        self.sent_turn = False
        
        # Graph of Duckietown
        self.graph = DuckieGraph()
        rospack = rospkg.RosPack()
        filename = rospack.get_path('pursuit_evasion')
        filename = filename + '/src/duckiemap.csv'
        self.graph.build_DTown(filename)
        self.searcher = GraphSearch()
        self.curr_pose = None
        
        # Path 0-left, 1-forward, 2-right
        self.turn_dict = {'L': 0, 'F': 1, 'R': 2}
        self.path = [1, 0, 2, 2, 1, 1]
        self.path.reverse()
        
        # Publishers
        self.pub_turn = rospy.Publisher("~turn_type", Int16, queue_size=1)
        
        # Subscribers
        rospy.Subscriber("~intersection_info", Int16MultiArray, self.setPose)
        rospy.Subscriber("~criminal_pose", Int16MultiArray, self.planRoute)
        rospy.Subscriber("~at_stop_line", BoolStamped, self.atIntersection)
        rospy.Subscriber("~intersection_done", BoolStamped, self.finishedIntersection)
                
        
    def atIntersection(self, stop_msg):
        if bool(stop_msg.data):
            if not self.sent_turn:
                self.sent_turn = True
                rospy.sleep(1)
                turn = self.path.pop()
                cmd = Int16()
                cmd.data = turn
                self.pub_turn.publish(cmd)
                rospy.logwarn(self.curr_pose)
                self.changePose(turn)
                
    def changePose(self, turn):
        curr_idx = self.graph.find_vertex(self.curr_pose)
        children = self.graph.children(curr_idx)
        for child in children:
            if self.turn_dict[child[1]] == turn:
                self.curr_pose = self.graph.vertices[child[0]]
                rospy.logwarn(self.curr_pose)

        
    def finishedIntersection(self, done_msg):
        if bool(done_msg.data):
            rospy.logwarn(self.curr_pose)
            self.sent_turn = False
                
    def planRoute(self, crim_pose):
        #Pose: leaving, heading
        self.target = crim_pose.data[1]
        if self.curr_pose is not None:
            cmd_path = self.searcher.shortest_path(self.graph.find_vertex(self.curr_pose), self.target, self.graph)
            self.path = []
            for cmd in cmd_path:
                self.path.append(self.turn_dict[cmd])
            self.path.reverse()
        
    def setPose(self, pose_msg):
        if self.curr_pose is not None:
            if pose_msg.data[0] != self.curr_pose[1]:
                pass
            else:
                self.curr_pose = (pose_msg.data[0], pose_msg.data[1])
        else:
            self.curr_pose = (pose_msg.data[0], pose_msg.data[1])
        '''if self.curr_pose is None:
            self.curr_pose = (pose_msg.data[0], pose_msg.data[1])
        rospy.logwarn(self.curr_pose)'''
                
if __name__ == "__main__":
    rospy.init_node("pursuit_nav", anonymous=False)
    node = PursuitNavControl()
    rospy.spin()
