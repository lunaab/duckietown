#!/usr/bin/env python
import rospy
import rospkg
import numpy as np
import copy

from duckietown_msgs.msg import FSMState, BoolStamped, WheelsCmdStamped
from std_msgs.msg import String, Int16, Float32, Int16MultiArray

from duckietown_msgs.srv import SetFSMState

from Graph import DuckieGraph, GraphSearch

class Criminal(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        
        # State Variables
        self.sent_turn = False
        self.delay = 2.0
        
        # Graph of Duckietown
        self.graph = DuckieGraph()
        rospack = rospkg.RosPack()
        filename = rospack.get_path('pursuit_evasion')
        filename = filename + '/src/duckiemap.csv'
        self.graph.build_DTown(filename)
        self.curr_pose = (0, 2)
        
        # Path 0-left, 1-forward, 2-right
        #FLLLLF
        self.turn_dict = {'L': 0, 'F': 1, 'R': 2}
        self.pure_path = [1, 0, 0, 0, 0, 1]
        self.path = copy.deepcopy(self.pure_path)
        self.path.reverse()
        
        # Publishers
        self.pub_pose = rospy.Publisher("~criminal_pose", Int16MultiArray, queue_size=1)
        
        #rospy.sleep(4.8 + self.delay)
        while True:
            #self.atIntersection(0)
            #self.sent_turn = False
            msg = Int16MultiArray()
            msg.data = [0, 3]
            self.pub_pose.publish(msg)
                
        
    def atIntersection(self, stop_msg):
        if not self.sent_turn:
            self.sent_turn = True
            rospy.sleep(1)
            turn = self.path.pop()
            if len(self.path) == 0:
                self.path = copy.deepcopy(self.pure_path)
                self.path.reverse
            self.changePose(turn)
                
                
    def changePose(self, turn):
        curr_idx = self.graph.find_vertex(self.curr_pose)
        children = self.graph.children(curr_idx)
        for child in children:
            if self.turn_dict[child[1]] == turn:
                self.curr_pose = self.graph.vertices[child[0]]
                msg = Int16MultiArray()
                msg.data = [self.curr_pose[0], self.curr_pose[1]]
                self.pub_pose.publish(msg)
        print child[2]
        rospy.sleep(child[2] + self.delay)
        
                
                
if __name__ == "__main__":
    rospy.init_node("criminal_nav", anonymous=False)
    node = Criminal()
    rospy.spin()
