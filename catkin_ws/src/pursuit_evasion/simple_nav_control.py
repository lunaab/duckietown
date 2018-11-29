#!/usr/bin/env python
import rospy
import numpy as np

from duckietown_msgs.msg import FSMState, BoolStamped, WheelsCmdStamped
from std_msgs.msg import String, Int16, Float32

from duckietown_msgs.srv import SetFSMState

class SimpleNavControl(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        
        # Path 0-left, 1-forward, 2-right 
        self.path = [1, 0, 2, 2, 1, 1]
        self.path.reverse()
        
        # Publishers
        self.pub_turn = rospy.Publisher("~turn_type", Int16, queue_size=1)
        
        # Subscribers
        rospy.Subscriber("/stop_line_filter_node/at_stop_line", BoolStamped, self.atIntersection)
        rospy.Subscriber("/open_loop_intersection_control_node/intersection_done", BoolStamped, self.finishedIntersection)
        
        rospy.wait_for_service("/open_loop_intersection_control_node/set_state")
        try:
            set_state = rospy.ServiceProxy("/fsm/set_state", SetFSMState)
            set_state("LANE_FOLLOWING")
        except rospy.ServiceException, e:
                print "Service call failed: %s" %e
                
        rospy.spin()
        
    def atIntersection(self, stop_msg):
        if stop_msg.data:
            rospy.wait_for_service("/fsm/set_state")
            rospy.wait_for_service("/open_loop_intersection_control_node/turn_left")
            rospy.wait_for_service("/open_loop_intersection_control_node/turn_right")
            rospy.wait_for_service("/open_loop_intersection_control_node/turn_forward")
            try:
                # Set intersection control state
                set_state = rospy.ServiceProxy("fsm/set_state", SetFSMState)
                set_state("INTERSECTION_CONTROL")
                # Tell duckie which way to turn
                turn = self.path.pop()
                turn_serv = None
                if turn == 0:
                    turn_serv = rospy.ServiceProxy("/open_loop_intersection_control_node/turn_left", Empty)
                elif turn == 1:
                    turn_serv = rospy.ServiceProxy("/open_loop_intersection_control_node/turn_forward", Empty)
                else:
                    turn_serv = rospy.ServiceProxy("/open_loop_intersection_control_node/turn_right", Empty)
                    
                turn_serv()
            except rospy.ServiceException, e:
                print "Service call failed: %s" %e
        
    def finishedIntersection(self, done_msg):
        if done_msg.data:
            rospy.wait_for_service("/fsm/set_state")
            try:
                set_state = rospy.ServiceProxy("/fsm/set_state", SetFSMState)
                set_state("LANE_FOLLOWING")
            except rospy.ServiceException, e:
                print "Service call failed: %s" %e
                
if __name__ == "__main__":
    rospy.init_node("simple_nav", anonymous=False)
    node = SimpleNavControl()
