#!/usr/bin/env python
import rospy
import yaml
from duckietown_msgs.msg import AprilTagsWithInfos, TagInfo, AprilTagDetectionArray
from std_msgs.msg import String, Int16MultiArray, Float32
import numpy as np

class AprilTagsNav(object):
    """ """
    def __init__(self):    
        """ """
        self.node_name = rospy.get_name()
        self.pub_int = rospy.Publisher("~intersection_info", Int16MultiArray, queue_size=1)
        rospy.Subscriber("~apriltags_info", AprilTagsWithInfos, self.processTags)
        
    def processTags(self, msg):
        intersection = -1
        interprev = -1
        tagid = -1
        for detection in msg.detections:
            tagid = int(detection.id)
            if tagid == 2 or tagid == 3 or tagid == 4:
                if tagid == 2:
                    interprev = 1
                elif tagid == 3:
                    interprev = 3
                elif tagid == 4:
                    interprev = 2
                intersection = 0
            elif tagid == 5 or tagid == 6 or tagid == 7:
                if tagid == 5:
                    interprev = 0
                elif tagid == 6:
                    interprev = 2
                elif tagid == 7:
                    interprev = 4
                intersection = 3
            elif tagid == 8 or tagid == 9 or tagid == 10 or tagid == 11:
                if tagid == 8:
                    interprev = 0
                elif tagid == 9:
                    interprev = 3
                elif tagid == 10:
                    interprev = 4
                elif tagid == 11:
                    interprev = 1
                intersection = 2
            elif tagid == 12 or tagid == 13 or tagid == 14:
                if tagid == 12:
                    interprev = 0
                elif tagid == 13:
                    interprev = 2
                elif tagid == 14:
                    interprev = 4
                intersection = 1
            elif tagid == 15 or tagid == 16 or tagid == 17:
                if tagid == 15:
                    interprev = 2
                elif tagid == 16:
                    interprev = 3
                elif tagid == 17:
                    interprev = 1
                intersection = 4
        new_inter = Int16MultiArray()
        new_inter.data = [interprev,intersection]
        self.pub_int.publish(new_inter)
        


if __name__ == '__main__': 
    rospy.init_node('april_nav',anonymous=False)
    node = AprilTagsNav()
    rospy.spin()
