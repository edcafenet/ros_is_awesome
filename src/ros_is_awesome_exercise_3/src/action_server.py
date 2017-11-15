#! /usr/bin/python

import rospy
import actionlib

from ros_is_awesome_exercise_3.msg import RobotFeedback, RobotResult, RobotAction
from sensor_msgs.msg import CompressedImage


class RobotAS(object):
    _feedback = RobotFeedback()
    _result = RobotResult()
    
    def __init__(self):
        
        # init the action server
        self._as = actionlib.SimpleActionServer("~/robot_action_server", RobotAction, self.Callback, False)
        self._as.start()
        
        # connect to the front camera
        self._camera = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.cameraCallback)
        
        self._result.allPictures = []
        
    def cameraCallback(self, msg):
        
        self._lastImage = msg
        
        
    def Callback(self, goal):
        r = rospy.Rate(1)
        
        success = True
        
        for i in xrange(1, goal.nseconds):
            
            # check if there are a preemption request
            if self._as.is_preempt_requested():
                rospy.loginfo('Cancelling image taking action server')
                self._as.set_preempted()
                success = False
                break
            
            self._feedback.lastImage = self._lastImage
            self._result.allPictures.append(self._lastImage)
            
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            
            r.sleep()
            
        if success:
            rospy.loginfo('Finishing.All the images have been taken')
            self._as.set_succeeded(self._result)
            
if __name__ == '__main__':
    
    rospy.init_node('robot_action_server')
    RobotAS()
    rospy.spin()
            