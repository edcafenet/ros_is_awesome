#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse # you import the service message python classes generated from Empty.srv.
from move_robot import Move

def my_callback(request):
    rospy.loginfo("The service move_in_square has been called")
    robot.move_square()
    rospy.loginfo("Success!")
    return EmptyResponse()

rospy.init_node('service_server') 
robot = Move()
my_service = rospy.Service('/move_in_square', Empty , my_callback) # create the Service called my_service with the defined callback
rospy.spin() # mantain the service open.