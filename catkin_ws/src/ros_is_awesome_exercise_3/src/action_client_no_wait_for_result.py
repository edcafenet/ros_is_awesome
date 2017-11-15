#! /usr/bin/env python

import rospy
import time
import actionlib
import cv2 as cv
import numpy as np

from geometry_msgs.msg import Twist
from ros_is_awesome_exercise_3.msg import RobotAction, RobotGoal, RobotResult, RobotFeedback

# We create some constants with the corresponing vaules from the SimpleGoalState class
PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

nImage = 1

# definition of the feedback callback. This will be called when feedback
# is received from the action server
# it just prints a message indicating a new message has been received
def feedback_callback(feedback):
    global nImage
    print('[Feedback] image n.%d received'%nImage)
    # We save the images we obtain from the feedback msgs
    np_arr = np.fromstring(feedback.lastImage.data, np.uint8)
    image_np = cv.imdecode(np_arr, cv.CV_LOAD_IMAGE_COLOR)
    cv.imwrite("rgb" + str(nImage) + ".png", image_np)
    
    nImage += 1

# initializes the action client node
rospy.init_node('example_no_waitforresult_action_client_node')

action_server_name = '/robot_action_server'
client = actionlib.SimpleActionClient(action_server_name, RobotAction)

# waits until the action server is up and running
rospy.loginfo('Waiting for action Server '+action_server_name)
client.wait_for_server()
rospy.loginfo('Action Server Found...'+action_server_name)

# creates a goal to send to the action server
goal = RobotGoal()
goal.nseconds = 10 # indicates, take pictures along 10 seconds

client.send_goal(goal, feedback_cb=feedback_callback)


# You can access the SimpleAction Variable "simple_state", that will be 1 if active, and 2 when finished.
#Its a variable, better use a function like get_state.
#state = client.simple_state
# state_result will give the FINAL STATE. Will be 1 when Active, and 2 if NO ERROR, 3 If Any Warning, and 3 if ERROR
state_result = client.get_state()

rate = rospy.Rate(1)

rospy.loginfo("state_result: "+str(state_result))

# EXAMPLE: let's prepare a publisher to move the robot
vel = Twist()
vel.angular.z = 0.2
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

while state_result < DONE:
    rospy.loginfo("Doing Stuff while waiting for the Server to give a result....")
    
    #EXAMPLE: This will definitely be executed
    pub.publish(vel)
    
    rate.sleep()
    state_result = client.get_state()
    rospy.loginfo("state_result: "+str(state_result))
    
rospy.loginfo("[Result] State: "+str(state_result))
if state_result == ERROR:
    rospy.logerr("Something went wrong in the Server Side")
if state_result == WARN:
    rospy.logwarn("There is a warning in the Server Side")
