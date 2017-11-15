#! /usr/bin/env python
import rospy
import time
import actionlib
import cv2 as cv
import numpy as np

from ros_is_awesome_exercise_3.msg import RobotFeedback, RobotResult, RobotAction, RobotGoal

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
rospy.init_node('robot_action_client')

# create the connection to the action server
client = actionlib.SimpleActionClient('/robot_action_server', RobotAction)

# waits until the action server is up and running
client.wait_for_server()

# creates a goal to send to the action server
goal = RobotGoal()
goal.nseconds = 10 # indicates, take pictures along 10 seconds

# sends the goal to the action server, specifying which feedback function
# to call when feedback received
client.send_goal(goal, feedback_cb=feedback_callback)

# we wait until a result is obtained
client.wait_for_result()

print('[Result] State: %d'%(client.get_state()))