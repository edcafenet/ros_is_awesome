#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time

class Move():
    
    def __init__(self):
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        self.rate = rospy.Rate(10) # 10hz
    
    
    def publish_once_in_cmd_vel(self, cmd):
        """
        This is because publishing in topics sometimes fails the  first time you publish.
        In continuos publishing systems there is no big deal but in systems that publish only
        once it IS very important.
        """
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
            if connections > 0:
                self.vel_publisher.publish(cmd)
                rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()
    
    
    def shutdownhook(self):
            # works better than the rospy.is_shut_down()
            self.stop()
            self.ctrl_c = True

    def stop(self):
        rospy.loginfo("shutdown time! Stop the robot")
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel(cmd)


    def move_x_time(self, moving_time, linear_speed=0.2, angular_speed=0.2):
        
        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed
        
        rospy.loginfo("Moving Forwards")
        self.publish_once_in_cmd_vel(cmd)
        time.sleep(moving_time)
        self.stop()
        rospy.loginfo("######## Finished Moving Forwards")
    
    def move_square(self):
        
        i = 0
        while not self.ctrl_c and i < 4:
            # Move Forwards
            self.move_x_time(moving_time=2.0, linear_speed=0.2, angular_speed=0.0)
            # Stop
            self.move_x_time(moving_time=4.0, linear_speed=0.0, angular_speed=0.0)
            # Turn 
            self.move_x_time(moving_time=3.5, linear_speed=0.0, angular_speed=0.2)
            # Stop
            self.move_x_time(moving_time=0.1, linear_speed=0.0, angular_speed=0.0)
            
            i += 1
        rospy.loginfo("######## Finished Moving in a Square")
        
if __name__ == '__main__':
    rospy.init_node('move_test', anonymous=True)
    move_object = Move()
    try:
        move_object.move_square()
    except rospy.ROSInterruptException:
        pass