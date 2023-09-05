#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped

class MiRoController:
    def __init__(self):

        rospy.init_node("miro_controller", anonymous=True)
        
        self.vel_pub = rospy.Publisher("/pacman/control/cmd_vel", TwistStamped, queue_size=1)
        
        self.rate = rospy.Rate(10)
        
    def move_forward(self):
      
        msg_cmd_vel = TwistStamped()
        msg_cmd_vel.twist.linear.x = 1.0
        msg_cmd_vel.twist.angular.z = 0.0  
        
        self.vel_pub.publish(msg_cmd_vel)
    
    def move_back(self):

        msg_cmd_vel = TwistStamped()
        msg_cmd_vel.twist.linear.x = -0.2  
        msg_cmd_vel.twist.angular.z = 0.0  
        
        self.vel_pub.publish(msg_cmd_vel)

    def turn_left(self):

        msg_cmd_vel = TwistStamped()
        msg_cmd_vel.twist.linear.x = 0.0
        msg_cmd_vel.twist.angular.z = 0.2  

        angular_distance = 1.5708  # 90 degrees in radians
        rotation_time = angular_distance / 0.2

        start_time = rospy.get_time()
        while rospy.get_time() - start_time < rotation_time and not rospy.is_shutdown():
            self.vel_pub.publish(msg_cmd_vel)
            rospy.sleep(0.1)

        msg_cmd_vel.twist.angular.z = 0.0
        
        self.vel_pub.publish(msg_cmd_vel)

    def turn_right(self):

        msg_cmd_vel = TwistStamped()
        msg_cmd_vel.twist.linear.x = 0.0
        msg_cmd_vel.twist.angular.z = -0.4  # Adjust the angular velocity

        self.vel_pub.publish(msg_cmd_vel)

        start_time = rospy.Time.now()
        rotate_duration = rospy.Duration.from_sec(2.25)  # Adjust as needed

        rate = rospy.Rate(10)  # Control loop rate
        while not rospy.is_shutdown():
            if rospy.Time.now() - start_time >= rotate_duration:
                # Stop the rotation
                msg_cmd_vel.twist.angular.z = 0.0
                self.vel_pub.publish(msg_cmd_vel)
                break

            self.vel_pub.publish(msg_cmd_vel)
            rate.sleep()
        
    def stop(self):
        
        msg_cmd_vel = TwistStamped()
        msg_cmd_vel.twist.linear.x = 0.0
        msg_cmd_vel.twist.angular.z = 0.0
        
        self.vel_pub.publish(msg_cmd_vel)

    def main_function(self):
        # main loop to control the miro to move 
        while not rospy.is_shutdown():
            self.turn_right()  
            self.rate.sleep()  # 控制发布指令的频率

if __name__ == "__main__":
    miro_controller = MiRoController()  # create miro object
    miro_controller.main_function()  # start the loop

