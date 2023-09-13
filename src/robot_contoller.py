#!/usr/bin/env python3

import rospy
import rostopic
import argparse
from dataclasses import dataclass

from geometry_msgs.msg import Twist, TwistStamped, Pose2D
from nav_msgs.msg import Odometry


# parse args
cli = argparse.ArgumentParser(
    prog='AI_Pac-man',
    description='Script to control the Pac-man and Ghosts autonomously', 
    epilog='Good luck!'
)
cli.add_argument(
    "--robot_ns",
    dest="ns",
    default="blinky",
    help="Robot namespace"
)
cli.add_argument(
    "--gazebo_ns",
    dest="gz",
    default="/gazebo_server",
    help="Gazebo topic path" 
)

@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

class RobotController:
    def __init__(self):

        # Process args and params
        self.args = cli.parse_args(rospy.myargv()[1:])

        self.ns = str(self.args.ns)
        self.ns = self.ns[1:] if self.ns[0] == '/' else self.ns # for consistency

        # Create node
        rospy.init_node(self.ns + "_controller", anonymous=False)
       
        # Find relevant topics within the provided ns
        pub_list = rostopic.get_topic_list()[0]  # Look through Publishers
        self.odom_topic = [t[0] for t in pub_list 
                        if ('/' + self.ns in t[0] and 'odom' in t[0])][0]  
        sub_list = rostopic.get_topic_list()[1]  # Look through Subscribers

        self.cmd_vel_topic = [t[0] for t in sub_list 
                              if ('/' + self.ns in t[0] and 'cmd_vel' in t[0])][0]
        if not self.odom_topic:
            print(f"Could not find the required /odom topic for the namespace {self.ns}")
            raise SystemExit
        if not self.cmd_vel_topic:
            print(f"Could not find the required /cmd_vel topic for the namespace {self.ns}")
            raise SystemExit        

        # Create Publishers and Subscribers       
        self.cmd_vel_msg_type, topic_str, _ = rostopic.get_topic_class(self.cmd_vel_topic)    
        self.vel_pub = rospy.Publisher(self.cmd_vel_topic, self.cmd_vel_msg_type, queue_size=1)

        self.pos_sub = rospy.Subscriber(self.odom_topic, Odometry, self.pose_cb, queue_size=1)   
        self.rate = rospy.Rate(10)

        # we need this to be able to manipulate the object states
        self.gz_ns = str(self.args.gz)
        
        # Create variables
        self.robot = Pose() 
    
    def pose_cb(self, data):
        self.robot.x = 1
        self.robot.y = 1
        self.robot.theta = 1
        # print(self.robot)
  
    def pub_cmd_vel(self, linear = 1.0, angular = 0.5):
        msg = self.cmd_vel_msg_type()
        if self.cmd_vel_msg_type is TwistStamped:
            msg.twist.linear.x = linear
            msg.twist.angular.z = angular 
        elif self.cmd_vel_msg_type is Twist:
            msg.linear.x = linear 
            msg.angular.z = angular 
        else:
            print("Unknown format of /cmd_vel message")
            raise SystemExit
        self.vel_pub.publish(msg)
        

    def move_forward(self):
        msg_cmd_vel.linear.x = 1.0
        msg_cmd_vel.angular.z = 0.0  

    def move_back(self):

        msg_cmd_vel = self.cmd_vel_msg_type()

        
        self.vel_pub.publish(msg_cmd_vel)

    def turn_left(self):

        msg_cmd_vel = self.cmd_vel_msg_type()
        msg_cmd_vel.linear.x = 0.0
        msg_cmd_vel.angular.z = 0.2  

        angular_distance = 1.5708  # 90 degrees in radians
        rotation_time = angular_distance / 0.2

        start_time = rospy.get_time()
        while rospy.get_time() - start_time < rotation_time and not rospy.is_shutdown():
            self.vel_pub.publish(msg_cmd_vel)
            rospy.sleep(0.1)

        msg_cmd_vel.angular.z = 0.0
        
        self.vel_pub.publish(msg_cmd_vel)

    def turn_right(self):

        msg_cmd_vel = self.cmd_vel_msg_type()
        msg_cmd_vel.linear.x = 0.0
        msg_cmd_vel.angular.z = -0.4  # Adjust the angular velocity

        self.vel_pub.publish(msg_cmd_vel)

        start_time = rospy.Time.now()
        rotate_duration = rospy.Duration.from_sec(2.25)  # Adjust as needed

        rate = rospy.Rate(10)  # Control loop rate
        while not rospy.is_shutdown():
            if rospy.Time.now() - start_time >= rotate_duration:
                # Stop the rotation
                msg_cmd_vel.angular.z = 0.0
                self.vel_pub.publish(msg_cmd_vel)
                break

            self.vel_pub.publish(msg_cmd_vel)
            rate.sleep()
        
    def stop(self):
        
        msg_cmd_vel = self.cmd_vel_msg_type()
        msg_cmd_vel.linear.x = 0.0
        msg_cmd_vel.angular.z = 0.0
        
        self.vel_pub.publish(msg_cmd_vel)

    def shutdown_hook(self):
        # Stop moving
        self.pub_cmd_vel(0, 0)

    def main_function(self):
        rospy.on_shutdown(self.shutdown_hook)
        # main loop to control the miro to move 
        while not rospy.is_shutdown():
            # self.turn_right() 
            self.pub_cmd_vel()
            self.rate.sleep()  # 控制发布指令的频率

if __name__ == "__main__":
    miro_controller = RobotController()  # create miro object
    miro_controller.main_function()  # start the loop

