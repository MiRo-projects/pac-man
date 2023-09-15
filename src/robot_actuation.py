#!/usr/bin/env python3

import argparse
from dataclasses import dataclass

import rospy
import rostopic
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, TwistStamped, Pose2D
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt32
import miro2 as miro


# parse args
cli = argparse.ArgumentParser(
    prog='AI_Pac-man',
    description='Script to control the Pac-man and Ghosts autonomously',
    epilog='Good luck!'
)
pose_control = cli.add_mutually_exclusive_group()
pose_control.add_argument('--sim_pose', dest='pose_ctrl', action='store_true', default=True)
pose_control.add_argument('--odom_pose', dest='pose_ctrl', action='store_false', default=False)
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
    yaw: float = 0.0

class RobotController:
    def __init__(self):

        # Process args
        self.args = cli.parse_args(rospy.myargv()[1:])

        self.ns = str(self.args.ns)
        self.ns = self.ns[1:] if self.ns[0] == '/' else self.ns # for consistency
        self.gz_ns = str(self.args.gz)
        self.gz_ns = self.gz_ns[1:] if self.gz_ns[0] == '/' else self.gz_ns # same

        # Create node
        rospy.init_node(self.ns + "_controller", anonymous=False)
        rospy.on_shutdown(self.shutdown_hook)

        # Find relevant topics within the provided ns
        pub_list = rostopic.get_topic_list()[0]  # Look through Publishers
        self.odom_topic = [t[0] for t in pub_list
                        if ('/' + self.ns in t[0] and 'odom' in t[0])]
        if not self.odom_topic:
            print(f"Could not find the required /odom topic for the namespace {self.ns}")
            raise SystemExit
        self.odom_topic = self.odom_topic[0]

        sub_list = rostopic.get_topic_list()[1]  # Look through Subscribers
        self.cmd_vel_topic = [t[0] for t in sub_list
                              if ('/' + self.ns in t[0] and 'cmd_vel' in t[0])]
  
        if not self.cmd_vel_topic:
            print(f"Could not find the required /cmd_vel topic for the namespace {self.ns}")
            raise SystemExit
        self.cmd_vel_topic = self.cmd_vel_topic[0]

        # Create Publishers and Subscribers
        self.cmd_vel_msg_type, topic_str, _ = rostopic.get_topic_class(self.cmd_vel_topic)
        self.vel_pub = rospy.Publisher(self.cmd_vel_topic, 
                                       self.cmd_vel_msg_type,
                                       queue_size=1,
                                       latch=True
                                       )
        self.pos_sub = rospy.Subscriber(self.odom_topic,
                                        Odometry,
                                        self.pose_cb,
                                        queue_size=1
                                        )
        if self.args.pose_ctrl:
            self.sim_pose = rospy.ServiceProxy('/'+self.gz_ns+'/get_model_state',
                                               GetModelState)
        # Create variables and params
        self.robot = Pose()
        self.rate = rospy.Rate(50)

        # MiRo bridge flags, if we're controlling Pac-man
        self.flag_topic = [t[0] for t in sub_list
                              if ('/' + self.ns in t[0] and 'flags' in t[0])]
        if self.flag_topic:
            self.flag_topic = self.flag_topic[0]
            self.flags = miro.constants.PLATFORM_D_FLAG_DISABLE_CLIFF_REFLEX
            self.flags |= miro.constants.PLATFORM_D_FLAG_PERSISTENT
            self.flags_pub = rospy.Publisher(self.flag_topic, 
                                             UInt32,
                                             queue_size=1,
                                             latch=True)

    def pose_cb(self, data):
        if self.args.pose_ctrl:
            ##TODO This shouldn't really be here    
            payload = self.sim_pose(self.ns, "")
            self.robot.x = payload.pose.position.x
            self.robot.y = payload.pose.position.y
            (_, _, self.robot.yaw) = euler_from_quaternion([
                payload.pose.orientation.x,
                payload.pose.orientation.y,
                payload.pose.orientation.z,
                payload.pose.orientation.w
                ],'sxyz')
        else:
            self.robot.x = data.pose.pose.position.x
            self.robot.y = data.pose.pose.position.y
            (_, _, self.robot.yaw) = euler_from_quaternion([
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w
                ],'sxyz')

    def pub_cmd_vel(self, linear = 0, angular = 0):
        if self.flag_topic:
            self.flags_pub.publish(UInt32(self.flags))
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
        self.pub_cmd_vel(1, 0)

    def move_back(self):
        self.pub_cmd_vel(-1, 0)

    def turn_left(self):
        self.pub_cmd_vel(0, -1)

    def turn_right(self):
        self.pub_cmd_vel(0, 1)

    def stop(self):
        self.pub_cmd_vel(0, 0)

    def shutdown_hook(self):
        # Stop moving
        rospy.sleep(0.3)
        self.pub_cmd_vel(0, 0)

    def main(self):
        while not rospy.is_shutdown():
            # self.turn_right()
            self.move_forward()
            rospy.sleep(1)

            self.move_back()
            rospy.sleep(1)

            self.turn_left()
            rospy.sleep(1)

            self.turn_right()
            rospy.sleep(1)

            self.rate.sleep()

if __name__ == "__main__":
    miro_controller = RobotController()  # instantiate robot
    miro_controller.main()  # start the loop

