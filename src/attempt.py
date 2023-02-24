#!/usr/bin/env python3


"""
##CHANGES

1. Added Namespace
2. Introduce euler angles from quaternions
3. fine tune the turn so that it is always 0, 90, 180, 270 (within acceptable tolerance)

"""


import rospy

from geometry_msgs.msg import Twist
from math import pi
# import the Odometry message for subscribing to the odom topic:
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class OutAndBack():
    maze = [
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 0, 1, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 1, 0, 1, 0, 0, 1, 0, 1],
        [1, 0, 1, 0, 1, 0, 0, 1, 0, 1],
        [1, 0, 0, 0, 1, 0, 0, 1, 0, 1],
        [1, 0, 0, 0, 1, 0, 0, 1, 0, 1],
        [1, 0, 1, 0, 1, 0, 0, 1, 0, 1],
        [1, 0, 1, 0, 1, 0, 0, 1, 0, 1],
        [1, 0, 1, 0, 0, 0, 0, 1, 0, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    ]
    dirs = [
        lambda x, y: (x, y + 1),  # right
        lambda x, y: (x + 1, y),  # down
        lambda x, y: (x, y - 1),  # left
        lambda x, y: (x - 1, y),  # up
    ]
    stack = []

    def callback_function(self, odom_data):
        # obtain the orientation co-ords:
        or_x = odom_data.pose.pose.orientation.x
        or_y = odom_data.pose.pose.orientation.y
        or_z = odom_data.pose.pose.orientation.z
        or_w = odom_data.pose.pose.orientation.w

        # obtain the position co-ords:
        self.pos_x = odom_data.pose.pose.position.x
        self.pos_y = odom_data.pose.pose.position.y

        (roll, pitch, yaw) = euler_from_quaternion([or_x, or_y, or_z, or_w],
                                                   'sxyz')
        self.yaw = yaw

    def __init__(self):

        self.ns1 = 'ghost'  # Ghost 1 Namespace

        rospy.init_node('out_and_back', anonymous=False)

        rospy.on_shutdown(self.shutdown)

        sub = rospy.Subscriber(
            self.ns1 + '/odom', Odometry, self.callback_function)
        cmd_vel = rospy.Publisher(self.ns1 + '/cmd_vel', Twist, queue_size=1)
        rate = 50

        r = rospy.Rate(rate)

        linear_speed = 0.2

        goal_distance = 1.0

        linear_duration = goal_distance / linear_speed

        angular_speed = 1.0

        goal_angle = pi

        angular_duration = goal_angle / angular_speed

        x1 = 1
        y1 = 1
        x2 = 8
        y2 = 8
        self.stack.append((x1, y1))
        self.maze[x1][y1] = 2  # the way already went
        while len(self.stack) > 0:
            cur_node = self.stack[-1]
            if cur_node == (x2, y2):
                print(self.stack)
                self.route(self.stack)
            for d in self.dirs:
                next_x, next_y = d(*cur_node)
                if self.maze[next_x][next_y] == 0:
                    self.stack.append((next_x, next_y))
                    self.maze[next_x][next_y] = 2
                    break
            else:
                self.stack.pop()
        print('no way')

        self.cmd_vel.publish(Twist())

    def route(self, stack):

        up = 0
        left = 1
        down = 2
        right = 3

        i = 0
        loc = [1, 1]
        fin = [8, 8]
        direction = right
        while loc != fin:
            i = i+1
            nextLoc = stack.pop(1)
            if loc[0]+1 == nextLoc[0]:
                aimD = right
                print("right")

                self.turn(direction, aimD)
                direction = aimD

                loc[0] = loc[0]+1

            elif loc[0]-1 == nextLoc[0]:
                aimD = left
                print(("left"))  # left

                self.turn(direction, aimD)
                direction = aimD
                loc[0] = loc[0] - 1
            elif loc[1] + 1 == nextLoc[1]:
                aimD = up
                print("up")

                self.turn(direction, aimD)
                direction = aimD
                loc[1] = loc[1] + 1
            elif loc[1] - 1 == nextLoc[1]:
                aimD = down
                print("down")

                self.turn(direction, aimD)
                direction = aimD
                loc[1] = loc[1] - 1
            print(loc)

    def turn(self, direction, aimD):

        sub = rospy.Subscriber(
            self.ns1 + 'odom', Odometry, self.callback_function)
        cmd_vel = rospy.Publisher(self.ns1 + '/cmd_vel', Twist, queue_size=1)
        rate = 50

        r = rospy.Rate(rate)

        linear_speed = 0.2

        goal_distance = 1.0

        linear_duration = goal_distance / linear_speed

        angular_speed = 1.0

        a = 90*(aimD - direction)
        if (a < 0):
            a = a+360
        print(a)
        goal_angle = 2*pi*a/360

        move_cmd = Twist()
        move_cmd.angular.z = angular_speed
        angular_duration = goal_angle / angular_speed
        ticks = int(goal_angle * rate)
        for t in range(ticks):
            cmd_vel.publish(move_cmd)
            r.sleep()

        move_cmd = Twist()
        cmd_vel.publish(move_cmd)
        rospy.sleep(angular_duration+0.5)

        move_cmd = Twist()
        move_cmd.linear.x = linear_speed
        ticks = int(linear_duration * rate)
        for t in range(ticks):
            cmd_vel.publish(move_cmd)
            r.sleep()

        # introduce a check that fine tunes the turn
        if ((self.yaw*180.0/pi) - goal_angle) < -2:  # degrees
            # rotate counter-clockwise
            pass
        elif ((self.yaw*180.0/pi) - goal_angle) > 2:
            # rotate clockwise
            pass

        move_cmd = Twist()
        cmd_vel.publish(move_cmd)
        rospy.sleep(1)

    def shutdown(self):
        cmd_vel = rospy.Publisher(self.ns1 + '/cmd_vel', Twist, queue_size=1)
        rospy.loginfo("Stopping the robot...")
        cmd_vel.publish(Twist())
        rospy.sleep(1)

    def print_stuff(self, a_message, x, z,):
        # pos_x = odom_data.pose.pose.position.x
        # pos_y = odom_data.pose.pose.position.y
        # a function to print information to the terminal (use as you wish):
        # print the message that has been passed in to the method via the "a_message" input:
        print(a_message)
        # you could use this to print the current velocity command:
        print("current velocity: lin.x = {:.1f}, ang.z = {:.1f}".format(x, z))
        print("current odometry: x = {:.3f}, y = {:.3f}".format(
            self.pos_x, self.pos_y))
        # print("current odometry: x = {:.3f}, y = {:.3f}, theta_z = {:.3f}".format(pos_x, pos_y, self.theta_z))


if __name__ == '__main__':

    OutAndBack()
