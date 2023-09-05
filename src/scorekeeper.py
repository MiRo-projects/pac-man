#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Twist

class LinkCollisionsHandler:

    GAZEBO_NS = "/gazebo_server"
    N_POINTS = 247

    def __init__(self):
        self.links = ["maze_level01::Score_Point" + str(i) for i in range(1, self.N_POINTS)]
        self.pacman_model_name = "pacman"
        self.model_states = None
        self.ground_height = -1.0

        rospy.init_node('link_collisions_handler_node', anonymous=True)
        rospy.Subscriber(self.GAZEBO_NS + "/link_states", LinkStates, self.link_states_callback)
        rospy.Subscriber(self.GAZEBO_NS + "/model_states", ModelStates, self.pacman_callback)

    def link_states_callback(self, data):
        for link_name in self.links:
            if link_name in data.name:
                link_index = data.name.index(link_name)
                link_position = data.pose[link_index].position

                # Check for collision with pacman
                if self.check_collision_with_pacman(link_position):
                    rospy.loginfo("Link {} collided with pacman. Moving it to the ground.".format(link_name))
                    self.move_link_to_ground(link_name,link_position)

    def pacman_callback(self, data):
        self.model_states = data

    def check_collision_with_pacman(self, link_position):
        if self.model_states is not None:
            pacman_index = self.get_model_index_by_name(self.pacman_model_name)
            if pacman_index is not None:
                pacman_position = self.model_states.pose[pacman_index].position
                return self.distance_between_points(link_position, pacman_position) < 0.3  
        return False

    def move_link_to_ground(self, link_name,link_position):
        try:
            link_state_pub = rospy.Publisher(self.GAZEBO_NS + "/set_link_state", LinkState, queue_size=1)

            link_state = LinkState()
            link_state.link_name = link_name
            link_state.pose.position.x = link_position.x
            link_state.pose.position.y = link_position.y
            link_state.pose.position.z = self.ground_height
            link_state.twist = Twist()

            link_state_pub.publish(link_state)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

    def get_model_index_by_name(self, model_name):
        if self.model_states is not None:
            try:
                return self.model_states.name.index(model_name)
            except ValueError:
                rospy.logwarn("{} not found in the received data. Model Names: {}".format(model_name, self.model_states.name))
        return None

    def distance_between_points(self, p1, p2):
        return ((p1.x - p2.x)**2 + (p1.y - p2.y)**2)**0.5

    def run(self):
        rate = rospy.Rate(5)  # 5 Hz, equivalent to every 0.2 seconds
        while not rospy.is_shutdown():
            rate.sleep()
            self.link_states_callback(rospy.wait_for_message(self.GAZEBO_NS + "/link_states", LinkStates))
            self.pacman_callback(rospy.wait_for_message(self.GAZEBO_NS + "/model_states", ModelStates))

if __name__ == '__main__':
    handler = LinkCollisionsHandler()
    handler.run()
