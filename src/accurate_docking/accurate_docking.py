#!/usr/bin/env python
import re

import rospy
import actionlib

from tf import TransformListener
from tf.transformations import euler_from_quaternion
from robotnik_navigation_msgs.msg import MoveAction, MoveActionGoal
from robotnik_navigation_msgs.msg import DockAction

from rcomponent.rcomponent import *


class AccurateDocking(RComponent):
    """
    A class used to make a simple docking process
    """

    def __init__(self):
        self.dock_action_name = "/rb1_base/pp_docker"
        self.move_action_name = "/rb1_base/move"

        self.robot_link = "rb1_base_base_footprint"
        self.pregoal_link = "laser_docking_pregoal_footprint"
        self.goal_link = "laser_docking_goal_footprint"

        RComponent.__init__(self)

    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)

        #self.command_name = rospy.get_param('~command_name', self.command_name)

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        self.tf = TransformListener()

        self.move_action_client = actionlib.SimpleActionClient(
            self.move_action_name, MoveAction)
        self.move_action_client.wait_for_server()

        self.dock_action_client = actionlib.SimpleActionClient(
            self.dock_action_name, DockAction)
        self.dock_action_client.wait_for_server()

        return 0

    def ready_state(self):
        """Actions performed in ready state"""

        # Docking
        dock_goal = MoveActionGoal()
        dock_goal.goal.dock_frame = self.pregoal_link
        dock_goal.goal.robot_dock_frame = self.robot_link
        #dock_goal.goal.dock_frame = self.goal_link
        #dock_goal.goal.dock_offset.x = -0.5
        self.dock_action_client.send_goal(dock_goal)
        self.dock_action_client.wait_for_result()

        if self.dock_action_client.get_result() == True:

            # Execute move
            if self.tf.frameExists(self.robot_link) and self.tf.frameExists(self.goal_link):

                # Get relative goal position
                t = self.tf.getLatestCommonTime(
                    self.robot_link, self.goal_link)
                (position, quaternion) = self.tf.lookupTransform(
                    self.robot_link, self.goal_link, t)
                (_, _, orientation) = euler_from_quaternion(quaternion)

                # Orientate robot
                move_goal = MoveActionGoal()
                move_goal.goal.goal.theta = orientation
                self.move_action_client.send_goal(move_goal)
                self.move_action_client.wait_for_result()

                if self.move_action_client.get_result() == True:

                    # Get relative goal position
                    t = self.tf.getLatestCommonTime(
                        self.robot_link, self.goal_link)
                    (position, quaternion) = self.tf.lookupTransform(
                        self.robot_link, self.goal_link, t)
                    (_, _, orientation) = euler_from_quaternion(quaternion)

                    # Move forward
                    move_goal = MoveActionGoal()
                    move_goal.goal.goal.x = position[0]
                    self.move_action_client.send_goal(move_goal)
                    self.move_action_client.wait_for_result()

            else:
                rospy.ERROR("Move failed. TF are not available")
        else:
            rospy.ERROR("Docking failed")

        switch_to_state(State.SHUTDOWN_STATE)

    def shutdown(self):

        return RComponent.shutdown(self)

    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return RComponent.switch_to_state(self, new_state)
