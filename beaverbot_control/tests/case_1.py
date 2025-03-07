#!/usr/bin/env python3
##
# @file test_demo_2.py
#
# @brief Provide test demo 2 for trailer tractor system.
#
# @section author_doxygen_example Author(s)
# - Created by Tran Viet Thanh on 2025/01/28.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# Standard library
import numpy as np
from collections import namedtuple

# External library
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class Case1(object):
    """! Test demo 2
    """
    def __init__(self):
        """! Constructor
        """
        super(Case1, self).__init__()

        rospy.init_node("beaverbot_control")

        self._cmd_vel_publisher = rospy.Publisher(
            "beaverbot/cmd_vel", Twist, queue_size=1)

        rospy.Subscriber("emergency_stop", Bool,
                self._emergency_stop_callback)
        
        self._cmd_vel_message = Twist()

        self._emergency_flag = False

        trajectory_file = (
            "/root/catkin_ws/src/beaverbot_control/trajectories/"
            "demo2.csv"
        )

        self._obstacle_confirmation = False

        self._nx, self._nu = 3, 2

        trajectory_type = "wheel"

        self._length_base = 0.53

        self._trajectory = self._generate_trajectory(
            trajectory_file, self._nx, self._nu, trajectory_type)

        self._running_mode = "normal"

        self._obstacle_timer = 10

    def _emergency_stop_callback(self, msg):
        """! Emergency stop callback
        """
        self._emergency_flag = msg.data

    def _avoid_obstacle(self):
        """! Avoid obstacle
        """
        rospy.loginfo("Avoding obstacle")
        sampling_time = 0.2

        for i in range(len(self._trajectory.t)):
            
            self._cmd_vel_message.linear.x = self._trajectory.u[0, i]

            self._cmd_vel_message.angular.z = self._trajectory.u[1, i]

            self._cmd_vel_publisher.publish(self._cmd_vel_message)

            rospy.sleep(sampling_time)

    def robot_control(self):
        """! Robot control
        """
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._emergency_flag:
            
                rospy.loginfo("Obstacle detected, stopping and waiting for 10 seconds")
            
                self._cmd_vel_message.linear.x = 0
            
                self._cmd_vel_message.angular.z = 0
            
                self._cmd_vel_publisher.publish(self._cmd_vel_message)
            
                rospy.sleep(self._obstacle_timer)

                if self._emergency_flag:
                    self._avoid_obstacle()

                else:
                    rospy.loginfo("Obstacle cleared, continuing straight")

                    self._cmd_vel_message.linear.x = 0.5  # Adjust speed as needed

                    self._cmd_vel_message.angular.z = 0

                    self._cmd_vel_publisher.publish(self._cmd_vel_message)
            else:
                rospy.loginfo("No obstacle detected, going straight")

                self._cmd_vel_message.linear.x = 0.5  # Adjust speed as needed

                self._cmd_vel_message.angular.z = 0

                self._cmd_vel_publisher.publish(self._cmd_vel_message)

            rate.sleep()
        
    # ==================================================================================================
    # PRIVATE METHODS
    # ==================================================================================================
    def _generate_trajectory(self, file_path, nx, nu, trajectory_type=False):
        """! Generate a simple trajectory.
        @param file_path<str>: The file path of the
        generated trajectory
        @param nx<int>: The number of states
        @param nu<int>: The number of inputs
        @param trajectory_type<bool>: The flag to indicate if the
        generated trajectory is a derivative
        @return None
        """
        trajectory = {}

        data = np.genfromtxt(file_path, delimiter=",")

        initial_index = 0

        if np.isnan(np.nan):
            initial_index = 1

        trajectory["x"] = np.array(data[initial_index:, 1: 1 + nx])

        if len(data) > 1 + nx:
            trajectory["u"] = self._retrieve_u(
                initial_index, data, nx, nu, trajectory_type)

        trajectory["t"] = np.array(data[initial_index:, 0])

        trajectory["sampling_time"] = trajectory["t"][1] - trajectory["t"][0]

        trajectory_instance = namedtuple("Trajectory", trajectory.keys())(
            *trajectory.values())

        # self._visualize_trajectory(trajectory_instance)

        return trajectory_instance

    def _retrieve_u(self, initial_index, data, nx, nu, trajectory_type):
        """! Retrieve the input at time t.
        @param t<float>: The time
        @return u<list>: The input
        """
        if trajectory_type == "normal":
            u = np.transpose(np.array(
                data[initial_index:, 1 + nx: 1 + nx + nu]))

        elif trajectory_type == "derivative":
            u = np.zeros((self._nu, len(data) - initial_index))

            u[0, :] = np.hypot(
                np.array(data[initial_index:, 1 + nx: 1 + nx + 1]),
                np.array(data[initial_index:, 1 + nx + 1: 1 + nx + 2]),
            ).reshape(-1)

            u[1, :] = np.array(
                data[initial_index:, 1 + nx + 2: 1 + nx + 3]).reshape(-1)

        elif trajectory_type == "wheel":
            u = np.zeros((self._nu, len(data) - initial_index))

            u[0, :] = (
                np.array(data[initial_index:, 1 + nx: 1 + nx + 1]) +
                np.array(data[initial_index:, 1 + nx + 1: 1 + nx + 2])
            ).reshape(-1) / 2

            u[1, :] = (
                np.array(data[initial_index:, 1 + nx: 1 + nx + 1]) -
                np.array(data[initial_index:, 1 + nx + 1: 1 + nx + 2])
            ).reshape(-1) / self._length_base

        return u


if __name__ == "__main__":
    test_demo_2 = Case1()

    test_demo_2.robot_control()
