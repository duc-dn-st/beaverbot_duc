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
import time
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
        
        self._parameter_register()

        self._pubisher_register()

        self._subscriber_register()

        self._trajectory_register()

    # ==================================================================================================
    # PRIVATE METHODS
    # ==================================================================================================
    def _parameter_register(self):
        """! Register the parameters
        """
        self._cmd_vel_message = Twist()

        self._sampling_time = 0.2

        self._stop_duration = 10

        self._avoiding_duration = 5

        self._lidar_check = False

        self._state = "moving"

        self._stop_timer = None

        self._avoiding_timer = None

        self._steps = 0

    def _pubisher_register(self):
        """! Register the publisher
        """
        self._cmd_vel_publisher = rospy.Publisher(
            "beaverbot/cmd_vel", Twist, queue_size=1)
        
    def _subscriber_register(self):
        """! Register the subscriber
        """
        rospy.Subscriber("emergency_stop", Bool,
                self._emergency_stop_callback)
        
    def _emergency_stop_callback(self, msg):
        """! Emergency stop callback
        """
        self._lidar_check = msg.data

    def _move_straight(self):
        """! Move straight
        """
        self._cmd_vel_message.linear.x = 0.5

        self._cmd_vel_message.angular.z = 0

        self._cmd_vel_publisher.publish(self._cmd_vel_message)

        rospy.sleep(self._sampling_time)
    
    def _stop(self):
        """! Stop
        """
        if self._stop_timer is None:
            self._stop_timer = time.time()

        self._cmd_vel_message.linear.x = 0

        self._cmd_vel_message.angular.z = 0

        self._cmd_vel_publisher.publish(self._cmd_vel_message)

        rospy.sleep(self._sampling_time)

    def _stop_decision(self):
        """! Make a decision to stop the robot
        """
        if self._stop_timer is None:  # Start timer only once
            self._stop_timer = time.time()

        elapsed_time = time.time() - self._stop_timer  # Compute elapsed time

        if elapsed_time < self._stop_duration:
            if not self._lidar_check:
                self._state = "moving"
            
                self._stop_timer = None
        else:
            self._state = "avoiding"

            self._stop_timer = None

    def _avoid_obstacle(self, steps):
        """! Start avoiding obstacles using the precomputed trajectory. 
        """
        rospy.loginfo(f"State: {self._state},  Lidar check: {self._lidar_check}")

        self._cmd_vel_message.linear.x = self._trajectory.u[0, steps]

        self._cmd_vel_message.angular.z = self._trajectory.u[1, steps]

        self._cmd_vel_publisher.publish(self._cmd_vel_message)

        rospy.sleep(self._sampling_time)

    def run(self):
        """! Robot control
        """
        while not rospy.is_shutdown():

            rospy.loginfo(f"State: {self._state},  Lidar check: {self._lidar_check}")

            if self._state == "moving":
                if self._lidar_check:
                    self._stop()

                    self._state = "stopping"

                else:
                    self._move_straight()
            
            elif self._state == "stopping":
                self._stop_decision()

                rospy.sleep(self._sampling_time)

            elif self._state == "avoiding":
                if self._avoiding_timer is None:
                    self._avoiding_timer = time.time()
                
                if time.time() - self._avoiding_timer < self._avoiding_duration:
                    self._avoid_obstacle(self._steps)

                    self._steps += 1

                else :
                    if self._lidar_check:
                        self._stop()

                        self._state = "stopping"

                        self._steps = 0

                        self._avoiding_timer = None
                    else:
                        self._avoid_obstacle(self._steps)

                        self._steps += 1

    # ==================================================================================================
    # EXTERNALS METHODS
    # ==================================================================================================

    def _trajectory_register(self):
        """! Register the trajectory
        """
        self._nx, self._nu = 3, 2

        trajectory_file = (
            "/root/catkin_ws/src/beaverbot_control/trajectories/"
            "demo2.csv")

        trajectory_type = "wheel"

        self._length_base = 0.53

        self._trajectory = self._generate_trajectory(
            trajectory_file, self._nx, self._nu, trajectory_type)

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

    test_demo_2.run()