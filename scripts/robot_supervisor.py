#!/usr/bin/env python

import rospy
import numpy as np

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry


class RobotSupervisor:
    def __init__(self):
        # get parameters
        self.state_ = rospy.get_param("~state_init", "Idle")
        self.state_set_ = None

        self.rand_move_vel_lin = rospy.get_param("~rand_move_vel_lin", 0.3)
        self.rand_move_vel_ang = rospy.get_param("~rand_move_vel_ang", 1.0)

        self.rand_move_params = {"rotate": {},
                                 "forward": {},
                                 "pause": {}}
        self.rand_move_params["rotate"]["t_min"] = rospy.get_param("~rand_move_rotate_time_min", 0.5)
        self.rand_move_params["rotate"]["t_max"] = rospy.get_param("~rand_move_rotate_time_max", 3.0)
        self.rand_move_params["forward"]["t_min"] = rospy.get_param("~rand_move_forward_time_min", 0.5)
        self.rand_move_params["forward"]["t_max"] = rospy.get_param("~rand_move_forward_time_max", 2.0)
        self.rand_move_params["pause"]["t_min"] = rospy.get_param("~rand_move_pause_time_min", 0.5)
        self.rand_move_params["pause"]["t_max"] = rospy.get_param("~rand_move_pause_time_max", 3.0)
        self.rand_move_t_total = rospy.get_param("~rand_move_time_total", 10.0)

        # rand move can have 3 modes
        # rotate - keep rotating for a random amount of time
        # forward - move forward for a random amount of time
        # pause - stop for a random amount of time
        self.rand_move_mode_ = "rotate"
        self.rand_move_t_mode_ = 0.5
        self.rand_move_t_start_ = 0.0
        self.rand_move_t_start_mode_ = 0.0

        self.pose_ = Pose2D()
        self.cmd_vel_ = Twist()
        self.cmd_vel_teleop_ = Twist()

        # subscribers and publishers
        self.robot_pose_sub_ = rospy.Subscriber("/robot_pose2d", Pose2D, self.robot_pose_callback)
        self.state_control_sub_ = rospy.Subscriber("/set_robot_state", String, self.state_control_callback)
        self.teleop_sub_ = rospy.Subscriber("/cmd_vel_teleop", Twist, self.teleop_callback)

        self.sys_msg_pub_ = rospy.Publisher("/sys_message", String, queue_size=1)
        self.cmd_vel_pub_ = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.state_pub_ = rospy.Publisher("/robot_state", String, queue_size=1)

    def robot_pose_callback(self, pose_msg):
        self.pose_ = pose_msg

    def state_control_callback(self, state_msg):
        self.state_set_ = state_msg.data

    def teleop_callback(self, vel_msg):
        self.cmd_vel_teleop_ = vel_msg

    def send_cmd_vel(self, v, om):
        self.cmd_vel_.linear.x = v
        self.cmd_vel_.angular.z = om
        self.cmd_vel_pub_.publish(self.cmd_vel_)

    def check_set_state(self):
        if self.state_set_ is None or self.state_set_ == self.state_:
            return

        if self.state_set_ == "Idle":
            self.send_cmd_vel(0, 0)
            self.state_ = "Idle"
        elif self.state_set_ == "RandMove":
            self.send_cmd_vel(0, 0)
            self.set_rand_move(flag_reset=True)
            self.state_ = "RandMove"
        elif self.state_set_ == "Teleop":
            self.send_cmd_vel(0, 0)
            self.state_ = "Teleop"

        self.state_set_ = None

    def idle(self):
        # check for state commands
        self.check_set_state()

    def set_rand_move(self, flag_reset=False):
        rand_move_mode = np.random.randint(0, 3)
        if rand_move_mode == 0:
            self.rand_move_mode_ = "rotate"
        elif rand_move_mode == 1:
            self.rand_move_mode_ = "forward"
        elif rand_move_mode == 2:
            self.rand_move_mode_ = "pause"

        param = self.rand_move_params[self.rand_move_mode_]
        self.rand_move_t_mode_ = np.random.rand() * (param["t_max"] - param["t_min"]) + param["t_min"]

        # optionally reset the start time
        self.rand_move_t_start_mode_ = rospy.get_time()
        if flag_reset:
            self.rand_move_t_start_ = rospy.get_time()

    def rand_move(self):
        # check for state commands
        self.check_set_state()

        # check for state timer
        t_state = rospy.get_time() - self.rand_move_t_start_
        if t_state > self.rand_move_t_total:
            self.send_cmd_vel(0, 0)
            self.state_ = "Idle"

        # check for mode timer
        t_mode = rospy.get_time() - self.rand_move_t_start_mode_
        if t_mode >= self.rand_move_t_mode_:
            self.set_rand_move()

        # send velocity commands based on mode
        if self.rand_move_mode_ == "rotate":
            self.send_cmd_vel(0, self.rand_move_vel_ang)
        elif self.rand_move_mode_ == "forward":
            self.send_cmd_vel(self.rand_move_vel_lin, 0)
        elif self.rand_move_mode_ == "pause":
            self.send_cmd_vel(0, 0)

    def teleop(self):
        # check for state commands
        self.check_set_state()

        # send velocity commands
        self.send_cmd_vel(self.cmd_vel_teleop_.linear.x, self.cmd_vel_teleop_.angular.z)

    def update(self):
        # state machine
        if self.state_ == "Idle":
            self.idle()
        elif self.state_ == "RandMove":
            self.rand_move()
        elif self.state_ == "Teleop":
            self.teleop()

        # publish the state
        state_msg = String()
        state_msg.data = self.state_
        self.state_pub_.publish(state_msg)


if __name__ == "__main__":
    # initialize
    rospy.init_node("robot_supervisor")

    # create supervisor object
    supervisor = RobotSupervisor()

    # loop
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        supervisor.update()
        rate.sleep()
