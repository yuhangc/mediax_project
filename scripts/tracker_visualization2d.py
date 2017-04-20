#!/usr/bin/env python

import rospy
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

from geometry_msgs.msg import Pose2D

inch_to_meter = 0.0254


class TrackerVisualization2D:
    def __init__(self, ax):
        # subscribers and publishers
        self.pose2d_sub = rospy.Subscriber("tracking/pose2d", Pose2D, self.pose_callback)

        # get parameters
        map_file_path = rospy.get_param("~map_file", "map.json")
        self.marker_size = rospy.get_param("~marker_size", 0.204)

        # plot handles
        self.marker_handles = {}
        self.cam_handle = None

        # records the pose
        self.pose2d = Pose2D()
        self.pose2d.y = 3.0

        # load and plot the map
        self.ax = ax
        with open(map_file_path, 'r') as f:
            map_object = json.load(f)
            self.load_and_plot_map(map_object)

    def load_and_plot_map(self, mmap):
        # setup a plot axis
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-3, 3)
        self.ax.set_ylim(0, 5)

        for i in range(mmap["num_markers"]):
            # load marker pose
            map_entry = "marker{0}".format(i)
            x = mmap[map_entry]["x"]
            y = mmap[map_entry]["y"]
            th = mmap[map_entry]["theta"]

            if mmap[map_entry]["units"][0] == "inch":
                x *= inch_to_meter
                y *= inch_to_meter
            if mmap[map_entry]["units"][1] == "deg":
                th *= np.pi / 180.0

            # calculate marker boarders for plotting
            x_border = np.array([x + 0.5 * self.marker_size * np.cos(th),
                                 x - 0.5 * self.marker_size * np.cos(th)])
            y_border = np.array([y + 0.5 * self.marker_size * np.sin(th),
                                 y - 0.5 * self.marker_size * np.sin(th)])

            # plotting and keep the handle
            h = self.ax.plot(x_border, y_border, 'r-', lw=2)
            self.marker_handles[mmap[map_entry]["id"]] = h

        # draw camera at (0, 0)
        cam_wedge = mpatches.Wedge((0, 3.0), .2, 75, 105,
                                   facecolor="blue", alpha=0.75)
        self.cam_handle = self.ax.add_patch(cam_wedge)

        # return the axis
        # return ax

    def pose_callback(self, pose_msg):
        self.pose2d = pose_msg

    def update_plot(self):
        self.cam_handle.update({"center": (self.pose2d.x, self.pose2d.y)})
        self.cam_handle.update({"theta1": self.pose2d.theta * 180 / np.pi + 90 - 18})
        self.cam_handle.update({"theta2": self.pose2d.theta * 180 / np.pi + 90 + 18})


if __name__ == "__main__":
    rospy.init_node("tracker_visualizer")

    fig, ax = plt.subplots()
    plt.ion()
    visualizer = TrackerVisualization2D(ax)
    plt.pause(0.01)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        visualizer.update_plot()
        plt.pause(0.01)
        rate.sleep()
