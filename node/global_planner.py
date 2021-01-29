#! /usr/bin/env python

import rospy 
from nav_msgs.msg import OccupancyGrid, MapMetaData

import numpy as np
from matplotlib import pyplot as plt


class GlobalPlanner:
    def __init__(self):
        self.map_data = None
        self.map_metadata = None

        self.height = None
        self.width = None
        self.resolution = None
        self.origin = None

        map_topic = rospy.get_param("f1tenth_simulator/map_topic")

        map_sub = rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback)
        map_meta = rospy.Subscriber('map_metadata', MapMetaData, self.map_metadata_callback)

        rospy.spin()


    def map_callback(self, grid):
        rospy.loginfo("Map Callback called")
        self.map_data = np.array(grid.data)

        if self.map_metadata is not None:
            self.process_map()
        

    def map_metadata_callback(self, data):
        rospy.loginfo("map_metadata callback called")
        self.map_metadata = data 

        self.height = data.height
        self.width = data.width
        self.resolution = data.resolution
        self.origin = data.origin

        if self.map_data is not None:
            self.process_map()

    def process_map(self):
        
        self.map_data = np.reshape(self.map_data, (self.height, self.width))
        rospy.loginfo("size: " + str(self.map_data.shape))

        plt.figure(1)
        plt.imshow(self.map_data)
        plt.show()








if __name__ == "__main__":
    try:
        rospy.init_node("global_planner", anonymous=True)

        rospy.loginfo("Node Global planner has been initialised")

        gp = GlobalPlanner()

    except rospy.ROSInterruptException:
        pass

