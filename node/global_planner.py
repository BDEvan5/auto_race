#! /usr/bin/env python

import rospy 
from nav_msgs.msg import OccupancyGrid, MapMetaData

import numpy as np
from matplotlib import pyplot as plt
from scipy import ndimage


class GlobalPlanner:
    def __init__(self):
        self.map_data = None
        self.map_metadata = None

        self.height = None
        self.width = None
        self.resolution = None
        self.origin = None

        self.cline = None
        self.dt = None

        map_topic = rospy.get_param("f1tenth_simulator/map_topic")

        map_sub = rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback)
        map_meta = rospy.Subscriber('map_metadata', MapMetaData, self.map_metadata_callback)

        rospy.spin()


    def map_callback(self, grid):
        rospy.loginfo("Map Callback called")
        self.map_data = np.array(grid.data, dtype=np.bool)
        self.map_data = np.array(self.map_data, dtype=np.int)
        self.map_data = np.ones_like(self.map_data) - self.map_data

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

        # plt.figure(1)
        # plt.imshow(self.map_data)
        # plt.show()

        dt = ndimage.distance_transform_edt(self.map_data)
        self.dt = dt

        plt.figure(1)
        plt.clf()
        plt.imshow(self.dt)
        plt.show()

        self.find_center_line(True)


        

    def find_center_line(self, show):
        dt = ndimage.distance_transform_edt(self.map_data)
        self.dt = dt

        # plt.figure(1)
        # plt.clf()
        # plt.imshow(self.dt)

        d_search = 1
        n_search = 11
        dth = (np.pi * 0.7) / (n_search-1)

        # makes a list of search locations
        search_list = []
        for i in range(n_search):
            th = -np.pi/2 + dth * i
            x = -np.sin(th) * d_search
            y = np.cos(th) * d_search
            loc = [x, y]
            search_list.append(loc)

        rospy.loginfo("Search list: " + str(search_list))

        pt = [0, 0] # start at the origin
        self.cline = [pt]
        th = np.pi/2 # start theta ????
        while (get_distance(pt, [0, 0]) > d_search or len(self.cline)) < 10 and len(self.cline) < 100: # makes sure it goes round
            vals = []
            self.search_space = []
            for i in range(n_search):
                d_loc = transform_coords(search_list[i], -th)
                search_loc = add_locations(pt, d_loc)

                self.search_space.append(search_loc)

                x, y = self.rc_to_inds(search_loc)
                val = dt[y, x]
                vals.append(val)

            # rospy.loginfo("Search space: " + str(self.search_space))
            # rospy.loginfo("Search Values: " + str(vals))

            ind = np.argmax(vals)
            d_loc = transform_coords(search_list[ind], -th)
            pt = add_locations(pt, d_loc)
            while len(self.cline) > 3 and (get_distance(pt, self.cline[-2]) < d_search \
                                    or    get_distance(pt, self.cline[-3]) < d_search):
                vals[ind] = 0
                ind = np.argmax(vals)
                d_loc = transform_coords(search_list[ind], -th)
                pt = add_locations(pt, d_loc)

            self.cline.append(pt)
            rospy.loginfo("Adding pt: " + str(pt))

            if show:
                self.plot_raceline_finding()

            th = get_bearing(self.cline[-2], pt)

        self.cline = np.array(self.cline)
        self.N = len(self.cline)
        rospy.loginfo("Raceline found")
        self.plot_raceline_finding(True)

    def rc_to_inds(self, rc_pos):
        x = int((rc_pos[0] - self.origin.position.x) / self.resolution)
        y = int((rc_pos[1] - self.origin.position.y) / self.resolution)

        return x, y

    def plot_raceline_finding(self, wait=False):
        plt.figure(1)
        plt.clf()
        plt.imshow(self.dt, origin='lower')

        for pt in self.cline:
            s_x, s_y = self.rc_to_inds(pt)
            plt.plot(s_x, s_y, '+', markersize=25)
            # plt.plot(s_y, s_x, '+', markersize=16)

        for pt in self.search_space:
            s_x, s_y = self.rc_to_inds(pt)
            plt.plot(s_x, s_y, 'x', markersize=20)


        plt.pause(0.001)
        # plt.show()


def transform_coords(x=[0, 0], theta=np.pi):
    # i want this function to transform coords from one coord system to another
    new_x = x[0] * np.cos(theta) - x[1] * np.sin(theta)
    new_y = x[0] * np.sin(theta) + x[1] * np.cos(theta)

    return np.array([new_x, new_y])

def add_locations(x1=[0, 0], x2=[0, 0], dx=1):
    # dx is a scaling factor
    ret = [0.0, 0.0]
    for i in range(2):
        ret[i] = x1[i] + x2[i] * dx
    return ret


def get_distance(x1=[0, 0], x2=[0, 0]):
    d = [0.0, 0.0]
    for i in range(2):
        d[i] = x1[i] - x2[i]
    return np.linalg.norm(d)


def get_bearing(x0, x1):
    dy = x1[1] - x0[1]
    dx = x1[0] - x0[0]

    bearing = np.arctan2(dy, dx)

    return bearing




if __name__ == "__main__":
    try:
        rospy.init_node("global_planner", anonymous=True)

        rospy.loginfo("Node Global planner has been initialised")

        gp = GlobalPlanner()

    except rospy.ROSInterruptException:
        pass

