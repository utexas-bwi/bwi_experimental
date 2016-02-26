#!/usr/bin/env python
"""
Author: Pato Lankenau (plankenau@gmail.com)
License: BSD
"""
from __future__ import print_function
from nav_msgs.msg import OccupancyGrid
import rospy
import sys
import rosbag
import yaml
import pprint
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

def view(costmap):
    hdr  = costmap.header
    info = costmap.info
    data = costmap.data

    width  = info.width
    height = info.height

    print(width)
    print(height)
    print(len(data))
    
    # TODO: will need to reconstruct the matrix from the 1D array
    matrix = np.array(data).reshape((height,width))

    fig = plt.figure(figsize=(16,6))

    ax = fig.add_subplot(1,1,1)
    ax.set_title("Costmap Heatmap")
    ax.set_aspect('equal')
    plt.imshow(matrix)

    cax = fig.add_axes([0.1, 0.1, 0.96, 0.8])
    cax.get_xaxis().set_visible(False)
    cax.get_yaxis().set_visible(False)
    cax.patch.set_alpha(0)
    cax.set_frame_on(False)
    plt.colorbar(orientation='vertical',drawedges=False)

    plt.show()

def get_costmaps():
    # Init ros node
    rospy.init_node('bag_viewer')

    # Get a pretty printer
    pp = pprint.PrettyPrinter()

    # Get the bag to view
    whichBag = rospy.get_param("~bag", None)

    # Ensure the bag was specified
    if whichBag == None:
        print("No bag specified", file=sys.stderr)
        return 9
    rospy.loginfo("Bag to view: " + whichBag)

    # Load the bag
    bag = rosbag.Bag(whichBag)

    # Get bag information
    info = yaml.load(bag._get_yaml_info())
    pp.pprint(info)

    # Read the bag
    for topic, msg, time in bag.read_messages():
      if topic == "/move_base/global_costmap/costmap":
        view(msg)

    # Close the bag
    bag.close()

    return 0

if __name__ == '__main__':
    try:
        get_costmaps()
    except rospy.ROSInterruptException:
        pass
