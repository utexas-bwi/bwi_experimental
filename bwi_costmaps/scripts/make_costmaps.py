#!/usr/bin/env python
"""
Author: Pato Lankenau (plankenau@gmail.com)
License: BSD
"""
from __future__ import print_function
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
import rospy
import sys
import rosbag
import yaml
import pprint
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

"""
Display a costmap
"""
def viewCostmap(costmap1, costmap2):
    fig = plt.figure(1, figsize=(16,6))

    ax = fig.add_subplot(1,1,1)
    ax.set_title("Costmap Heatmap")
    ax.set_aspect('equal')
    plt.imshow(costmap1)

    cax = fig.add_axes([0.1, 0.1, 0.96, 0.8])
    cax.get_xaxis().set_visible(False)
    cax.get_yaxis().set_visible(False)
    cax.patch.set_alpha(0)
    cax.set_frame_on(False)
    plt.colorbar(orientation='vertical',drawedges=False)

    fig = plt.figure(2, figsize=(16,6))

    ax = fig.add_subplot(1,1,1)
    ax.set_title("Costmap Heatmap")
    ax.set_aspect('equal')
    plt.imshow(costmap2)

    cax = fig.add_axes([0.1, 0.1, 0.96, 0.8])
    cax.get_xaxis().set_visible(False)
    cax.get_yaxis().set_visible(False)
    cax.patch.set_alpha(0)
    cax.set_frame_on(False)
    plt.colorbar(orientation='vertical',drawedges=False)

    plt.show()

"""
Return a numpy matrix containing the costmap
"""
def constructCostmap(costmap_msg):
    hdr  = costmap_msg.header
    info = costmap_msg.info
    data = costmap_msg.data

    width  = info.width
    height = info.height
    
    return np.array(data).reshape((height,width))

"""
Takes a costmap and a update message and applies the patch
"""
def applyPatch(costmap, updateMsg):
    updateData   = updateMsg.data
    updateWidth  = updateMsg.width
    updateHeight = updateMsg.height
    updateX      = updateMsg.x
    updateY      = updateMsg.y

    costmapHeight, costmapWidth = costmap.shape

#    costmapFlat = costmap.reshape(-1)
#
#    data = np.array(updateData)
#    index = updateX + updateY * updateWidth
#    indeces = range(index, index + updateWidth * updateHeight)
#
#    np.put(costmapFlat, indeces, data)
#
#    costmap = costmapFlat.reshape((costmapHeight, costmapWidth))
    
    print("uW:%d\tuH:%d\tcmW:%d\tcmH:%d\n" % (updateWidth, updateHeight, costmapWidth, costmapHeight))

    for i in xrange(len(updateData)):
        #if i%updateWidth < costmapWidth and i/updateWidth < costmapHeight:
        # THIS IS WHERE THE ERROR IS
        if updateWidth == 1857 and i == 573:
            print(i)
        costmap[i%updateWidth, i/updateWidth] = 1#updateData[i]

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
    costmap = None
    originalCostmap = None
    for topic, msg, time in bag.read_messages():
        if topic == "/move_base/global_costmap/costmap":
            costmap = constructCostmap(msg)
            originalCostmap = np.copy(costmap)
        else:
            if costmap == None:
                rospy.logerror("Found a costmap update without first finding a costmap")
                return 8
            else:
                applyPatch(costmap, msg)

    # View original vs final costmap
    viewCostmap(originalCostmap, costmap)

    # Close the bag
    bag.close()

    return 0

if __name__ == '__main__':
    try:
        get_costmaps()
    except rospy.ROSInterruptException:
        pass
