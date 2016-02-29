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
Display all views
"""
def displayViews():
    plt.show()

"""
Create a costmap view
"""
def viewCostmap(costmap, title, num):
    fig = plt.figure(num, figsize=(16,6))

    ax = fig.add_subplot(1,1,1)
    ax.set_title(title)
    ax.set_aspect('equal')
    plt.imshow(costmap)

    cax = fig.add_axes([0.1, 0.1, 0.96, 0.8])
    cax.get_xaxis().set_visible(False)
    cax.get_yaxis().set_visible(False)
    cax.patch.set_alpha(0)
    cax.set_frame_on(False)
    plt.colorbar(orientation='vertical',drawedges=False)

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
Construct an entropy map that is the
same size as the costmap
"""
def constructEntropymap(costmap_msg):
    hdr  = costmap_msg.header
    info = costmap_msg.info
    data = costmap_msg.data

    width  = info.width
    height = info.height
    
    return np.zeros((height,width),dtype=np.int)

"""
Calculate the difference between two costmaps and
return a costmap with the diff
"""
def costmapDiff(costmap1, costmap2):
    return np.subtract(costmap2, costmap1)

"""
Takes a costmap and a update message and applies the patch
as well as calculates the entropy
"""
def applyPatch(entropymap, costmap, updateMsg):
    updateData   = updateMsg.data
    updateWidth  = updateMsg.width
    updateHeight = updateMsg.height
    updateX      = updateMsg.x
    updateY      = updateMsg.y

    costmapHeight, costmapWidth = costmap.shape

    # skip map resets
    if updateWidth == costmapWidth and updateHeight == costmapHeight:
        sys.stdout.write(".")
        sys.stdout.flush()
        return

    # apply the patch
    for i in xrange(len(updateData)):
        # indexes
        xindex = i/updateWidth + updateY
        yindex = i%updateWidth + updateX

        # calculate entropy
        old = costmap[i/updateWidth+updateY, i%updateWidth+updateX] 
        new = updateData[i]
        diff = 1 if (old != new) else 0
        entropymap[xindex, yindex] += diff

        # apply patch
        costmap[xindex, yindex] = updateData[i]

def get_costmaps():
    # Init ros node
    rospy.init_node('bag_viewer')

    # Get a pretty printer
    pp = pprint.PrettyPrinter()

    # Get the bag to view
    whichBag = rospy.get_param("~bag", None)

    # Ensure the bag was specified
    if whichBag == None:
        rospy.logerror("No bag specified")
        return 9
    rospy.loginfo("Bag to view: " + whichBag)

    # Load the bag
    bag = rosbag.Bag(whichBag)

    # Get bag information
    info = yaml.load(bag._get_yaml_info())
    pp.pprint(info)

    # Read the bag
    costmap = None
    entropymap = None
    originalCostmap = None
    for topic, msg, time in bag.read_messages():
        if topic == "/move_base/global_costmap/costmap":
            costmap = constructCostmap(msg)
            originalCostmap = np.copy(costmap)
            entropymap = constructEntropymap(msg)
        else:
            if costmap is None:
                rospy.logerror("Found a costmap update without first finding a costmap")
                return -1
            else:
                applyPatch(entropymap, costmap, msg)
    print("Done")

    # Create views for the original and patched costmaps
    viewCostmap(originalCostmap, "Original Costmap", 1)
    viewCostmap(costmap, "Patched Costmap", 2)

    # Calculate Diff and create a view for it
    diffmap = costmapDiff(originalCostmap, costmap)
    viewCostmap(diffmap, "Difference (red is added, blue is removed)", 3)

    # Create a view for the entropy map
    viewCostmap(entropymap, "Entropy (highe means more uncertainty)", 4)

    # Display the views
    displayViews()

    # Close the bag
    bag.close()

    # Exit
    return 0

if __name__ == '__main__':
    try:
        get_costmaps()
    except rospy.ROSInterruptException:
        pass
