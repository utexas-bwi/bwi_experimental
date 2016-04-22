#!/usr/bin/env python
"""
Author: Pato Lankenau (plankenau@gmail.com)
License: BSD
"""
from __future__ import print_function
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from scipy.misc import imread
import matplotlib.pyplot as plt
import scipy.ndimage as ndi
import matplotlib as mpl
import numpy as np
import pprint
import rosbag
import rospy
import yaml
import copy
import sys

secondFloor = False

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

    if secondFloor:
        img = imread("2b.png")
        plt.imshow(img, cmap="gray", zorder=0, aspect="equal")
    else:
        img = imread("3ne.png")
        plt.imshow(img, cmap="gray", zorder=0)


    cmap = copy.copy(plt.cm.get_cmap("brg"))
    cmap.set_bad(alpha=0)
    cmap.set_under(alpha=0)

    ax = fig.add_subplot(1,1,1)
    ax.set_title(title)
    ax.set_aspect('equal')

    if secondFloor:
        c = np.copy(costmap)
        c = np.roll(costmap,10,axis=1)
        c = np.roll(costmap,200,axis=0)
        plt.imshow(c, interpolation='none', vmin=0.0000001, cmap=cmap, zorder=1, alpha=0.9)
    else:
        plt.imshow(costmap, interpolation='none', vmin=0.0000001, cmap=cmap, zorder=1, alpha=0.9)

    cax = fig.add_axes([0.1, 0.1, 0.96, 0.8])
    cax.get_xaxis().set_visible(False)
    cax.get_yaxis().set_visible(False)
    cax.patch.set_alpha(0)
    cax.set_frame_on(False)
    plt.colorbar(orientation='vertical',drawedges=False)

"""
Deflate a map using morphological erosion
"""
def deflatemap_sq(costmap, r):
    structure = np.ones((r,r))
    dmap = ndi.binary_erosion(costmap, structure=structure)
    return dmap
def deflatemap_circ(costmap, r):
    structure = circular_structure(r)
    dmap = ndi.binary_erosion(costmap, structure=structure)
    return dmap

"""
Return a circular structure of radius
"""
def circular_structure(radius):
    size = radius*2+1
    i,j = np.mgrid[0:size, 0:size]
    i -= (size/2)
    j -= (size/2)
    return np.sqrt(i**2+j**2) <= radius


"""
Get the edges of an  inflated costmap
"""
def edgemap(costmap):
    height, width = costmap.shape
    edgemap = np.zeros((height, width))
    wallmap = np.zeros((height, width))

    # generate the edgemap
    for x in xrange(height):
        for y in xrange(width):
            # if the current pixel is on
            if costmap[x,y] > 0:
                u = costmap[x,y+1]
                d = costmap[x,y-1]
                l = costmap[x-1,y]
                r = costmap[x+1,y]
                tl = costmap[x-1,y+1]
                tr = costmap[x+1,y+1]
                bl = costmap[x-1,y-1]
                br = costmap[x+1,y-1]
                # and one of the neighboring pixels is off
                if u == 0 or d == 0 or l == 0 or r == 0 or tl == 0 or tr == 0 or bl == 0 or br == 0:
                    # turn on the pixel
                    edgemap[x,y] = 1

    # generate the wallmap
    # iterate through all the edges, and if they are manhattan distance =
    # radius of robot away from an on pixel in the original costmap (and every
    # point in the path to that pixel is on) then turn on the center in the
    # wallmap
#    for x in xrange(height):
#        for y in xrange(width):
#            if edgemap[x,y] == 1:

    return edgemap

"""
Return a thresholded binary version of the map
"""
def thresholdmap(costmap, threshold):

    height, width = costmap.shape
    binary = np.zeros((height, width), dtype=np.float64)
    for x in xrange(height):
        for y in xrange(width):
            if costmap[x,y] > threshold:
                binary[x,y] = 1
    return binary 
    
"""
Return a static map
"""
def staticMap(costmap, entropymap):
    return np.subtract(costmap, entropymap)

"""
Return average map
"""
def averageMap(averagemap, updatemap):
    #return np.divide(averagemap, updatemap)

    height, width = averagemap.shape
    corrected = np.zeros((height, width), dtype=np.float64)
    for x in xrange(height):
        for y in xrange(width):
            e = float(averagemap[x,y])
            u = float(updatemap[x,y])
            corrected[x,y] = 0 if u == 0 else e / u
    return corrected

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
Normalize an entropy map based on where the most updates happened
"""
def correctEntropy(entropymap, updatemap):
    height, width = entropymap.shape
    corrected = np.zeros((height, width), dtype=np.float64)
    #np.divide(entropymap, updatemap, corrected)
    for x in xrange(height):
        for y in xrange(width):
            e = float(entropymap[x,y])
            u = float(updatemap[x,y])
            corrected[x,y] = 0 if u == 0 else e / u
    return corrected


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
def applyPatch(entropymap, updatemap, costmap, averagemap, updateMsg):
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

        # add to updatemap
        updatemap[xindex, yindex] += 1

        # add to averageMap
        averagemap[xindex, yindex] += 1 if (new > 50) else 0

        # apply patch
        costmap[xindex, yindex] = updateData[i]

def deflate():
    # Init ros node
    rospy.init_node('bag_viewer')

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

    # Read the bag
    costmap    = None
    originalCostmap = None
    for topic, msg, time in bag.read_messages():
        if topic == "/move_base/global_costmap/costmap":
            costmap = constructCostmap(msg)
            originalCostmap = np.copy(costmap)
    print("Done")

    viewCostmap(originalCostmap, "Original Costmap", 1)

    binarymap = thresholdmap(originalCostmap, 90)
#    viewCostmap(binarymap, "Binary Thresholded Costmap", 2)

#    emap = edgemap(binarymap)
#    viewCostmap(emap, "Edges of Binary Costmap", 3)

    dmap1 = deflatemap_sq(binarymap, 12)
    viewCostmap(dmap1, "Deflated (Square) Map", 4)

    dmap2 = deflatemap_circ(binarymap, 7)
    viewCostmap(dmap2, "Deflated (Circle) Map", 5)



    # Display the views
    displayViews()

    return


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
    costmap    = None
    entropymap = None
    updatemap  = None
    originalCostmap = None
    for topic, msg, time in bag.read_messages():
        if topic == "/move_base/global_costmap/costmap":
            costmap = constructCostmap(msg)
            originalCostmap = np.copy(costmap)
            entropymap = constructEntropymap(msg)
            updatemap  = constructEntropymap(msg)
            averagemap = constructEntropymap(msg)
        else:
            if costmap is None:
                rospy.logerror("Found a costmap update without first finding a costmap")
                return -1
            else:
                applyPatch(entropymap, updatemap, costmap, averagemap, msg)
    print("Done")


    # Create views for the original and patched costmaps
    #viewCostmap(originalCostmap, "Original Costmap", 1)
    viewCostmap(costmap, "Patched Costmap", 2)


    # Calculate Diff and create a view for it
    diffmap = costmapDiff(originalCostmap, costmap)
    #viewCostmap(diffmap, "Difference (red is added, blue is removed)", 3)

    # Create a view for the entropy map
    #viewCostmap(entropymap, "Entropy (higher means more uncertainty)", 4)

    # Create a view for the update map
    viewCostmap(updatemap, "Updatemap (which areas had the most updates)", 5)

    # Create a corrected entropy map
    correctedEntropy = correctEntropy(entropymap, updatemap)
    viewCostmap(correctedEntropy, "Corrected entropy map", 6)


    # Create average map
    averageM = averageMap(averagemap, updatemap)
    viewCostmap(averagemap, "Average costmap", 8)
    viewCostmap(averageM, "Average costmap 2", 9)

    # Create negative map
    staticM = staticMap(averageM, entropymap)
    viewCostmap(staticM, "Map of static objects", 10)

    # Display the views
    displayViews()

    # Close the bag
    bag.close()

    # Exit
    return 0

if __name__ == '__main__':
    try:
        deflate()
        #get_costmaps()
    except rospy.ROSInterruptException:
        pass
