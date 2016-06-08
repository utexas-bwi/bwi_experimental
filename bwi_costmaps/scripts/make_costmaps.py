#!/usr/bin/env python
"""
Author: Pato Lankenau (plankenau@gmail.com)
License: BSD
"""
from __future__ import print_function
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from scipy.misc import imread
from datetime import datetime
import matplotlib.pyplot as plt
import scipy.ndimage as ndi
import scipy.misc
import matplotlib as mpl
import numpy as np
import pprint
import rosbag
import rospy
import yaml
import copy
import sys
import os

secondFloor = False
saveFiles = True
resultsDir = os.path.expanduser('~') + "/costmap_results"
dirName = ""

"""
Display all views
"""
def displayViews():
    plt.show()

"""
Create a costmap view
"""
def viewCostmap(costmap, title, num):
    global dirName

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

    if saveFiles:
        fileName = title.replace(' ', '_') + ".png"
        plt.savefig(dirName + "/" + fileName)
        scipy.misc.imsave(dirName + "/raw_" + fileName, costmap)


"""
Deflate a map using morphological erosion
r - radius
i - iterations
"""
def deflatemap_sq(costmap, r, i):
    structure = np.ones((r,r))
    dmap = ndi.binary_erosion(costmap, structure=structure, iterations=i)
    return dmap

"""
Deflate a map using morphological erosion with a circle structure
r - radius
i - iterations
"""
def deflatemap_circ(costmap, r, i):
    structure = circular_structure(r)
    dmap = ndi.binary_erosion(costmap, structure=structure, iterations=i)
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
    #return np.subtract(costmap, entropymap)

    height, width = costmap.shape
    static = np.zeros((height, width), dtype=np.float64)
    for x in xrange(height):
        for y in xrange(width):
            static[x,y] = costmap[x,y] #- entropymap[x,y]
    return static

"""
Invert a binary costmap element-wise
"""
def invertMap(costmap):
    height, width = costmap.shape
    inverted = np.zeros((height, width), dtype=np.float64)
    for x in xrange(height):
        for y in xrange(width):
            c = costmap[x,y]
            inverted[x,y] = 1 - c
    return inverted


"""
Substract entropy
"""
def subtractEntropy(costmap, entropymap):
    height, width = costmap.shape
    res = np.zeros((height, width), dtype=np.float64)
    for x in xrange(height):
        for y in xrange(width):
            c = costmap[x,y]
            e = entropymap[x,y]
            d = c - e
            res[x,y] = 0 if d < 0 else d
    return res

"""
Combine an average map with another map
ie: use the average map where available, otherwise use other map
"""
def combineMaps(primary, secondary):
    height, width = primary.shape
    combined = np.zeros((height, width), dtype=np.float64)
    for x in xrange(height):
        for y in xrange(width):
            p = primary[x,y]
            s = secondary[x,y]
            combined[x,y] = p if p >= 0 else s
    return combined



"""
Return average map
where every slot has either a normalized float representing costmap value or -1
if there were no updates for it
"""
def averageMap(averagemap, updatemap):
    #return np.divide(averagemap, updatemap)

    height, width = averagemap.shape
    corrected = np.zeros((height, width), dtype=np.float64)
    #corrected = np.full((height, width), -1, dtype=np.float64)
    for x in xrange(height):
        for y in xrange(width):
            e = float(averagemap[x,y])
            u = float(updatemap[x,y])
            corrected[x,y] = -1 if u == 0 else e / u
    return corrected

"""
Return a normalized map
"""
def normalize(costmap):
    height, width = costmap.shape
    norm = np.zeros((height, width), dtype=np.float64)
    for x in xrange(height):
        for y in xrange(width):
            norm[x,y] = costmap[x,y] / 100
            #norm[x,y] = (1.0 * costmap[x,y]) / 100.0
    return norm

"""
Deflate shorten
"""
def deflateshorten(costmap):
    height, width = costmap.shape
    norm = np.zeros((height, width), dtype=np.int8)
    for x in xrange(height):
        for y in xrange(width):
            c = costmap[x,y]
            if c == 100:
                norm[x,y] = 255
    return norm


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
    
    zeros = np.zeros((height,width),dtype=np.float64)
    return zeros

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
floodfill the outside region of an image
much like the paint bucket tool in paint
"""
def floodfill(costmap):
  copy = np.copy(costmap)
  start = (1,1)
  value = 200
  floodfill_helper(copy, start, value)
  return copy

"""
helper function
"""
def floodfill_helper(costmap, start, value):
    visited, queue = set(), [start]
    h,w = costmap.shape
    while queue:
        pixel = queue.pop(0)
        if pixel not in visited:
            visited.add(pixel)
            costmap[pixel] = value
            bound_check = lambda (x,y): x > 0 and y > 0 and x < h and y < w
            wall_check = lambda (x,y): costmap[x,y] < 0.9
            check = lambda px: bound_check(px) and wall_check(px)
            (x,y) = pixel
            if check((x+1,y+1)): queue.append((x+1,y+1))
            if check((x+1,y  )): queue.append((x+1,y  ))
            if check((x  ,y+1)): queue.append((x  ,y+1))
            if check((x-1,y-1)): queue.append((x-1,y-1))
            if check((x  ,y-1)): queue.append((x  ,y-1))
            if check((x-1,y  )): queue.append((x-1,y  ))

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

def gradientMap(cmap):
    height, width = cmap.shape
    res = np.zeros((height, width), dtype=np.int8)
    for x in xrange(height):
        for y in xrange(width):
            c = cmap[x,y]
            res[x,y] = c * 255
    return res


def get_costmaps():
    global dirName
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
                break
    print("Done processing bag data")

    # Create results dir if needed
    if saveFiles:
        if not os.path.exists(resultsDir):
            os.makedirs(resultsDir)
        dirName = resultsDir + "/" + datetime.today().strftime("%m-%d-%Y(%H:%M:%S)") + "[" + whichBag + "]"
        if not os.path.exists(dirName):
            os.makedirs(dirName)

    # Function for saving costmaps to disc
    save = lambda title, cmap: scipy.misc.imsave(dirName + "/raw_" + title + ".png", scipy.misc.bytescale(cmap))

    # Create average map
    averageM = averageMap(averagemap, updatemap)
    save("averageM", averageM)

    # Create thresholded global costmap
    globalmap = thresholdmap(costmap, 90)
    save("globalmap", globalmap)

    # Combine average with full costmap
    combined  = combineMaps(averageM, globalmap)
    save("combined", combined)

    # This seems to work best
    ccostmap = np.copy(costmap)
    save("ccostmap", ccostmap)


    # Deflate
    deflated = deflateshorten(ccostmap)
    save("deflated", deflated)

    # Floodfill
    fill = floodfill(deflated)
    save("floodfill", fill)

    """
    # Normalize
    normalized = normalize(fill)
    save("normalized", normalized)

    # Invert
    inverted = invertMap(normalized)
    save("inverted", inverted)

    # Flip map
    flipped = np.flipud(inverted)
    save("flipped", flipped)

    # Regradient
    gradient = gradientMap(flipped)
    #save("gradient", gradient)

    """

    # Result
    result = deflated

    print("Done post-processing")

    # Save the deflated result
    if saveFiles:
        scipy.misc.imsave(dirName + "/result.png", result)

    # Display the views
    displayViews()

    # Close the bag
    bag.close()

    # Exit
    return 0

if __name__ == '__main__':
    try:
        #deflate()
        get_costmaps()
    except rospy.ROSInterruptException:
        pass
