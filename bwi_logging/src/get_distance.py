
import time
import rosbag
import rospy
import datetime

if rospy.has_param('bag_file'):
  bag_file = rospy.get_param('bag_file', '')
  print('bag_file: ' + bag_file)
else:
  exit('please set the bag_file first: "rosparam set bag_file /path/to/your/bag_file"')

bag = rosbag.Bag(bag_file)

time_start = -1
time_consumed = 0

distance_traveled = 0.0
x = None
y = None

for topic, msg, t in bag.read_messages(topics=['odom']):
  time_current = int(msg.header.stamp.secs)

  x_current = float(msg.pose.pose.position.x)
  y_current = float(msg.pose.pose.position.y)

  if time_start < 0:
    x = x_current
    y = y_current
    time_start = time_current
    continue
  
  distance_traveled += ((x_current - x)**2 + (y_current - y)**2)**0.5

  x = x_current
  y = y_current

time_consumed = time_current - time_start
print('time_consumed: ' + str(datetime.timedelta(seconds=time_consumed)))
print('distance_traveled: ' + str(distance_traveled))

bag.close()

