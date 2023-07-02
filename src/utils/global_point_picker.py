#!/usr/bin/env python
from fileinput import filename
import rospy
from geometry_msgs.msg import Point
from std_srvs.srv import Empty
import os

'''
Generate global map
'''

class GlobalPointPicker:
    def __init__(self):
        self.cur_pos = Point()
        self.points = []

        rospy.init_node("global_map_generator_node")
        rospy.Subscriber('/utm_pos', Point, self.onReceiveUTMPos)
        rospy.Service('~save_map', Empty, self.saveMap)
        rospy.Service('~add_point', Empty, self.addPoint)

        self.global_map_file_path = './points.txt'

    def onReceiveUTMPos(self, msg):
        # update previous position
        self.cur_pos = msg

    def addPoint(self, args):
        rospy.logdebug('Add point (%f, %f)' % (self.cur_pos.x, self.cur_pos.y))
        self.points.append((self.cur_pos.x, self.cur_pos.y))
    
    def saveMap(self, args):
        # create directory -> permission issue
        # if not os.path.exists(os.path.dirname(self.global_map_file_path)):
        #     os.makedirs(os.path.dirname(self.global_map_file_path))
        with open(self.global_map_file_path, 'w') as f:
            for point in self.points:
                f.write('%f %f \n' % (point[0], point[1]))

        rospy.logdebug('Save map to %s' % (self.global_map_file_path))
        


if __name__ == '__main__':
    try:
        globalPointPicker = GlobalPointPicker()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Error!')
