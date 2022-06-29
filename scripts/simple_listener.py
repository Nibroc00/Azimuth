#!/user/bin/env python
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import pynmeagps
from geographiclib.geodesic import Geodesic
import math


class translator:


    def __init__(self, originlat, originlon):
        self.originlat = originlat
        self.originlon = originlon
        self.lastx = 0
        self.lasty = 0
        self.lastx = 0
        self.geod = Geodesic(6378388, 1/297.0)

    def updatepos(self, curx, cury, curz):
        self.lastx = curx
        self.lasty = cury
        self.lastz = curz

    def getlocalloc(self):
        return (self.lastx, self.lasty, self.lastz)

    def azimuth(self, curx, cury):
        azimuth = math.degrees(math.atan2(curx ,cury))
        return azimuth

    def distance(self, curx, cury):
        dis = math.sqrt(curx ** 2 + cury ** 2)
        return dis

    def gpsfromlocal(self, azi, dis):
        return self.geod.Direct(self.originlat, self.originlon,
                                azi, dis)

trans = translator(40.819375, -96.706161)

def callback(t):

    azi = trans.azimuth(t.transform.translation.x,
                        t.transform.translation.y)
    dis = trans.distance(t.transform.translation.x,
                        t.transform.translation.y)
    location = trans.gpsfromlocal(azi, dis)

    rospy.loginfo("azi: {}\tdis{}\tlat: {}\tlon: {}".format(azi, dis, location['lat2'], location['lon2']))
    trans.updatepos(t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z)
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("vicon/BATMAN/BATMAN", geometry_msgs.msg.TransformStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
