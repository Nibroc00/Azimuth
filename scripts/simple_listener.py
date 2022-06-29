#!/user/bin/env python
from ast import Str
from gettext import translation
import rospy
import geometry_msgs.msg
from pynmeagps import NMEAMessage, GET
from geographiclib.geodesic import Geodesic
from azimuth.msg import GPS
import math
from datetime import datetime, timezone
import serial

class translator:

    def __init__(self, originlat, originlon):
        self.originlat = originlat
        self.originlon = originlon
        self.lastx = 0
        self.lasty = 0
        self.lastz = 0
        self.geod = Geodesic(6378388, 1/297.0)

    def updatepos(self, curx, cury, curz):
        self.lastx = curx
        self.lasty = cury
        self.lastz = curz

    def getlocalloc(self):  # returns local cartesian postion
        return (self.lastx, self.lasty, self.lastz)

    def azimuth(self, curx, cury):  # Calculates the azimuth
        azimuth = math.degrees(math.atan2(curx, cury))
        return azimuth

    def distance(self, curx, cury):  # returns distance form origin
        dis = math.sqrt(curx ** 2 + cury ** 2)
        return dis

    def gpsfromlocal(self, curx, cury):  # returns gps cordinates
        azi = self.azimuth(curx, cury)
        dis = self.distance(curx, cury)
        return self.geod.Direct(self.originlat, self.originlon,
                                azi, dis)

    def craftmsg(self, curx, cury):
        loc = self.gpsfromlocal(curx, cury)
        time = datetime.now(timezone.utc).time

        msg = NMEAMessage('GP', 'GGA', GET, time=time,
                          lat=loc['lat2'], NS='N',
                          lon=loc['lon2'], EW='W',
                          quality=1, numSV=12,
                          HPOD=0, alt=self.lastz,
                          altUnit='M', sep=0, sepUnit='M',
                          diffStation=0)
        return msg


trans = translator(40.819375, -96.706161)
ser = serial.Serial('/dev/ttyUSB0', baudrate=230400)

pub = rospy.Publisher('gps_output', GPS, queue_size=10)
rospy.init_node('translator', anonymous=True)

def callback(t):

    location = trans.gpsfromlocal(t.transform.translation.x,
                                  t.transform.translation.y)
    msg = trans.craftmsg(t.transform.translation.x,
                         t.transform.translation.y)

    rospy.loginfo("{}".format(msg))
    ser.write(msg.serialize())
    rospy.loginfo("azi: {}\tdis{}\tlat: {}\tlon: {}".format(azi, dis, location['lat2'], location['lon2']))
    



    gps_message = GPS()
    gps_message.latitude = location['lat2']
    gps_message.longitude = location['lon2']
    gps_message.altitude = t.transform.translation.z
    pub.publish(gps_message)

    trans.updatepos(t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z)


def listener():
    rospy.Subscriber("vicon/wand/wand", geometry_msgs.msg.TransformStamped, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
    