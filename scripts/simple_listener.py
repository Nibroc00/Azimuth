#!/user/bin/env python
from ast import Str
from gettext import translation
import rospy
import geometry_msgs.msg
from pynmeagps import NMEAMessage, GET
from pynmeagps import NMEAMessage, GET
from geographiclib.geodesic import Geodesic
from azimuth.msg import GPS
import math
from datetime import datetime, timezone
import serial
MAG_DECLINATION = 2.78
MPS_TO_KPH = 3.6
MPS_TO_KNOTS = 1.94384

class translator:

    def __init__(self, originlat, originlon):
        self.originlat = originlat
        self.originlon = originlon
        self.lastx = 0
        self.lasty = 0
        self.lastz = 0
        self.lastupdate = datetime.now(timezone.utc)
        self.lastspeedcalc = datetime.now(timezone.utc)
        self.geod = Geodesic(6378388, 1/297.0)
        self.vel = 0

    def updatepos(self, curx, cury, curz):
        self.lastx = curx
        self.lasty = cury
        self.lastz = curz
        self.lastupdate = datetime.now(timezone.utc)

    def getlocalloc(self):  # returns local cartesian postion
        return (self.lastx, self.lasty, self.lastz)

    def azimuth(self, curx, cury):  # Calculates the azimuth
        azimuth = math.degrees(math.atan2(curx, cury))
        return azimuth

    def distance_origin(self, curx, cury):  # returns distance from origin
        dis = math.sqrt(curx ** 2 + cury ** 2)
        return dis

    def distance_last(self, curx, cury):  # returns distance from last pos
        dis = math.sqrt((curx - self.lastx) ** 2 + (cury - self.lasty) ** 2)
        return dis

    def course(self, curx, cury):  # returns angle of travel
        azimuth = math.degrees(math.atan2(curx - self.lastx,
                                          cury - self.lasty))
        if azimuth < 0:
            azimuth = 360 + azimuth
        return azimuth

    def speed(self, curx, cury):
        curtime = datetime.now(timezone.utc)
        deltatime = curtime.timestamp() - self.lastspeedcalc.timestamp()  # sec
        if deltatime < 0.01:  # 0.01 is update rate
            return self.vel   # If updated to fase results get inaccurate
        dis = self.distance_last(curx, cury)

        mps = round(dis, 4) / round(deltatime, 8)
        self.lastspeedcalc = curtime
        self.vel = mps
        return mps

    def gpsfromlocal(self, curx, cury):  # returns gps cordinates
        azi = self.azimuth(curx, cury)
        dis = self.distance_origin(curx, cury)
        return self.geod.Direct(self.originlat, self.originlon,
                                azi, dis)

    def craftmsg(self, curx, cury, curz):  # create msgs from local x, y, z
        #Inilizise variables needed for msgs
        loc = self.gpsfromlocal(curx, cury)
        time = datetime.now(timezone.utc).time()
        course = self.course(curx, curz)
        coursem = course + MAG_DECLINATION
        if (course + MAG_DECLINATION) < 360:
            coursem = 360 - (course + MAG_DECLINATION)
        speed = self.speed(curx, cury)


        GGA = NMEAMessage('GP', 'GGA', GET, time=time,
                          lat=round(loc['lat2'], 7), NS='N',
                          lon=round(loc['lon2'], 7), EW='W',
                          quality=1, numSV=12,
                          HPOD=1, alt=round(curz, 2),
                          altUnit='M', sep=0, sepUnit='M',
                          diffStation=0)

        VTG = NMEAMessage('GP', 'VTG', GET, cogt=round(course, 2),
                          cogtUnit='DEG', cogm=round(coursem, 2),
                          sogn=round(speed * MPS_TO_KPH, 2),
                          sognUnit='K',
                          sogk=round(speed * MPS_TO_KNOTS, 2),
                          sogkUnit='N')

        RMC = NMEAMessage('GP', 'RMC', GET,
                          status='A',
                          lat=round(loc['lat2'], 7), NS='N',
                          lon=round(loc['lon2'], 7), EW='W',
                          spd=round(speed * MPS_TO_KNOTS, 2),
                          cog=round(course, 2),
                          mv=MAG_DECLINATION,
                          mvEW='E',
                          posMode='A',
                          navStatus='A'
                          )

        return {'GGA': GGA, 'VTG': VTG, 'RMC': RMC}


trans = translator(40.819375, -96.706161)
ser = serial.Serial('/dev/ttyUSB0', baudrate=38400)

pub = rospy.Publisher('gps_output', GPS, queue_size=10)
rospy.init_node('translator', anonymous=True)


def callback(t):
    location = trans.gpsfromlocal(t.transform.translation.x,  # Calc gps lat/
                                  t.transform.translation.y)  # lon from local
    msgs = trans.craftmsg(t.transform.translation.x,  # Creates a NMEA 
                          t.transform.translation.y,  # msgs from local postion
                          t.transform.translation.z)
    ser.write(msgs['GGA'].serialize())
    ser.write(msgs['VTG'].serialize())
    ser.write(msgs['RMC'].serialize())
    rospy.loginfo("{}\n{}\n{}".format(msgs['GGA'], msgs['VTG'], msgs['RMC']))

    gps_message = GPS()
    gps_message.latitude = location['lat2']
    gps_message.longitude = location['lon2']
    gps_message.altitude = t.transform.translation.z
    pub.publish(gps_message)

    trans.updatepos(t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z)


def listener():
    rospy.Subscriber("vicon/CNC_Head/CNC_Head", geometry_msgs.msg.TransformStamped,
                     callback)
    rospy.spin()


if __name__ == '__main__':
    listener() 
