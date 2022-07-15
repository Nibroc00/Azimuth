#!/user/bin/env python
from ast import Str
from gettext import translation
import rospy
import geometry_msgs.msg
from pynmeagps import NMEAMessage, GET
from geographiclib.geodesic import Geodesic
from azimuth.msg import GPS
from datetime import datetime, timezone
import serial
import Translator
import numpy as np
import quaternion

# Physical Constants
MPS_TO_KPH = 3.6
MPS_TO_KNOTS = 1.94384

# Constants that change based on your geological position and setup
MAG_DECLINATION = 2.78
LAT = 40.819375
LON = -96.706161
INITIAL_QUATERNION_ROTATION = np.quaternion(0, 0, 1, 0)

trans = Translator.Translator(LAT, LON, INITIAL_QUATERNION_ROTATION) # Initilize translator object
                                                                     # Does translation between cartesion
                                                                     # and gps corridnate space
                                                                     # and between quaternion and 3d vectors
ser = serial.Serial('/dev/ttyUSB0', baudrate=115200)
rospy.init_node('translator', anonymous=True)
global last_msg_sent_time   # variable needed to set send rate for serial msg
last_msg_sent_time = datetime.now(timezone.utc).timestamp()


def craftMsgs(curx, cury, curz):  # Creates messages that ArduPilot request and is equip to handle

    time = datetime.now(timezone.utc).time()
    # Use Translator to translate data from local x, y, z to GPS
    locationGPS = trans.localToGPS(curx, cury)
    course = trans.course(curx, cury)
    courseM = course + MAG_DECLINATION
    speed = trans.speed(curx, cury)

    # course calculates from -180-180 degrees, we want 0-360
    if (courseM) < 360:
        courseM = 360 - (courseM)


    GGA = NMEAMessage(  'GP',
                        'GGA',
                        GET,
                        time=time,
                        lat=round(locationGPS['lat2'], 7), NS='N',
                        lon=round(locationGPS['lon2'], 7), EW='W',
                        quality=1, numSV=12,
                        HPOD=1, alt=round(curz, 2),
                        altUnit='M', sep=0, sepUnit='M',
                        diffStation=0
                    )

    VTG = NMEAMessage(  'GP',
                        'VTG',
                        GET,
                        cogt=round(course, 2),
                        cogtUnit='DEG', cogm=round(courseM, 2),
                        sogn=round(speed * MPS_TO_KPH, 2),
                        sognUnit='K',
                        sogk=round(speed * MPS_TO_KNOTS, 2),
                        sogkUnit='N'
                    )

    RMC = NMEAMessage(  'GP',
                        'RMC',
                        GET,
                        status='A',
                        lat=round(locationGPS['lat2'], 7), NS='N',
                        lon=round(locationGPS['lon2'], 7), EW='W',
                        spd=round(speed * MPS_TO_KNOTS, 2),
                        cog=round(course, 2),
                        mv=MAG_DECLINATION,
                        mvEW='E',
                        posMode='A',
                        navStatus='A'
                    )

    return {'GGA': GGA, 'VTG': VTG, 'RMC': RMC}

def callback(t):

    global last_msg_sent_time  # Has to be defined twice or it get unhappy
    msgs = craftMsgs(t.transform.translation.x,  # Creates a NMEA msgs from local postion
                     t.transform.translation.y,
                     t.transform.translation.z)
    # 8hz loop
    now = datetime.now(timezone.utc).timestamp()  # gets current time in sec
    if now - last_msg_sent_time > 0.125:  # rate limit so we dont brown out
        last_msg_sent_time = datetime.now(timezone.utc).timestamp()
        ser.write(msgs['GGA'].serialize())
        ser.write(msgs['VTG'].serialize())
        ser.write(msgs['RMC'].serialize())

    # Update current position data for speed calculations
    trans.updatePos(t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z)


def listener():
    rospy.Subscriber("vicon/wand/wand", geometry_msgs.msg.TransformStamped, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
