#!/user/bin/env python
from ast import Str
from gettext import translation
# from Azimuth.src.Translator import Translator
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
import pigpio
import threading
import time

# I2C addres
I2C_ADDR = 0x0E

# Physical Constants
MPS_TO_KPH = 3.6
MPS_TO_KNOTS = 1.94384

# Constants that change based on your geological position and setup
MAG_DECLINATION = 2.78
LAT = 40.819375
LON = -96.706161
INITIAL_QUATERNION_ROTATION = np.quaternion(0, 0, 1, 0)

trans = Translator.Translator(LAT, LON, INITIAL_QUATERNION_ROTATION)
#ser = serial.Serial('/dev/ttyUSB0', baudrate=115200)
rospy.init_node('translator', anonymous=True)
global last_msg_sent_time   # variable needed to set send rate for serial msg
last_msg_sent_time = datetime.now(timezone.utc).timestamp()

# init i2c
pi = pigpio.pi()
if not pi.connected:
    exit()

        #pi.bb_i2c_open(10, 11, 50000)
        #pi.bsc_xfer((I2C_ADDR << 16) | 0x305,[])
pi.bsc_i2c(I2C_ADDR)
        #pi.bsc_xfer(0,[])
        #pi.bb_i2c_close(10)

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
    global last_msg_sent_time  # !!!!!!!DONT REMOVE !!!!!!!!! Has to be defined twice or it get unhappy
    msgs = craftMsgs(t.transform.translation.x,  # Creates a NMEA msgs from local postion
                    t.transform.translation.y,
                    t.transform.translation.z)
    rotation = trans.quaternionToVector(t.transform.rotation)
    now = datetime.now(timezone.utc).timestamp()  # gets current time in sec

    # 8hz loop
    if now - last_msg_sent_time > 0.125:  # rate limit so we dont brown out
        last_msg_sent_time = datetime.now(timezone.utc).timestamp()
        # ser.write(msgs['GGA'].serialize())
        # ser.write(msgs['VTG'].serialize())
        # ser.write(msgs['RMC'].serialize())
        #rospy.loginfo("\nx: {}\ny: {}\nz: {}".format(rotation['x'],
        #                                             rotation['y'],
        #                                             rotation['z']))

    # check i2c reg for messages
    s, b, d = pi.bsc_i2c(I2C_ADDR)
    if b:
        if d[0] == 0x07: # respond with ID
            pi.bsc_i2c(I2C_ADDR,bytes.fromhex("C4"))
            rospy.loginfo(d)
        elif d[0] == 0x10 or d[0] == 0x11:  #config 1 and 2 respectivly
            rospy.loginfo(d)
        elif d[0] == 0x01:  # respond wit mag data
            pi.bsc_i2c(I2C_ADDR,bytes.fromhex("BEEFFF"))
            rospy.loginfo(d)
        elif d[0] == 0x00:
            pi.bsc_i2c(I2C_ADDR, bytes.fromhex("04"))

    trans.updatePos(t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z)

def listener():
    rospy.Subscriber("vicon/wand/wand", geometry_msgs.msg.TransformStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

# Stop i2c device
pi.bsc_i2c(0)
pi.stop()
