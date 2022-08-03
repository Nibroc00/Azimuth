#!/user/bin/env python
import rospy
import geometry_msgs.msg
from pynmeagps import NMEAMessage, GET
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
INITIAL_QUATERNION_ROTATION = np.quaternion(0, 0, 1, 0)

# Initialize ROS node
rospy.init_node('translator', anonymous=True)

# Parameters from ROS launch
vicon_target_id = rospy.get_param("/azimuth/vicon_target_id")
serial_port_handle = rospy.get_param("/azimuth/serial_port_handle")
baud_rate = rospy.get_param("/azimuth/baud_rate")
LON = rospy.get_param("/azimuth/lon") # Longitude of (0, 0) of your tracking system
LAT = rospy.get_param("/azimuth/lat") # Latitude of (0, 0) of your tracking system

# Translator object that takes cartesian coordinates (in meters) and translates to
# GPS coordinates based off of the (Longitude, Latitude) offset of (0, 0) origin.
# Also takes quaternion orientation data and translates to a heading.
trans = Translator.Translator(LAT, LON, INITIAL_QUATERNION_ROTATION)

# Open serial port from Node to Drone
ser = serial.Serial(serial_port_handle, baud_rate)

# variable needed to set send rate for serial msg to prevent brown outs
global previousTime   
previousTime = datetime.now(timezone.utc).timestamp()

# Creates NMEA messages that ArduPilot requests and is equip to handle 
# (GGA, VTG, RMC, HDT)
# Takes current X position, Y Position, Z position, and a quaternion object of the 
# current orientation.
def craftMsgs(curx, cury, curz, q):

    time = datetime.now(timezone.utc).time()
    # Use Translator to translate data from local x, y, z to GPS
    locationGPS = trans.localToGPS(curx, cury)
    course = trans.course(curx, cury)
    courseM = course + MAG_DECLINATION
    speed = trans.speed(curx, cury)
    heading = trans.heading(q)

    # course calculates from -180 to 180 degrees, we want 0 to 360 degress
    if (courseM) < 360:
        courseM = 360 - (courseM)

    # GPS location data message
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

    # Velocity and course data message
    VTG = NMEAMessage(  'GP',
                        'VTG',
                        GET,
                        cogt=round(course, 2),
                        cogtUnit='DEG', cogm=round(courseM, 2),
                        sogn=round(speed * MPS_TO_KPH, 3),
                        sognUnit='K',
                        sogk=round(speed * MPS_TO_KNOTS, 2),
                        sogkUnit='N'
                    )

    # Required minimum data message
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

    # Heading data message
    HDT = NMEAMessage( 'GP',
                       'HDT',
                       GET,
                       heading=heading
                     )

    return {'GGA': GGA, 'VTG': VTG, 'RMC': RMC, 'HDT': HDT}


# Callback for when Vicon ROS topic gets published to
def callback(t):
    global previousTime

    # Create NMEA messages from data published on topic
    msgs = craftMsgs(t.transform.translation.x,  # Creates a NMEA msgs from local postion
                     t.transform.translation.y,
                     t.transform.translation.z,
                     t.transform.rotation)
    
    time = datetime.now(timezone.utc).timestamp()  # gets current time in sec

    # Callback only sends messages if 0.125 seconds has passed (8Hz). This is
    # because faster rates can brown out the drone, causing it to have unstable 
    # readings.
    if time - previousTime > 0.125: 
        previousTime = datetime.now(timezone.utc).timestamp()
        ser.write(msgs['GGA'].serialize())
        ser.write(msgs['VTG'].serialize())
        ser.write(msgs['RMC'].serialize())
        ser.write(msgs['HDT'].serialize())

    # Update current position data for speed calculations
    trans.updatePos(t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z)

# Listen to Vicon's published topic on your target
def listener():
    rospy.Subscriber(vicon_target_id, geometry_msgs.msg.TransformStamped, callback)
    rospy.loginfo("Starting loop")
    rospy.spin()

# Main
if __name__ == '__main__':
    listener()
