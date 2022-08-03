'''
Translator class that takes cartesian coordinates (in meters) and translates to
GPS coordinates based off of the (Longitude, Latitude) offset of (0, 0) origin.
Also takes quaternion orientation data and translates to a heading.
'''
from pynmeagps import NMEAMessage, GET
from geographiclib.geodesic import Geodesic
import math
from datetime import datetime, timezone
import numpy as np



class Translator:

    # Constructor, requires latitude of origin, longitude of origin, and base quaternion oreintation.
    def __init__(self, originLat, originLon, originQ):
        self.originLat = originLat
        self.originLon = originLon
        self.originQ = originQ  # e.g. np.quaternion(0, 0, 1, 0)
        self.lastX = 0
        self.lastY = 0
        self.lastZ = 0
        self.lastUpdate = datetime.now(timezone.utc)
        self.lastSpeedCalc = datetime.now(timezone.utc)
        self.geod = Geodesic.WGS84
        self.velocity = 0

    def updatePos(self, currrentX, currrentY, currrentZ):
        self.lastX = currrentX
        self.lastY = currrentY
        self.lastZ = currrentZ
        self.lastUpdate = datetime.now(timezone.utc)

    # Returns the azimuth of the given postition from the origin
    def azimuth(self, currrentX, currrentY):  
        azimuth = math.degrees(math.atan2(currrentX, currrentY))
        return azimuth

    # Returns the 3D vector that points in the same direction as given quaternion
    def quaternionToVector(self, q):
        q = np.quaternion(q.w, q.x, q.y, q.z)
        mag = q * self.originQ * q.inverse()
        return {'x': mag.x, 'y': mag.y, 'z': mag.z}

    # Returns the distance from origin of the given position
    def distanceOrigin(self, currrentX, currrentY):
        distance = math.sqrt(currrentX ** 2 + currrentY ** 2)
        return distance

    # Returns the distance from last known postition
    def distanceLast(self, currrentX, currrentY):
        distance = math.sqrt((currrentX - self.lastX) ** 2 + (currrentY - self.lastY) ** 2)
        return distance

    # Returns the angle of travel from north
    def course(self, currrentX, currrentY):
        deltaX = currrentX - self.lastX
        deltaY = currrentY - self.lastY

        # Check and make sure difference between points isnt too small else return 0
        # Necessary because of Vicon noise when not moving
        if deltaX < 0.001: deltaX = 0
        if deltaY < 0.001: deltaY = 0

        azimuth = math.degrees(math.atan2(currrentX - self.lastX, currrentY - self.lastY))
        if azimuth < 0: azimuth = 360 + azimuth
        return azimuth

    # Returns heading from a given quaternion
    def heading(self, q):
        mag = self.quaternionToVector(q)
        heading = math.degrees(math.atan2(mag['x'], mag['y']))
        return heading

    # Returns the speed based on change in current and last position/time, returns 
    # a value in meters per second
    def speed(self, currrentX, currrentY): # Returns speed based on change in time and position
        currentTime = datetime.now(timezone.utc)
        deltaTime = currentTime.timestamp() - self.lastSpeedCalc.timestamp()

        # Only returns a change in velocity if 0.01 seconds has passed (100Hz). This is
        # because faster rates can give bad results due to Vicon noise, causing it to 
        # have unstable readings.
        if deltaTime < 0.01:
            return self.velocity

        distance = self.distanceLast(currrentX, currrentY)
        mps = round(distance, 4) / round(deltaTime, 8)
        self.lastSpeedCalc = currentTime
        self.velocity = mps
        return mps

    # Returns GPS coordinates based on cartesian coordinates from origin, where the
    # origin's longitude and latitude are know. 
    def localToGPS(self, currrentX, currrentY):
        azimuth = self.azimuth(currrentX, currrentY)
        distance = self.distanceOrigin(currrentX, currrentY)
        return self.geod.Direct(self.originLat, self.originLon, azimuth, distance)


