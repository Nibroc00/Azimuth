'''Translator module, takes corridinates and turns them into gps
    data'''
from pynmeagps import NMEAMessage, GET
from geographiclib.geodesic import Geodesic
import math
from datetime import datetime, timezone
import numpy as np



class Translator:

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

    # def getlocalloc(self):  # returns local cartesian postion
    #     return {"lastX": self.lastX, "lastY": self.lastY, "lastZ": self.lastZ}

    def azimuth(self, currrentX, currrentY):  # Calculates the azimuth
        azimuth = math.degrees(math.atan2(currrentX, currrentY))
        return azimuth

    # def toeuler(self, qx, qy, qz, qw):
    #     # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    #     # x-axis
    #     sinr_cosp = 2 * (qw * qx + qy * qz)
    #     cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    #     roll = math.atan2(sinr_cosp, cosr_cosp)

    #     # y-axis
    #     sinp = 2 * (qw * qy - qz * qx)

    #     if abs(sinp) >= 1:
    #         pitch = math.copysign(math.pi / 2, sinp)
    #     else:
    #         pitch = math.asin(sinp)

    #     # z-axis
    #     siny_cosp = 2 * (qw * qz + qx * qy)
    #     cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    #     yaw = math.atan2(siny_cosp, cosy_cosp)

    #     return (roll, pitch, yaw)

    # def oldtomagnitudes(self, qx, qy, qz, qw):
    #     xr, yr, zr = self.toeuler(qx, qy, qz, qw)
    #     zr = -zr  # flip it
    #     if zr < 0:
    #         zr = (math.pi * 2) + zr
    #     if xr < 0:
    #         xr = (math.pi * 2) + xr
    #     x = math.sin(zr)
    #     y = math.cos(zr)
    #     z = math.sin(xr)
    #     b = math.sqrt(x**2 + y**2)
    #     z = math.sin(b * math.tan(xr))

    #     # return z
    #     return {'x': qx, 'y': qy, 'z': qz}

    def quaternionToVector(self, q): #takes measured quarternion rotation and calculates 3D vector based on originQ
        q = np.quaternion(q.w, q.x, q.y, q.z)
        mag = q * self.originQ * q.inverse()
        return {'x': mag.x, 'y': mag.y, 'z': mag.z}

    def distanceOrigin(self, currrentX, currrentY):  # returns distance from origin
        distance = math.sqrt(currrentX ** 2 + currrentY ** 2)
        return distance

    def distanceLast(self, currrentX, currrentY):  # returns distance from last pos
        distance = math.sqrt((currrentX - self.lastX) ** 2 + (currrentY - self.lastY) ** 2)
        return distance

    def course(self, currrentX, currrentY):  # returns angle of travel
        deltaX = currrentX - self.lastX
        deltaY = currrentY - self.lastY

        # Check and make sure difference between points isnt too small
        if deltaX < 0.001: deltaX = 0
        if deltaY < 0.001: deltaY = 0

        azimuth = math.degrees(math.atan2(currrentX - self.lastX, currrentY - self.lastY))
        if azimuth < 0: azimuth = 360 + azimuth
        return azimuth

    def speed(self, currrentX, currrentY): # Returns speed based on change in time and position
        currentTime = datetime.now(timezone.utc)
        deltaTime = currentTime.timestamp() - self.lastSpeedCalc.timestamp()
        if deltaTime < 0.01:  # 0.01 is update rate
            return self.velocity   # If updated to fase results get inaccurate
        distance = self.distanceLast(currrentX, currrentY)

        mps = round(distance, 4) / round(deltaTime, 8)
        self.lastSpeedCalc = currentTime
        self.velocity = mps
        return mps

    def localToGPS(self, currrentX, currrentY):  # returns gps cordinates based on initial longitude and latitude
        azimuth = self.azimuth(currrentX, currrentY)
        distance = self.distanceOrigin(currrentX, currrentY)
        return self.geod.Direct(self.originLat, self.originLon, azimuth, distance)


