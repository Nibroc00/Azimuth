'''Translator module, takes corridinates and turns them into gps
    data'''
import pynmeagps, geographiclib
import math
class translator:

    def __init__(self, originlat, originlon):
        self.originlat = originlat
        self.originlon = originlon
        self.lastx = 0
        self.lasty = 0
        self.lastx = 0

    def updatepos(self, curx, cury, curz):
        self.lastx = curx
        self.lasty = cury
        self.lastz = curz

    def getlocalloc(self):
        return (self.lastx, self.lasty, self.lastz)

    def azimuth(self, curx, cury):
        azimuth = math.degrees(math.atan2((curx-self.lastx),(cury-self.lasty)))
        return azimuth
