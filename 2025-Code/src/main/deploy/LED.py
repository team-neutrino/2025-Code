import board
import neopixel
import sys
import time
import logging
from networktables import NetworkTables
from colorutils import Color

logging.basicConfig(level=logging.DEBUG)

'''
if len(sys.argv) != 2:
    print("Error: specify an IP to connect to!")
    ip = sys.argv[1]
'''

ip = "10.39.28.2"
global targetColorRGB
global newColorRGB
targetColorRGB = [255, 165, 0]
newColorRGB = [255, 165, 0]
pixels = neopixel.NeoPixel(board.D18, 19, auto_write=False)
rchange = 0
gchange = 0
bchange = 0

NetworkTables.setDashboardMode(1735)
NetworkTables.initialize(server=ip)

def ramp( r2, g2, b2):
    global newColorRGB
    r = newColorRGB[0]
    g = newColorRGB[1]
    b = newColorRGB[2]
    rate = 10
    if(r < r2):
        r = r + rate
    if(r > r2):
        r = r - rate
    if(g < g2):
        g = g + rate
    if(g > g2):
        g = g - rate
    if(b < b2):
        b = b + rate
    if(b > b2):
        b = b - rate
    newColorRGB[0] = min(255,max(r, 0))
    newColorRGB[1] = min(255,max(g, 0))
    newColorRGB[2] = min(255,max(b, 0))

def setCurrentColor(r, g, b):
    global targetColorRGB
    targetColorRGB[0] = r
    targetColorRGB[1] = g
    targetColorRGB[2] = b
   
def valueChanged(table, key, value, isNew):
    print("valueChanged: key: '%s'; value: %s; isNew: %s" % (key, value, isNew))
    global targetColorRGB
    if value == "white":
        setCurrentColor(255, 255, 255)
    if value == "red":
        setCurrentColor(255, 0, 0)
    if value == "orange":
        setCurrentColor(255, 165, 0)
    if value == "yellow":
        setCurrentColor(255, 255, 0)
    if value == "green":
        setCurrentColor(0, 255, 0)
    if value == "blue":
        setCurrentColor(0, 0, 255)
    if value =="purple":
        setCurrentColor(255, 0, 255)
    if value == "cyan":
        setCurrentColor(0, 255, 255)
    if value == "black":
        setCurrentColor(0, 0, 0)
    if value == "indigo":
        setCurrentColor(75, 0, 130)
    if value =="teal":
        setCurrentColor(0, 128, 128)
    for i in range(1):
        if value == "blinkwhite":
            pixels.fill((255,255,255))
            time.sleep(0.5)
            pixels.fill((0,0,0))
            time.sleep(0.5)
        if value == "blinkteal":
            pixels.fill((0,128,128))
            time.sleep(0.5)
            pixels.fill((0,0,0))
            time.sleep(0.5)
    for t in range(19,0,-1):
        for i in range(t, t + 19):
            if value == "rainbow":
                c = Color(hsv=((i-t) * (360/19), 1, 1))
                if(i < 19):
                    pixels[i] = (c.red, c.green, c.blue)
                else:
                    pixels[i - 19] = (c.red, c.green, c.blue)
                    
        

def connectionListener(connected, info):
    print(info, "; Connected=%s" % connected)

NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)
    # the robot needs to publish "/LED/color"
led = NetworkTables.getTable("/LED")
color = led.getAutoUpdateValue("color", "")
color.addListener(valueChanged, NetworkTables.NotifyFlags.UPDATE)

while True:
    time.sleep(0.01)
    ramp(targetColorRGB[0],targetColorRGB[1], targetColorRGB[2])
    pixels.fill((newColorRGB[0],newColorRGB[1], newColorRGB[2] ))    
    print("rc %d" % targetColorRGB[0])
    print("gc %d" % targetColorRGB[1])
    print("bc %d" % targetColorRGB[2])
    print(" ")
    print("rp %d" % newColorRGB[0])
    print("gp %d" % newColorRGB[1])
    print("bp %d" % newColorRGB[2])
    print(" ")
    pixels.show()