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
targetColorRGB = [0, 0, 0]
newColorRGB = [0, 0, 0]
blink = False
pixels = neopixel.NeoPixel(board.D18, 19, auto_write=False)
counter = 0

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
    setNewColor(r,g,b)

def setTargetColor(r, g, b):
    global targetColorRGB
    targetColorRGB[0] = r
    targetColorRGB[1] = g
    targetColorRGB[2] = b

def setNewColor(r, g, b):
    global newColorRGB
    newColorRGB[0] = min(255,max(r, 0))
    newColorRGB[1] = min(255,max(g, 0))
    newColorRGB[2] = min(255,max(b, 0))

def blinkColor(rgb):
    global counter
    counter += 1
    if(counter <= 10):
        setNewColor(rgb[0],rgb[1],rgb[2])
    elif(counter <= 20):
        setNewColor(0,0,0)
    else:
        counter = 0

def valueChanged(table, key, value, isNew):
    print("valueChanged: key: '%s'; value: %s; isNew: %s" % (key, value, isNew))
    global targetColorRGB
    global blink
    blink = False
    if value == "white":
        setTargetColor(255, 255, 255)
    if value == "red":
        setTargetColor(255, 0, 0)
    if value == "orange":
        setTargetColor(255, 80, 0)
    if value == "yellow":
        setTargetColor(255, 255, 0)
    if value == "green":
        setTargetColor(0, 255, 0)
    if value == "blue":
        setTargetColor(0, 0, 255)
    if value =="purple":
        setTargetColor(255, 0, 255)
    if value == "cyan":
        setTargetColor(0, 255, 255)
    if value == "black":
        setTargetColor(0, 0, 0)
    if value == "indigo":
        setTargetColor(75, 0, 130)
    if value =="turquoise":
        setTargetColor(27, 220, 85)
    if value == "blinkturquoise":
        blink = True
        setTargetColor(27,220,85)
    if value == "blinkwhite":
        blink=True
        setTargetColor(255,255,255)
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

#def printRGB():
    # print("rc %d" % targetColorRGB[0])
    # print("gc %d" % targetColorRGB[1])
    # print("bc %d" % targetColorRGB[2])
    # print(" ")
    # print("rp %d" % newColorRGB[0])
    # print("gp %d" % newColorRGB[1])
    # print("bp %d" % newColorRGB[2])
    # print(" ")

while True:
    time.sleep(0.01)
    ramp(targetColorRGB[0],targetColorRGB[1], targetColorRGB[2])
    if (blink):
        blinkColor(newColorRGB)
    pixels.fill((newColorRGB[0],newColorRGB[1], newColorRGB[2]))
    pixels.show()