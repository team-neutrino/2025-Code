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
pixels = neopixel.NeoPixel(board.D18, 298, auto_write=False)
previousBlinkTime = 0
amount = 0

white = (64, 64, 64)
red = (64, 0, 0)
orange = (84, 18, 0)
yellow = (64, 64, 0)
green = (0, 64, 0)
purple = (64, 0, 64)
black = (0, 0, 0)
cyan = (0, 64, 64)
turquoise = (9, 55, 19)
pink = (80, 30, 30)

NetworkTables.setDashboardMode(1735)
NetworkTables.initialize(server=ip)

def ramp(rgb):
    global newColorRGB
    rgb2 = newColorRGB
    rate = 10
    if(rgb2[0] < rgb[0]):
       rgb2[0] += rate
    if(rgb2[0] > rgb[0]):
       rgb2[0] -= rate
    if(rgb2[1] < rgb[1]):
        rgb2[1] += rate
    if(rgb2[1] > rgb[1]):
        rgb2[1] -= rate
    if(rgb2[2] < rgb[2]):
        rgb2[2] += rate
    if(rgb2[2] > rgb[2]):
        rgb2[2] -= rate
    setNewColor(rgb2)

def setTargetColor(rgb):
    global targetColorRGB
    targetColorRGB = rgb

def setNewColor(rgb):
    global newColorRGB
    newColorRGB[0] = min(255,max(rgb[0], 0))
    newColorRGB[1] = min(255,max(rgb[1], 0))
    newColorRGB[2] = min(255,max(rgb[2], 0))

def blinkColor(rgb):
    global previousBlinkTime
    global amount
    global blink
    timePassed = time.time() - previousBlinkTime
    if timePassed < 1:
        setNewColor(rgb)
    elif timePassed > 1 and timePassed < 2:
        setNewColor(black)
    else:
        amount -= 1
        if amount == 0:
            blink = False
            setNewColor(rgb)

def valueColorChanged(table, key, value, isNew):
    print("valueColorChanged: key: '%s'; value: %s; isNew: %s" % (key, value, isNew))
    global targetColorRGB
    if value == "white":
        setTargetColor(white)
    if value == "red":
        setTargetColor(red)
    if value == "orange":
        setTargetColor(orange)
    if value == "yellow":
        setTargetColor(yellow)
    if value == "green":
        setTargetColor(green)
    if value == "black":
        setTargetColor(black)
    if value=="cyan":
        setTargetColor(cyan)
    if value =="turquoise":
        setTargetColor(turquoise)
    if value == "pink":
        setTargetColor(pink)
    if value == "purple":
        setTargetColor(purple)
    for t in range(19,0,-1):
        for i in range(t, t + 19):
            if value == "rainbow":
                c = Color(hsv=((i-t) * (360/19), 1, 1))
                if(i < 19):
                    pixels[i] = (c.red, c.green, c.blue)
                else:
                    pixels[i - 19] = (c.red, c.green, c.blue)
                    

def valueStateChanged(table, key, value, isNew):
    print("valueStateChanged: key: '%s'; value: %s; isNew: %s" % (key, value, isNew))
    global blink
    global amount
    global previousBlinkTime
    if value == "blink":
        blink = True
        previousBlinkTime = time.time()
        amount = 1
    if value== "blinktwice":
        blink = True
        previousBlinkTime = time.time()
        amount = 2 
    if value == "solid":
        blink = False

def connectionListener(connected, info):
    print(info, "; Connected=%s" % connected)

NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)
led = NetworkTables.getTable("/LED")
color = led.getAutoUpdateValue("color", "")
state = led.getAutoUpdateValue("state", "")
color.addListener(valueColorChanged, NetworkTables.NotifyFlags.UPDATE)
state.addListener(valueStateChanged, NetworkTables.NotifyFlags.UPDATE)

# def printRGB():
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
    ramp(targetColorRGB)
    if (blink):
        blinkColor(newColorRGB)
    pixels.fill((newColorRGB[0],newColorRGB[1], newColorRGB[2]))
    pixels.show()