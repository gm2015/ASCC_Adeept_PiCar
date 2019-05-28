#!/usr/bin/python3
# File name   : led_strip.py
# Description : Control the led strip (12 LED pixels) 
# Modified    : 2019/5/8
# by HaDo
import RPi.GPIO as GPIO
import time
from rpi_ws281x import *
import threading as thread
import os
import sound

path = os.path.dirname(os.path.abspath(__file__))

WHITE = Color(255, 255, 255)
# Order of each LED pixels
PIXELS = (6, 7, 8, 9, 10, 11, 3, 4, 5, 0, 1, 2)  # Depend on the wire connection order

# LED strip configuration:
LED_COUNT      = 12      # Number of LED pixels.
LED_PIN        = 12      # GPIO pin connected to the pixels (18 uses PWM!).
#LED_PIN        = 10      # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53

strip = ""
flash = True

def wheel(pos):
    """Generate rainbow colors across 0-255 positions."""
    if pos < 85:
        return Color(pos * 3, 255 - pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return Color(255 - pos * 3, 0, pos * 3)
    else:
        pos -= 170
        return Color(0, pos * 3, 255 - pos * 3)

def colorWipe(strip, color):
    """Wipe color across display a pixel at a time."""
    for i in range(strip.numPixels()):
        strip.setPixelColor(PIXELS[i], color)
        strip.show()
        time.sleep(0.005)

def rainbowCycle(strip, wait_ms=20, iterations=5, sound_on = True):    
    """Draw rainbow that uniformly distributes itself across all pixels."""
    if sound_on:
        sound.play_mp3('{}/music/rainbow.mp3'.format(path), loops = -1)
    for j in range(256*iterations):
        for i in range(strip.numPixels()):
            strip.setPixelColor(PIXELS[i], wheel((int(i * 256 / strip.numPixels()) + j) & 255))
        strip.show()
        time.sleep(wait_ms/1000.0)
    if sound_on:
        sound.stop()
    side_off()

def rainbow(strip, wait_ms=20, iterations=1):
    """Draw rainbow that fades across all pixels at once."""    
    for j in range(256*iterations):
        for i in range(strip.numPixels()):
            strip.setPixelColor(PIXELS[i], wheel((i+j) & 255))
        strip.show()
        time.sleep(wait_ms/1000.0)
    side_off()

def rainbow_sound(strip, wait_ms=20, iterations=1, sound_on = True):
    """Draw rainbow that fades across all pixels at once with music."""
    # Write your code here
    pass
        
def theaterChase(strip, color, wait_ms=50, iterations=10):
    """Movie theater light style chaser animation."""
    for j in range(iterations):
        for q in range(3):
            for i in range(0, strip.numPixels(), 3):
                strip.setPixelColor(PIXELS[i+q], color)
            strip.show()
            time.sleep(wait_ms/1000.0)
            for i in range(0, strip.numPixels(), 3):
                strip.setPixelColor(PIXELS[i+q], 0)
    side_off()
            
def theaterChaseRainbow(strip, wait_ms=50):
    """Rainbow movie theater light style chaser animation."""
    for j in range(256):
        for q in range(3):
            for i in range(0, strip.numPixels(), 3):
                strip.setPixelColor(PIXELS[i+q], wheel((i+j) % 255))
            strip.show()
            time.sleep(wait_ms/1000.0)
            for i in range(0, strip.numPixels(), 3):
                strip.setPixelColor(PIXELS[i+q], 0)
    side_off()
                
def setCellColor(cell = 0, color = Color(255, 255, 255)):
    strip.setPixelColor(cell, color)
    strip.show()

def testPixelOrder():
    for i in range(0, strip.numPixels()):
        print("Cell {}".format(i))
        setCellColor(i)
        time.sleep(1)
        setCellColor(i,Color(0, 0, 0))
        
def side_on(side = 'all', color=Color(255,0,0)):
    # Turn on rows in left or right sides
    if ('right' in side) or ('all' in side):
        strip.setPixelColor(PIXELS[6], color)
        strip.setPixelColor(PIXELS[7], color)
        strip.setPixelColor(PIXELS[8], color)
        strip.setPixelColor(PIXELS[9], color)
        strip.setPixelColor(PIXELS[10], color)
        strip.setPixelColor(PIXELS[11], color)
    if ('left' in side) or ('all' in side):
        strip.setPixelColor(PIXELS[0], color)
        strip.setPixelColor(PIXELS[1], color)
        strip.setPixelColor(PIXELS[2], color)
        strip.setPixelColor(PIXELS[3], color)
        strip.setPixelColor(PIXELS[4], color)
        strip.setPixelColor(PIXELS[5], color)
    strip.show()

def side_off(side = 'all'):
    side_on(side, color=Color(0,0,0))

def flashing(side = 'all', color=Color(255,0,0), freq=5):
    side_on(side, color)
    time.sleep(0.5/freq)
    side_off(side)
    time.sleep(0.5/freq)

def flashing_thread(side, color=Color(255,0,0), freq=5):
    global flash
    flash = True
    while flash:
        flashing(side, color, freq)

def flashing_on(side='all', color=Color(255,0,0), freq=5):
    flashing_off()
    time.sleep(0.1)
    flashing_threading = thread.Thread(target=flashing_thread, args = (side, color, freq))
    flashing_threading.setDaemon(True)
    flashing_threading.start()

def flashing_off():
    global flash
    flash = False

def setup():
    global strip
    # Create NeoPixel object with appropriate configuration.
    strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
    # Intialize the library (must be called once before other functions).
    strip.begin()
    colorWipe(strip, Color(0,0,0))

def test():
    setup()
    play = True
    print('Press Ctrl-C to quit.')
    try:
        while play:
            # Color wipe animations.
            if play:
                print('colorWipe(strip, Color(255, 0, 0))')
                colorWipe(strip, Color(255, 0, 0))  # Red wipe
            if play:
                print('colorWipe(strip, Color(0, 255, 0))')
                colorWipe(strip, Color(0, 255, 0))  # Blue wipe
            if play:
                print('colorWipe(strip, Color(0, 0, 255))')
                colorWipe(strip, Color(0, 0, 255))  # Green wipe
            # Theater chase animations.
            if play:
                print('theaterChase(strip, Color(127, 127, 127))')
                theaterChase(strip, Color(127, 127, 127))  # White theater chase
            if play:
                print('theaterChase(strip, Color(127,   0,   0))')    
                theaterChase(strip, Color(127,   0,   0))  # Red theater chase
            if play:
                print('theaterChase(strip, Color(  0,   0, 127))')
                theaterChase(strip, Color(  0,   0, 127))  # Blue theater chase
            # Rainbow animations.
            if play:
                print('rainbow(strip)')
                rainbow(strip)
            if play:
                print('rainbowCycle(strip)')
                rainbowCycle(strip)
                
            if play:
                print('theaterChaseRainbow(strip')
                theaterChaseRainbow(strip)
            if play:
                print('flashing_on("left")')
                flashing_on("left")
            if play:
                time.sleep(5)
            if play:
                print('flashing_on("right")')   
                flashing_on("right")
            if play:
                time.sleep(2)
            

    except KeyboardInterrupt:
        play = False
        flashing_off()
        colorWipe(strip, Color(0,0,0))
        sound.stop()

