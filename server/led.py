#!/usr/bin/python3
# File name   : motor.py
# Description : Control LEDs 
# Website     : www.adeept.com
# E-mail      : support@adeept.com
# Author      : William
# Date        : 2018/10/12
# Modified    : 2019/5/20
# by HaDo
import RPi.GPIO as GPIO
import time
import threading as thread
import sound
import os

path = os.path.dirname(os.path.abspath(__file__))

left_R = 15
left_G = 16
left_B = 18

right_R = 19
right_G = 21
right_B = 22

on  = GPIO.LOW
off = GPIO.HIGH

done = False

def setup():
    #initialization
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(left_R, GPIO.OUT)
    GPIO.setup(left_G, GPIO.OUT)
    GPIO.setup(left_B, GPIO.OUT)
    GPIO.setup(right_R, GPIO.OUT)
    GPIO.setup(right_G, GPIO.OUT)
    GPIO.setup(right_B, GPIO.OUT)
    both_off()

def both_on():
    # Turn off both LED (White Color)
    GPIO.output(left_R, on)
    GPIO.output(left_G, on)
    GPIO.output(left_B, on)

    GPIO.output(right_R, on)
    GPIO.output(right_G, on)
    GPIO.output(right_B, on)
    
def both_off():
    # Turn off both LED
    GPIO.output(left_R, off)
    GPIO.output(left_G, off)
    GPIO.output(left_B, off)

    GPIO.output(right_R, off)
    GPIO.output(right_G, off)
    GPIO.output(right_B, off)

def side_on(side_X):
    # Turn of the side_X Led
    GPIO.output(side_X, on)

def side_off(side_X):
    # Turn of the side_X Led
    GPIO.output(side_X, off)
    
def side_color_on(side_X,side_Y):
    GPIO.output(side_X, on)
    GPIO.output(side_Y, on)

def side_color_off(side_X,side_Y):
    GPIO.output(side_X, off)
    GPIO.output(side_Y, off)

def flashing(): # Police flashing in 2 seconds
    for i in range (1,3):
        side_on(left_R)
        side_on(right_B)
        time.sleep(0.1)
        both_off()
        side_on(left_B)
        side_on(right_R)
        time.sleep(0.1)
        both_off()
    for i in range (1,4):
        side_on(left_R)
        side_on(right_B)
        time.sleep(0.3)
        both_off()
        side_on(left_B)
        side_on(right_R)
        time.sleep(0.3)
        both_off()
            
def police_thread(sound_on=True):
    global done
    done = False
    if sound_on:
        sound.play_mp3('{}/music/police.mp3'.format(path), loops = -1)
    while not done:
        flashing()
    sound.stop()     
        
def police_on(times = -1, sound_on=True):
    # Turn on the police flashing mode in times (2 x times [s])    
    # If times = -1, flashing until police_off() is called.
    if times == 0:
        both_off()
    elif times > 0:
        if sound_on:
            sound.play_mp3('{}/music/police.mp3'.format(path), loops = -1)
        for i in range (0, int(times)):
            flashing()
        sound.stop()
    else:        
        police_threading = thread.Thread(target=police_thread, args=[sound_on])
        police_threading.setDaemon(True)
        police_threading.start()
    
def police(times=3):
    # Flashing in times cycles
    for i in range (times):
        flashing()

def police_off():
    # Turn off the poice flashing mode
    global done
    done = True 
    
def red():
    side_on(right_R)
    side_on(left_R)

def green():
    side_on(right_G)
    side_on(left_G)

def blue():
    side_on(right_B)
    side_on(left_B)

def yellow():
    red()
    green()    

def pink():
    red()
    blue()

def cyan():
    blue()
    green()

def flash_left(times=3):
    # Flashing left LED in times cycles
    for i in range(0,times):
        both_off()
        side_on(left_G)
        side_on(left_R)
        time.sleep(0.5)
        both_off()
        time.sleep(0.5)

def flash_right(times=3):
    # Flashing right LED in times cycles
    for i in range(1,times):
        both_off()
        side_on(right_G)
        side_on(right_R)
        time.sleep(0.5)
        both_off()
        time.sleep(0.5)

def test():
    setup()
    print("both_on()")
    both_on()
    time.sleep(3)
    
    print("red()")
    red()
    time.sleep(3)
    
    print("both_off()")
    both_off()
    time.sleep(1)
    
    print("yellow()")
    yellow()
    time.sleep(3)
    
    print("pink()")
    pink()
    time.sleep(3)
    
    print("both_off()")
    both_off()
    time.sleep(1)
    
    print("cyan()")
    cyan()
    time.sleep(3)
    
    # Flashing in 2 cycles
    print("police_on(2)")
    police_on(2)
    
    print("both_off()")
    both_off()
    time.sleep(3)

    # Flashing in 6 seconds
    print("police_on()")    
    police_on()
    time.sleep(6) #6 seconds
    police_off()

