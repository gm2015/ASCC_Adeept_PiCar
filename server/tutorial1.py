#!/usr/bin/python3
# File name   : tutorial1.py
# Description : Example code to practice how to use functions in other modules
import time
# Import modules in Folder server
import motor
import turn
import led
import led_strip
import sound
import ultra
import findline

def test_led():    
    led.setup()
    print("both_on()")
    led.both_on()
    time.sleep(3)
    
    print("red()")
    led.red()
    time.sleep(3)
    
    print("both_off()")
    led.both_off()
    time.sleep(1)
    
    print("yellow()")
    led.yellow()
    time.sleep(3)
    
    print("pink()")
    led.pink()
    time.sleep(3)
    
    print("both_off()")
    led.both_off()
    time.sleep(1)
    
    print("cyan()")
    led.cyan()
    time.sleep(3)
    
    # Flashing in 2 cycles
    print("police_on(2)")
    led.police_on(2)
    
    print("both_off()")
    led.both_off()
    time.sleep(3)

    # Flashing in 6 seconds
    print("police_on()")    
    led.police_on()
    time.sleep(6) #6 seconds
    led.police_off()

def test_sound():
    sound.say("Hello, I am car robot. Hope you enjoy the camp!")
    time.sleep(5)
    sound.say("Let me play music for 10 seconds")
    sound.play_mp3("./music/rainbow.mp3", loops=-1)
    time.sleep(10)
    sound.stop()

def run_robot(t=0.1, t1=0.01, t2=0.01, direction=1, speed=100):
    motor.setup()    
    # Run robot forward in t seconds then backword in t seconds
    ## Forward
    motor.run_robot_t(t=t, t1=t1, t2=t2, direction=direction, speed=speed)
    time.sleep(1)
    ## Backward
    motor.run_robot_t(t=t, t1=t1, t2=t2, direction=1-direction, speed=speed)

def check_distance():
    print("Press Ctrl+C to stop")
    while True:
        t = time.time()
        dis = ultra.checkdist_inches()
        print("Time to detect:" , time.time()-t)
        print("Distance:", round(ultra.checkdist_inches(), 2))
        time.sleep(0.5)

def avoid_collision(in_range=30):
    # Write your code here
    pass

    


    




