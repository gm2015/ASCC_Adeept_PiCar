#!/usr/bin/python
# -*- coding: UTF-8 -*-
# Product     : Raspberry PiCar-B
# File name   : ASCC_client.py
# Description : client
# Author      : HaDo
# Address     : ASCC Lab - Oklahoma State University
# Date        : 2019/5/12
from socket import *
import sys
import subprocess
import time
import threading as thread
import tkinter as tk
import math
#import speech_recognition as sr
import cv2
import zmq
import base64
import numpy as np
import pygame

TEAM = "Team0"

color_bg = '#000000'        # Set background color
color_text = '#E1F5FE'      # Set text color
color_btn = '#212121'       # Set button color
color_line = '#01579B'      # Set line color
color_can = '#212121'       # Set canvas color
color_oval = '#2196F3'      # Set oval color
target_color = '#FF6D00'
color_red = '#FF0000'
color_blue = '#0000FF'
color_cyan = '#00FFFF'
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
a2t=''
TestMode = 0

stat=0          # A status value,ensure the mainloop() runs only once
tcpClicSock=''  # A global variable,for future socket connection
BUFSIZ=1024     # Set a buffer size
ip_stu=1        # Shows connection status

# Global variables of input status
c_f_stu=0
c_b_stu=0
c_l_stu=0
c_r_stu=0

b_l_stu=0
b_r_stu=0

l_stu=0
r_stu=0

BtnIP=''
ipaddr=''

led_status      = 0
opencv_status   = 0
auto_status     = 0
speech_status   = 0
findline_status = 0

ipcon=0
SR_mode=0

gear = 5
var_speed = 0.5
# Program control flags
exit_gui = False
pressed = False
# Global Variable
take_picture = False
img_id = 0
class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def print(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, [self.x, self.y])
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10

# Form objects
pygame.init()
# Used to manage how fast the screen updates
# Initialize Joystick
# Set the width and height of the screen [width,height]
size = [300, 400]
screen = pygame.display.set_mode(size)

pygame.display.set_caption("Joystick")

# Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

# Initialize the joysticks
pygame.joystick.init()

# Get ready to print
textPrint = TextPrint()

def view_joystick():
    # DRAWING STEP
    # First, clear the screen to white. Don't put other drawing commands
    # above this, or they will be erased with this command.
    screen.fill(WHITE)
    textPrint.reset()

    # Get count of joysticks
    joystick_count = pygame.joystick.get_count()

    textPrint.print(screen, "Number of joysticks: {}".format(joystick_count))
    textPrint.indent()

    # For each joystick:
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()

        textPrint.print(screen, "Joystick {}".format(i))
        textPrint.indent()

        # Get the name from the OS for the controller/joystick
        name = joystick.get_name()
        textPrint.print(screen, "Joystick name: {}".format(name))

        # Usually axis run in pairs, up/down for one, and left/right for
        # the other.
        axes = joystick.get_numaxes()
        textPrint.print(screen, "Number of axes: {}".format(axes))
        textPrint.indent()

        for i in range(axes):
            axis = joystick.get_axis(i)
            textPrint.print(screen, "Axis {} value: {:>4.1f}".format(i, axis))
            if i == 0:
                # Steering (-1 0 1)
                try:
                    tcpClicSock.send(("steering{:>4.1f}".format(axis)).encode())
                except:
                    pass
            elif i == 3:
                try:
                    # Speed
                    tcpClicSock.send(("moving{:>4.1f}".format(axis)).encode())
                except:
                    pass
        textPrint.unindent()
        buttons = joystick.get_numbuttons()
        textPrint.print(screen, "Number of buttons: {}".format(buttons))
        textPrint.indent()

        for i in range(buttons):
            button = joystick.get_button(i)  # Get value of the button #i

            textPrint.print(screen, "Button {:>2} value: {}".format(i, button))
        textPrint.unindent()
        # Hat switch. All or nothing for direction, not like joysticks.
        # Value comes back in an array.
        hats = joystick.get_numhats()
        textPrint.print(screen, "Number of hats: {}".format(hats))
        textPrint.indent()

        for i in range(hats):
            hat = joystick.get_hat(i)
            textPrint.print(screen, "Hat {} value: {}".format(i, str(hat)))
        textPrint.unindent()

        textPrint.unindent()

    # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT

    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()

def read_joystick():
    global exit_gui, pressed, take_picture, gear, var_speed
    joystick = pygame.joystick.Joystick(0)  # One joystick is connected
    joystick.init()
    while not exit_gui:
        # EVENT PROCESSING STEP
        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user clicked close
                exit_gui = True  # Flag that we are done so we exit this loop

            # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
            if event.type == pygame.JOYBUTTONDOWN:
                buttons = joystick.get_numbuttons()
                btns = []
                for i in range(buttons):
                    btns.append(joystick.get_button(i))
                if btns[0:3] == [1, 1, 1]:      #X+A+B
                    try:
                        # Turn on the LEDs in White
                        tcpClicSock.send(('led_on').encode())
                    except:
                        pass
                elif [btns[3], btns[6]] == [1, 1]:          # LT+Y
                    take_picture = True
                elif btns[2:4] == [1, 1]:                   # Y+B police mode
                    try:
                        tcpClicSock.send("police".encode())
                    except:
                        pass
                elif [btns[1], btns[7]] == [1, 1]:           #RT+A TheaterRaibow
                    try:
                        tcpClicSock.send("rainbow".encode())  #
                        print('RAINBOW')
                    except:
                        pass
                else:
                    if btns[0] == 1:                            # "X" pressed
                        try:
                            tcpClicSock.send("led_blue".encode())  # Blue Led
                        except:
                            pass
                    if btns[1] == 1:   # "A" pressed
                        try:
                            tcpClicSock.send("led_green".encode())  # Green Led
                        except:
                            pass
                    if btns[2] == 1:   # "B" pressed
                        try:
                            tcpClicSock.send("led_red".encode())  # Red Led
                        except:
                            pass
                    if btns[3] == 1:   # "Y" pressed
                        try:
                            tcpClicSock.send(('led_off').encode())
                        except:
                            pass
                    if btns[4] == 1:   # "LB" pressed
                        try:
                            tcpClicSock.send(('flash_left').encode())
                        except:
                            pass
                    if btns[5] == 1:   # "RB" pressed
                        try:
                            tcpClicSock.send(('flash_right').encode())
                        except:
                            pass
                    if btns[6] == 1:   # "LT" pressed
                        if gear >=2:
                            gear = gear - 1
                            str_speed = "gear{}".format(gear)
                            var_speed.set(gear)
                            try:
                                tcpClicSock.send(str_speed.encode())
                            except:
                                pass

                        pass
                    if btns[7] == 1:   # "RT" pressed
                        if gear <= 4:
                            gear = gear + 1
                            str_speed = "gear{}".format(gear)
                            var_speed.set(gear)
                            try:
                                tcpClicSock.send(str_speed.encode())
                            except:
                                pass

                    if btns[8] == 1:   # "Back" pressed
                        try:
                            tcpClicSock.send("ahead".encode())  # Camera ahead
                        except:
                            pass
                    if btns[9] == 1:   # "Start" pressed
                        pass

            if event.type == pygame.JOYBUTTONUP:
                pressed = False
                print("Joystick button released.")
            if event.type == 7:
                axes = joystick.get_numaxes()
                str_axis = "Axis:"
                for i in range(axes):
                    str_axis = "{}{:>4.1f}".format(str_axis, joystick.get_axis(i))
                # print(str_axis)
                try:
                    tcpClicSock.send(str_axis.encode())
                except:
                    pass

        # Head Control
        hats = joystick.get_hat(0)
        if hats[1] == 1:
            try:
                tcpClicSock.send("l_up".encode())  # Camera Looks up
            except:
                pass
        elif hats[1] == -1:
            try:
                tcpClicSock.send("l_do".encode())  # Camera Looks down
            except:
                pass
        if hats[0] == 1:
            try:
                tcpClicSock.send("l_ri".encode())  # Camera Looks up
            except:
                pass
        elif hats[0] == -1:
            try:
                tcpClicSock.send("l_le".encode())  # Camera Looks down
            except:
                pass
        # DRAWING STEP
        view_joystick()
        # Limit to 20 frames per second
        clock.tick(10)
    if exit_gui:
        pygame.quit()

# Joystick Thread
def joystick_thread():
    read_joystick()

def video_show():
    global take_picture, img_id
    while True:
        frame = footage_socket.recv_string()
        img = base64.b64decode(frame)
        npimg = np.fromstring(img, dtype=np.uint8)
        source = cv2.imdecode(npimg, 1)
        if take_picture:
            img_id = img_id + 1
            filename = "C:\\Photo\\{}_{}.png".format(TEAM, img_id)
            print(filename)
            try:
                cv2.imwrite(filename, source)
                take_picture = False
            except:
                print("Taking picture failed")
                take_picture = False
        # Show images
        cv2.imshow("Stream", source)
        cv2.waitKey(1)                

def call_scan(event):                 #When this function is called,client commands the ultrasonic to scan
    tcpClicSock.send(('scan').encode())
    print('scan')

def call_forward(event):
    #When this function is called,client commands the car to move forward
    global c_f_stu
    if c_f_stu == 0:
        tcpClicSock.send(('forward').encode())
        c_f_stu=1

def call_back(event):
    #When this function is called,client commands the car to move backward
    global c_b_stu 
    if c_b_stu == 0:
        tcpClicSock.send(('backward').encode())
        c_b_stu=1

def call_stop(event):
    #When this function is called,client commands the car to stop moving
    global c_f_stu,c_b_stu,c_l_stu,c_r_stu
    c_f_stu=0
    c_b_stu=0
    tcpClicSock.send(('stop').encode())

def call_stop_2(event):
    #When this function is called,client commands the car go straight
    global c_l_stu,c_r_stu
    c_r_stu=0
    c_l_stu=0
    tcpClicSock.send(('middle').encode())

def click_call_Left(event):
    #When this function is called,client commands the car to turn left
    tcpClicSock.send(('Left').encode())

def click_call_Right(event):
    #When this function is called,client commands the car to turn right
    tcpClicSock.send(('Right').encode())

def call_Left(event):
    #When this function is called,client commands the car to turn left
    global c_l_stu
    if c_l_stu == 0 :
        tcpClicSock.send(('Left').encode())
        c_l_stu=1

def call_Right(event):
    #When this function is called,client commands the car to turn right
    global c_r_stu
    if c_r_stu == 0 :
        tcpClicSock.send(('Right').encode())
        c_r_stu=1

def call_look_left(event):
    #Camera look left
    tcpClicSock.send(('l_le').encode())

def call_look_right(event):
    #Camera look right
    tcpClicSock.send(('l_ri').encode())

def call_look_up(event):
    #Camera look up
    tcpClicSock.send(('l_up').encode())

def call_look_down(event):
    #Camera look down
    tcpClicSock.send(('l_do').encode())

def call_ahead(event):
    #Camera look ahead
    tcpClicSock.send(('ahead').encode())
    print('ahead')

def call_auto(event):
    #When this function is called,client commands the car to start auto mode
    if auto_status == 0:
        tcpClicSock.send(('auto').encode())
    else:
        tcpClicSock.send(('Stop').encode())

def call_exit(event):
    global exit_gui
    exit_gui = True
    #When this function is called,client commands the car to shut down
    tcpClicSock.send(('exit').encode())

def call_Stop(event):
    #When this function is called,client commands the car to switch off auto mode
    tcpClicSock.send(('Stop').encode())

def scan(event):
    #When this function is called,client commands the ultrasonic to scan
    tcpClicSock.send(('scan').encode())
    print('scan')

def find_line(event):
    #Line follow mode
    if findline_status == 0:
        tcpClicSock.send(('findline').encode())
    else:
        tcpClicSock.send(('Stop').encode())

def replace_num(initial,new_num):
    #Call this function to replace data in '.txt' file
    newline=""
    str_num=str(new_num)
    with open("ip.txt","r") as f:
        for line in f.readlines():
            if(line.find(initial) == 0):
                line = initial+"%s" %(str_num)
            newline += line
    with open("ip.txt","w") as f:
        f.writelines(newline)
        #Call this function to replace data in '.txt' file

def num_import(initial):
    #Call this function to import data from '.txt' file
    with open("ip.txt") as f:
        for line in f.readlines():
            if(line.find(initial) == 0):
                r=line
    begin=len(list(initial))
    snum=r[begin:]
    n=snum
    return n    

def lights_ON(event):
    #Turn on the LEDs
    if led_status == 0:
        tcpClicSock.send(('lightsON').encode())
    else:
        tcpClicSock.send(('lightsOFF').encode())

def call_SR3():
    #Start speech recognition mode
    if speech_status == 0:
        tcpClicSock.send(('voice_3').encode())
    else:
        tcpClicSock.send(('Stop').encode())

def call_opencv():
    #Start OpenCV mode
    if opencv_status == 0:
        tcpClicSock.send(('opencv').encode())
    else:
        tcpClicSock.send(('Stop').encode())

def voice_input():
    global a2t
    r = sr.Recognizer()
    with sr.Microphone() as source:
        #r.adjust_for_ambient_noise(source)
        r.record(source,duration=2)
        print("Say something!")
        audio = r.listen(source)
    try:
        a2t=r.recognize_sphinx(audio,keyword_entries=[('forward',1.0),('backward',1.0),('left',1.0),('right',1.0),('stop',1.0),('find line',0.95),('follow',1),('lights on',1),('lights off',1)])
        print("Sphinx thinks you said " + a2t)
    except sr.UnknownValueError:
        print("Sphinx could not understand audio")
    except sr.RequestError as e:
        print("Sphinx error; {0}".format(e))
    BtnVIN.config(fg=color_text,bg=color_btn)
    return a2t

def voice_command_thread():
    while 1:
        if SR_mode == 1:
            l_VIN.config(text='Command?')
            v_command=voice_input()
            if SR_mode == 1:
                l_VIN.config(text='%s'%v_command)
                if 'forward' in v_command:
                    tcpClicSock.send(('forward').encode())
                elif 'backward' in v_command:
                    tcpClicSock.send(('backward').encode())
                elif 'left' in v_command:
                    tcpClicSock.send(('Left').encode())
                elif 'right' in v_command:
                    tcpClicSock.send(('Right').encode())
                elif 'stop' in v_command:
                    tcpClicSock.send(('stop').encode())
                    tcpClicSock.send(('Stop').encode())
                elif 'find line' in v_command:
                    tcpClicSock.send(('findline').encode())
                elif 'follow' in v_command:
                    tcpClicSock.send(('auto').encode())
                elif 'lights on' in v_command:
                    tcpClicSock.send(('lightsON').encode())
                elif 'lights off' in v_command:
                    tcpClicSock.send(('lightsOFF').encode())
                else:
                    pass
            else:
                pass
        else:
            time.sleep(0.2)

def voice_command(event):
    global SR_mode
    if SR_mode == 0:
        SR_mode = 1
        BtnVIN.config(fg='#0277BD',bg='#BBDEFB')
    else:
        BtnVIN.config(fg=color_text,bg=color_btn)
        SR_mode = 0

def draw_car_kinematic(canvas_car, car_pose, head_pose, leds):
    canvas_car.create()


def loop():
    #GUI
    global tcpClicSock, BtnIP, led_status, BtnVIN, l_VIN, TestMode, var_speed
    #The value of tcpClicSock changes in the function loop(),would also changes in global so the other functions could use it.
    if True:
        # Callback Functions #####################################
        def connect(event):  # Call this function to connect with the server
            if ip_stu == 1:
                sc = thread.Thread(target=socket_connect)  # Define a thread for connection
                sc.setDaemon(True)  # 'True' means it is a front thread,it would close when the mainloop() closes
                sc.start()  # Thread starts

        def connect_2():  # Call this function to connect with the server
            if ip_stu == 1:
                sc = thread.Thread(target=socket_connect)  # Define a thread for connection
                sc.setDaemon(True)  # 'True' means it is a front thread,it would close when the mainloop() closes
                sc.start()  # Thread starts

        def socket_connect():  # Call this function to connect with the server
            global ADDR, tcpClicSock, BUFSIZ, ip_stu, ipaddr
            ip_adr = ip_entry.get()  # Get the IP address from Entry

            if ip_adr == '':  # If no input IP address in Entry,import a default IP
                ip_adr = num_import('IP:')
                ip_l_status.config(text='Connecting')
                ip_l_status.config(bg='#FF8F00')
                ip_l_status.config(text='Default:%s' % ip_adr)
                pass

            SERVER_IP = ip_adr
            SERVER_PORT = 10223  # Define port serial
            BUFSIZ = 1024  # Define buffer size
            ADDR = (SERVER_IP, SERVER_PORT)
            tcpClicSock = socket(AF_INET, SOCK_STREAM)  # Set connection value for socket

            for i in range(1, 6):  # Try 5 times if disconnected
                try:
                    if ip_stu == 1:
                        print("Connecting to server @ %s:%d..." % (SERVER_IP, SERVER_PORT))
                        print("Connecting")
                        tcpClicSock.connect(ADDR)  # Connection with the server

                        print("Connected")

                        ip_l_status.config(text='Connected')
                        ip_l_status.config(bg='#558B2F')

                        replace_num('IP:', ip_adr)
                        ip_btn.config(state='disabled')  # Disable the Entry
                        ip_entry.config(state='disabled')
                        ip_stu = 0  # '0' means connected

                        at = thread.Thread(target=code_receive)  # Define a thread for data receiving
                        at.setDaemon(
                            True)  # 'True' means it is a front thread,it would close when the mainloop() closes
                        at.start()  # Thread starts

                        SR_threading = thread.Thread(
                            target=voice_command_thread)  # Define a thread for ultrasonic tracking
                        SR_threading.setDaemon(
                            True)  # 'True' means it is a front thread,it would close when the mainloop() closes
                        SR_threading.start()  # Thread starts

                        video_thread = thread.Thread(target=video_show)  # Define a thread for data receiving
                        video_thread.setDaemon(
                            True)  # 'True' means it is a front thread,it would close when the mainloop() closes
                        print('Video Connected')
                        video_thread.start()  # Thread starts

                        ipaddr = tcpClicSock.getsockname()[0]
                        break
                    else:
                        break
                except Exception:
                    print("Cannot connecting to server,try it latter!")
                    ip_l_status.config(text='Try %d/5 time(s)' % i)
                    ip_l_status.config(bg='#EF6C00')
                    print('Try %d/5 time(s)' % i)
                    ip_stu = 1
                    time.sleep(1)
                    continue
            if ip_stu == 1:
                ip_l_status.config(text='Disconnected')
                ip_l_status.config(bg='#F44336')

        def code_receive():  # A function for data receiving
            global var_speed, led_status, ipcon, findline_status, auto_status, opencv_status, speech_status, TestMode
            while True:
                code_car = tcpClicSock.recv(BUFSIZ)  # Listening,and save the data in 'code_car'
                l_car_status.config(text=code_car)  # Put the data on the label
                # print(code_car)
                if not code_car:
                    continue
                elif 'SET' in str(code_car):
                    print('set get')
                    set_list = code_car.decode()
                    set_list = set_list.split()
                    s1, s2, s3, s4, s5, s6 = set_list[1:]
                    E_C1.delete(0, 50)
                    E_C2.delete(0, 50)
                    E_M1.delete(0, 50)
                    E_M2.delete(0, 50)
                    E_T1.delete(0, 50)
                    E_T2.delete(0, 50)

                    E_C1.insert(0, '%d' % int(s1))
                    E_C2.insert(0, '%d' % int(s2))
                    E_M1.insert(0, '%d' % int(s3))
                    E_M2.insert(0, '%d' % int(s4))
                    E_T1.insert(0, '%d' % int(s5))
                    E_T2.insert(0, '%d' % int(s6))

                elif 'list' in str(code_car):  # Scan result receiving start
                    dis_list = []
                    f_list = []
                    list_str = code_car.decode()

                    while True:  # Save scan result in dis_list
                        code_car = tcpClicSock.recv(BUFSIZ)
                        if 'finished' in str(code_car):
                            break
                        list_str += code_car.decode()
                        l_car_status.config(text='Scanning')

                    dis_list = list_str.split()  # Save the data as a list
                    l_car_status.config(text='Finished')

                    for i in range(0, len(dis_list)):  # Translate the String-type value in the list to Float-type
                        try:
                            new_f = float(dis_list[i])
                            f_list.append(new_f)
                        except:
                            continue

                    dis_list = f_list
                    # can_scan.delete(line)
                    # can_scan.delete(point_scan)
                    can_scan_1 = tk.Canvas(root, bg=color_can, height=250, width=320,
                                           highlightthickness=0)  # define a canvas
                    can_scan_1.place(x=440, y=330)  # Place the canvas
                    line = can_scan_1.create_line(0, 62, 320, 62, fill='darkgray')  # Draw a line on canvas
                    line = can_scan_1.create_line(0, 124, 320, 124, fill='darkgray')  # Draw a line on canvas
                    line = can_scan_1.create_line(0, 186, 320, 186, fill='darkgray')  # Draw a line on canvas
                    line = can_scan_1.create_line(160, 0, 160, 250, fill='darkgray')  # Draw a line on canvas
                    line = can_scan_1.create_line(80, 0, 80, 250, fill='darkgray')  # Draw a line on canvas
                    line = can_scan_1.create_line(240, 0, 240, 250, fill='darkgray')  # Draw a line on canvas

                    x_range = var_x_scan.get()  # Get the value of scan range from IntVar

                    for i in range(0, len(dis_list)):  # Scale the result to the size as canvas
                        try:
                            len_dis_1 = int((dis_list[i] / x_range) * 250)  # 600 is the height of canvas
                            pos = int((i / len(dis_list)) * 320)  # 740 is the width of canvas
                            pos_ra = int(((i / len(dis_list)) * 140) + 20)  # Scale the direction range to (20-160)
                            len_dis = int(
                                len_dis_1 * (math.sin(math.radians(pos_ra))))  # len_dis is the height of the line

                            x0_l, y0_l, x1_l, y1_l = pos, (250 - len_dis), pos, (250 - len_dis)  # The position of line
                            x0, y0, x1, y1 = (pos + 3), (250 - len_dis + 3), (pos - 3), (
                                    250 - len_dis - 3)  # The position of arc

                            if pos <= 160:  # Scale the whole picture to a shape of sector
                                pos = 160 - abs(int(len_dis_1 * (math.cos(math.radians(pos_ra)))))
                                x1_l = (x1_l - math.cos(math.radians(pos_ra)) * 130)
                            else:
                                pos = abs(int(len_dis_1 * (math.cos(math.radians(pos_ra))))) + 160
                                x1_l = x1_l + abs(math.cos(math.radians(pos_ra)) * 130)

                            y1_l = y1_l - abs(math.sin(math.radians(pos_ra)) * 130)  # Orientation of line

                            line = can_scan_1.create_line(pos, y0_l, x1_l, y1_l,
                                                          fill=color_line)  # Draw a line on canvas
                            point_scan = can_scan_1.create_oval((pos + 3), y0, (pos - 3), y1, fill=color_oval,
                                                                outline=color_oval)  # Draw a arc on canvas
                        except:
                            pass
                    can_tex_11 = can_scan_1.create_text((27, 178), text='%sm' % round((x_range / 4), 2),
                                                        fill='#aeea00')  # Create a text on canvas
                    can_tex_12 = can_scan_1.create_text((27, 116), text='%sm' % round((x_range / 2), 2),
                                                        fill='#aeea00')  # Create a text on canvas
                    can_tex_13 = can_scan_1.create_text((27, 54), text='%sm' % round((x_range * 0.75), 2),
                                                        fill='#aeea00')  # Create a text on canvas

                elif '1' in str(code_car):  # Translate the code to text
                    l_car_status.config(text='Moving Forward')  # Put the text on the label
                elif '2' in str(code_car):  # Translate the code to text
                    l_car_status.config(text='Moving Backward')  # Put the text on the label
                elif '3' in str(code_car):  # Translate the code to text
                    l_car_status.config(text='Turning Left')  # Put the text on the label
                elif '4' in str(code_car):  # Translate the code to text
                    l_car_status.config(text='Turning Right')  # Put the text on the label
                elif '5' in str(code_car):  # Translate the code to text
                    l_car_status.config(text='Look Up')  # Put the text on the label
                elif '6' in str(code_car):  # Translate the code to text
                    l_car_status.config(text='Look Down')  # Put the text on the label
                elif '7' in str(code_car):  # Translate the code to text
                    l_car_status.config(text='Look Left')  # Put the text on the label
                elif '8' in str(code_car):  # Translate the code to text
                    l_car_status.config(text='Look Right')  # Put the text on the label
                elif '9' in str(code_car):  # Translate the code to text
                    l_car_status.config(text='Stop')  # Put the text on the label

                elif '0' in str(code_car):  # Translate the code to text
                    l_car_status.config(text='Follow Mode On')  # Put the text on the label
                    Btn5.config(text='Following', fg='#0277BD', bg='#BBDEFB')
                    auto_status = 1

                elif 'findline' in str(code_car):  # Translate the code to text
                    BtnFL.config(text='Finding', fg='#0277BD', bg='#BBDEFB')
                    l_car_status.config(text='Find Line')
                    findline_status = 1

                elif 'lightsON' in str(code_car):  # Translate the code to text
                    BtnLED.config(text='Lights ON', fg='#0277BD', bg='#BBDEFB')
                    led_status = 1
                    l_car_status.config(text='Lights On')  # Put the text on the label

                elif 'lightsOFF' in str(code_car):  # Translate the code to text
                    BtnLED.config(text='Lights OFF', fg=color_text, bg=color_btn)
                    led_status = 0
                    l_car_status.config(text='Lights OFF')  # Put the text on the label

                elif 'oncvon' in str(code_car):
                    if TestMode == 0:
                        BtnOCV.config(text='OpenCV ON', fg='#0277BD', bg='#BBDEFB')
                        BtnFL.config(text='Find Line', fg=color_text, bg=color_btn)
                        l_car_status.config(text='OpenCV ON')
                        opencv_status = 1

                elif 'auto_status_off' in str(code_car):
                    if TestMode == 0:
                        BtnSR3.config(fg=color_text, bg=color_btn, state='normal')
                        BtnOCV.config(text='OpenCV', fg=color_text, bg=color_btn, state='normal')
                    BtnFL.config(text='Find Line', fg=color_text, bg=color_btn)
                    Btn5.config(text='Follow', fg=color_text, bg=color_btn, state='normal')
                    findline_status = 0
                    speech_status = 0
                    opencv_status = 0
                    auto_status = 0

                elif 'voice_3' in str(code_car):
                    BtnSR3.config(fg='#0277BD', bg='#BBDEFB')
                    # BtnSR1.config(state='disabled')
                    # BtnSR2.config(state='disabled')
                    l_car_status.config(text='Sphinx SR')  # Put the text on the label
                    speech_status = 1

                elif 'TestVersion' in str(code_car):
                    TestMode = 1
                    BtnSR3.config(fg='#FFFFFF', bg='#F44336')
                    BtnOCV.config(fg='#FFFFFF', bg='#F44336')
            ##############################################################################
        # Create Form
        root = tk.Tk()            #Define a window named root
        root.title('Adeept / Modified by ASCC Lab@Okstate')      #Main window title
        #root.geometry('600x630')  #Main window size, middle of the English letter x.
        width = 620
        height = 650
        x0 = 20
        y0 = 10
        root.geometry('%dx%d+%d+%d' % (width, height, x0, y0))
        root.config(bg=color_bg)  #Set the background color of root window

        var_spd = tk.StringVar()  #Speed value saved in a StringVar
        var_spd.set(1)            #Set a default speed,but change it would not change the default speed value in the car,you need to click button'Set' to send the value to the car
        global var_speed
        var_speed = tk.IntVar()
        var_speed.set(gear)

        var_x_scan = tk.IntVar()  #Scan range value saved in a IntVar
        var_x_scan.set(2)         #Set a default scan value


        logo =tk.PhotoImage(file='logo.png')         #Define the picture of logo,but only supports '.png' and '.gif'
        l_logo=tk.Label(root, image=logo, bg=color_bg) #Set a label to show the logo picture
        l_logo.place(x=1, y=1)                        #Place the Label in a right position

        # IP selection
        ip_l_add = tk.Label(root, width=10, text='IP Address:', fg=color_text, bg='#000000')
        ip_l_add.place(x=100, y=5)  # Define a Label and put it in position

        ip_entry = tk.Entry(root, show=None, width=16, bg="#37474F", fg='#eceff1')
        ip_entry.insert(0, "192.168.1.128")
        ip_entry.place(x=180, y=8)  # Define a Entry and put it in position

        ip_btn = tk.Button(root, width=8, text='Connect', fg=color_text, bg=color_btn, command=connect_2, relief='ridge')
        ip_btn.place(x=290, y=5)

        ip_l_status = tk.Label(root, width=12, text='Disconnected', fg=color_text, bg='#F44336')
        ip_l_status.place(x=365, y=8)

        # Status
        l_car_status = tk.Label(root, width=30, text='Status', fg=color_text, bg=color_btn)
        l_car_status.place(x=120, y=35)

        # Exit Btn
        btn_exit = tk.Button(root, width=6, text='Exit', fg=color_text, bg=color_btn, relief='ridge')
        btn_exit.place(x=480, y=5)
        btn_exit.bind('<ButtonPress-1>', call_exit)

        # car Kinematics
        l_car_kine = tk.Label(root, width=15, text='Car Robot', fg=color_text, bg='#000000')
        l_car_kine.place(x=80, y=60)  # Define a Label and put it in position
        can_car_kine = tk.Canvas(root, bg=color_can, height=330, width=250, highlightthickness=1)  # define a canvas
        can_car_kine.place(x=10, y=80)  # Place the canvas
        # Robot Photo
        robot_photo = tk.PhotoImage(file='robot.png')
        l_photo = tk.Label(root, image=robot_photo, bg=color_text)
        l_photo.place(x=20, y=80)
        ## Robot Gear
        scale_speed = tk.Scale(root, label="               < Slow   Speed Adjustment   Fast >",
                               from_=1, to=5, orient=tk.HORIZONTAL, length=250, showvalue=1, tickinterval=1,
                               resolution=1, variable=var_speed, fg=color_text, bg=color_bg, highlightthickness=0)
        scale_speed.place(x=10, y=420)  # Define a Scale and put it in position

        # Video View and Ultrasonic Scan
        video_view = tk.BooleanVar()
        video_view.set(True)
        checkbox_video = tk.Checkbutton(root, text=' Video', variable=video_view, fg=color_blue, bg=color_btn)
        checkbox_video.place(x=550, y=20)
        auto_scan = tk.BooleanVar()
        auto_scan.set(False)
        checkbox_scan = tk.Checkbutton(root, text=' Scan  ', variable=auto_scan, fg=color_blue, bg=color_btn)
        checkbox_scan.place(x=550, y=55)
        l_scan = tk.Label(root, width=20, text='Ultrasonic Scan', fg=color_text, bg='#000000')
        l_scan.place(x=360, y=60)  # Define a Label and put it in position
        scale_scan = tk.Scale(root, label="     < Near   Scan Range Adjustment(Meter(s))   Far >",
                      from_=1, to=5, orient=tk.HORIZONTAL, length=320,
                      showvalue=1, tickinterval=1, resolution=1, variable=var_x_scan, fg=color_text, bg=color_bg,
                      highlightthickness=0)
        scale_scan.place(x=280, y=80)
        can_scan = tk.Canvas(root, bg=color_can, height=250, width=320, highlightthickness=1)  # define a canvas
        can_scan.place(x=280, y=160)  # Place the canvas
        line = can_scan.create_line(0, 62, 320, 62, fill='darkgray')  # Draw a line on canvas
        line = can_scan.create_line(0, 124, 320, 124, fill='darkgray')  # Draw a line on canvas
        line = can_scan.create_line(0, 186, 320, 186, fill='darkgray')  # Draw a line on canvas
        line = can_scan.create_line(160, 0, 160, 250, fill='darkgray')  # Draw a line on canvas
        line = can_scan.create_line(80, 0, 80, 250, fill='darkgray')  # Draw a line on canvas
        line = can_scan.create_line(240, 0, 240, 250, fill='darkgray')  # Draw a line on canvas
        x_range = var_x_scan.get()
        can_tex_11 = can_scan.create_text((27, 178), text='%sm' % round((x_range / 4), 2),
                                          fill='#aeea00')  # Create a text on canvas
        can_tex_12 = can_scan.create_text((27, 116), text='%sm' % round((x_range / 2), 2),
                                          fill='#aeea00')  # Create a text on canvas
        can_tex_13 = can_scan.create_text((27, 54), text='%sm' % round((x_range * 0.75), 2),
                                          fill='#aeea00')  # Create a text on canvas

        global stat
        if stat == 0:              # Ensure the mainloop runs only once
                root.mainloop()  # Run the mainloop()
                stat = 1           # Change the value to '1' so the mainloop() would not run again.

if __name__ == '__main__':
    opencv_socket = socket()
    opencv_socket.bind(('0.0.0.0', 8080))
    opencv_socket.listen(0)

    context = zmq.Context()
    footage_socket = context.socket(zmq.SUB)
    footage_socket.bind('tcp://*:5555')
    footage_socket.setsockopt_string(zmq.SUBSCRIBE, np.unicode(''))

    try:
        # Threads start
        joystick_thread = thread.Thread(target=joystick_thread)  # Define a thread for FPV and OpenCV
        joystick_thread.setDaemon(True)  # 'True' means it is a front thread,it would close when the mainloop() closes
        joystick_thread.start()
        loop()                   # Load GUI
    except KeyboardInterrupt:
        tcpClicSock = socket(AF_INET, SOCK_STREAM) # Define socket for future socket-closing operation
        cv2.destroyAllWindows()
    tcpClicSock.close()          # Close socket or it may not connect with the server again
    cv2.destroyAllWindows()
