__author__ = 'anton'

#!/usr/bin/env python
# ##############################################################################################################
# Program Name: plotter
# ================================
# This code is for controlling a plotter by a web browser using web sockets
# History
# ------------------------------------------------
# Author
# Anton Vanhoucke
#
# These files have been made available online through a Creative Commons Attribution-ShareAlike 3.0  license.
# (http://creativecommons.org/licenses/by-sa/3.0/)
#
###############################################################################################################

# CONNECTIONS-
#   Pen Motor - Port A
# 	Left Motor  - Port B
# 	Right Motor - Port C
#
# PREREQUISITES
#	Tornado Web Server for Python
#
# TROUBLESHOOTING:
#	Don't use Ctrl+Z to stop the program, use Ctrl+c.
#	If you use Ctrl+Z, it will not close the socket and you won't be able to run the program the next time.
#	If you get the following error:
#		"socket.error: [Errno 98] Address already in use "
#	Run this on the terminal:
#		"sudo netstat -ap |grep :9093"
#	Note down the PID of the process running it
#	And kill that process using:
#		"kill pid"
#	If it does not work use:
#		"kill -9 pid"
#	If the error does not go away, try changin the port number '9093' both in the client and server code


####################### Imports #########################
# For passing arguments to the script from the command line
import sys

import time

# To run motors on the brickpi, in a separate thread
from BrickPi import *  # import BrickPi.py file to use BrickPi operations
import threading
import subprocess
import tornado.ioloop
import tornado.web
import tornado.websocket
import tornado.template
from brickpi_helpers import *

################### Constants & settings ################


MOTOR_CMD_RATE = 20  # Max number of motor commands per second
L_ROPE_0 = 63  # Length of left rope in cm when pen is at 0,0 (top left)
R_ROPE_0 = 95  # same for right tope
ROPE_ATTACHMENT_WIDTH = 90  # space between the two attachment points of the plotter.In my case: door width. In cm.
PULLEY_DIAMETER = 4.4

# now some math
CM_TO_DEG = 180 / 3.1415 * 2 / PULLEY_DIAMETER * 24 / 8  #angle-in-deg = l-in-cm/(diameter/2) * 360 /(2*PI) * num_teeth_large_gear / num_teeth_small_gear
v_margin = triangle_area(L_ROPE_0, R_ROPE_0, ROPE_ATTACHMENT_WIDTH) / ROPE_ATTACHMENT_WIDTH * 2  #height of triangle
h_margin = (L_ROPE_0 ** 2 - v_margin ** 2) ** 0.5  #pythagoras to find distance from triangle point to left doorframe
canvas_size = ROPE_ATTACHMENT_WIDTH - 2 * h_margin


c = 0 # movement command



################# Movement functions ######################

def motor_targets_from_coords(x, y):
    l_rope = (x ** 2 + y ** 2) ** 0.5
    r_rope = ((ROPE_ATTACHMENT_WIDTH - x) ** 2 + y ** 2) ** 0.5

    l_target = (l_rope - L_ROPE_0) * CM_TO_DEG
    r_target = (r_rope - L_ROPE_0) * CM_TO_DEG

    return l_target, r_target

def pen_up():
    BrickPi.MotorSpeed[PORT_A] = 30
    BrickPiUpdateValues()
    time.sleep(0.3)
    BrickPi.MotorSpeed[PORT_A] = 0
    BrickPiUpdateValues()


def pen_down():
    BrickPi.MotorSpeed[PORT_A] = -30
    BrickPiUpdateValues()
    time.sleep(0.3)
    BrickPi.MotorSpeed[PORT_A] = 0
    BrickPiUpdateValues()


def set_origin():
    global drive_motors
    for motor in drive_motors:
        motor.zero = int(BrickPi.Encoder[motor.port])
        print "Encoder zero position set to", motor.zero, "For motor at port:", motor.port


def move_to_norm_coord(x_norm, y_norm):
    # convert normalized coordinates to global coordinates
    x = x_norm * canvas_size + h_margin
    y = y_norm * canvas_size + v_margin

    # set targets
    motor_B.target, motor_C.target = motor_targets_from_coords(x, y)

    # wait until targets are reached
    while 1:
        for motor in drive_motors:
            #get motor positions
            motor.position = BrickPi.Encoder[motor.port]
            #set motor speed accordingly
            BrickPi.MotorSpeed[motor.port] = motor.get_power()

        #We're done calculating and setting all motor speeds!
        BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors
        time.sleep(0.02)

        if (motor_C.error < 2) and (motor_B.error < 2):
            #we got where we want to be. Time for the next coordinate.
            break


def plot_from_file(filename):
    coords = open(filename)
    num_coords = int(coords.readline())  #coords contains it's length on the first line.

    #drive to the first coordinate
    pen_up()

    pen_down()
    for i in range(num_coords - 1):
        # read from file
        x_norm, y_norm = [float(n) for n in coords.readline().split(",")]

        #move
        move_to_norm_coord(x_norm, y_norm)

    coords.close()


def plot_circles(step=0.1):

    #draw circles form the left side until we reach the bottom
    for i in range(0,1,step*2):
        move_to_norm_coord(0,step)

        #turn on right motor, slowly
        BrickPi.MotorSpeed[PORT_C] = 100

        #calculate coordinates continuously until we reach the top, or right side of the canvas

        #move to the next circle

        #turn on right motor, slowly backwards

        #calculate coordinates continuously until we reach the left, or bottom side of the canvas

    #now draw circles from the bottom



################# Set up web server & threads #####################


# Initialize TOrnado to use 'GET' and load index.html
class MainHandler(tornado.web.RequestHandler):
    def get(self):
        loader = tornado.template.Loader(".")
        self.write(loader.load("index.html").generate())

#Code for handling the data sent from the webpage
class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        global websockets
        if self not in websockets:
            websockets.append(self)
        print 'connection opened...'

    def check_origin(self, origin):
        return True

    def on_message(self, message):  # receives the data from the webpage and is stored in the variable message
        global c
        print 'received:', message  # prints the revived from the webpage
        c = message
        print c

    def on_close(self):
        global websockets
        if self in websockets:
            websockets.remove(self)
        print 'connection closed...'

def wsSend(message):
    for ws in websockets:
        ws.write_message(message)


application = tornado.web.Application([
    (r'/ws', WSHandler),
    (r'/', MainHandler),
    (r"/(.*)", tornado.web.StaticFileHandler, {"path": "./resources"}),
])


class MotorThread(threading.Thread):
    def __init__(self, threadID, name, counter):
        self.motor_log = Logger("Motors")
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter

    def run(self):
        global c
        print "Starting motor thread"
        while running:
            try:
                if c == 'u':
                    print "Running left motor fwd"
                    BrickPi.MotorSpeed[PORT_B] = 100  #Set the speed of MotorA (-255 to 255)

                elif c == 'd':
                    print "Running left motor back"
                    BrickPi.MotorSpeed[PORT_B] = -100
                elif c == 'r':
                    print "Running right motor forward"
                    BrickPi.MotorSpeed[PORT_C] = 100
                elif c == 'l':
                    print "Running right motor back"
                    BrickPi.MotorSpeed[PORT_C] = 100
                elif c == 'b':
                    print "Stopped"
                    wsSend("Stopped")
                    BrickPi.MotorSpeed[PORT_B] = 0
                    BrickPi.MotorSpeed[PORT_C] = 0
                elif c == 'p':
                    plot_from_file('coords.csv')
                elif c == 'o':
                    set_origin()

                BrickPiUpdateValues();  # BrickPi updates the values for the motors
                time.sleep(0.05)
                #print "Values Updated"


            except KeyboardInterrupt:  #Triggered by pressing Ctrl+C. Time to clean up.

                #Shutting down all motors.
                BrickPi.MotorSpeed[PORT_A] = 0
                BrickPi.MotorSpeed[PORT_B] = 0
                BrickPi.MotorSpeed[PORT_C] = 0
                BrickPi.MotorSpeed[PORT_D] = 0
                BrickPiUpdateValues()

                print "Motor thread stopped"
                break  #Exit


################## Main #############################

if __name__ == "__main__":
    # Set up logging
    server_log = Logger("Server")

    #  Setup BrickPi and motors
    server_log.log("Revving up engines")
    BrickPiSetup()  # setup the serial port for communication
    BrickPi.MotorEnable[PORT_A] = 1  #Enable the Motor A
    BrickPi.MotorEnable[PORT_B] = 1  #Enable the Motor B
    BrickPi.MotorEnable[PORT_C] = 1  #Enable the Motor C
    BrickPi.MotorEnable[PORT_D] = 1  #Enable the Motor D

    motor_B = motorPID_control(PORT_B)
    motor_C = motorPID_control(PORT_C)
    drive_motors = [motor_B, motor_C]

    no_values = 1
    while no_values:
        # Make sure we have something before we start running
        # So we wait until no_values goes 0, which means values updated OK
        no_values = BrickPiUpdateValues()

    running = True

    thread1 = MotorThread(1, "Thread-1", 1)
    thread1.setDaemon(True)
    thread1.start()

    #set up web server
    websockets = []
    application.listen(9093)  # starts the websockets connection
    server_log.newline()  #done setting up. Log it.
    tornado.ioloop.IOLoop.instance().start()