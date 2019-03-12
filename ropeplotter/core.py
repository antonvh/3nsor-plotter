__author__ = 'anton'

import time
import ev3dev.auto as ev3
import math
from PIL import Image, ImageFilter
from ropeplotter.robot_helpers import PIDMotor, clamp, BrickPiPowerSupply
import logging

plotter_log = logging.getLogger("Plotter")

PEN_UP_POS = 0
PEN_DOWN_POS = -90
UP = 0
DOWN = 1
UNCHANGED = -1
SLOW = 600 # 320
FAST = 600 # 520

class RopePlotter(object):
    def __init__(self, l_rope_0, r_rope_0, attachment_distance, cm_to_deg=-175, Kp=2.2, Ki=0.2, Kd=0.02, chalk=False):

        self.cm_to_deg = cm_to_deg
        self.__l_rope_0 = float(l_rope_0)
        self.__r_rope_0 = float(r_rope_0)
        self.__att_dist = float(attachment_distance)
        self.direction = 1 # -1 is for reversing motors
        self.calc_constants()
        self.scanlines = 100
        self.r_step = 2.0 # cm

        # Start the engines
        self.pen_motor = PIDMotor(ev3.OUTPUT_A, Kp=2, Ki=0.1, Kd=0, brake=0.1, speed_reg=True)
        self.pen_motor.positionPID.precision = 10
        self.left_motor = PIDMotor(ev3.OUTPUT_B, Kp=Kp, Ki=Ki, Kd=Kd)
        self.left_motor.stop_action = 'brake'
        self.right_motor = PIDMotor(ev3.OUTPUT_C, Kp=Kp, Ki=Ki, Kd=Kd)
        self.right_motor.stop_action = 'brake'

        # Build lists for iterating over all motors
        self.drive_motors = [self.left_motor, self.right_motor]
        self.all_motors = [self.left_motor, self.right_motor, self.pen_motor]

        # Set starting point
        self.set_control_zeroes()

        # TODO:
        # Zero normal penlifter motor with stall speed.

        # Chalk extruder startup
        self.chalk = chalk
        if chalk:
            self.chalk_motor = ev3.Motor(ev3.OUTPUT_D)
            self.chalk_sensor = ev3.TouchSensor(ev3.INPUT_4)

            # Drive slowly to find the end position
            self.chalk_motor.run_direct(duty_cycle_sp=-50)
            self.chalk_motor.wait_until('stalled')
            self.chalk_motor.stop()
            self.chalk_motor.position = -10
            self.right_motor.polarity = 'inversed'

    # Getters & setters for plotter properties.
    # After setting these, some calculations need to be done, that's why I define special functions
    # And decorate them as setters and getters.

    @property
    def Kp(self):
        return self.drive_motors[0].positionPID.Kp

    @Kp.setter
    def Kp(self,Kp):
        for motor in self.drive_motors:
            motor.positionPID.Kp = float(Kp)

    @property
    def Ti(self):
        return self.drive_motors[0].positionPID.Ti

    @Ti.setter
    def Ti(self, Ti):
        for motor in self.drive_motors:
            motor.positionPID.Ti = float(Ti)

    @property
    def Td(self):
        return self.drive_motors[0].positionPID.Td

    @Td.setter
    def Td(self, Td):
        for motor in self.drive_motors:
            motor.positionPID.Td = float(Td)

    @property
    def cm_to_deg(self):
        return self.__cm_to_deg

    @cm_to_deg.setter
    def cm_to_deg(self, setting):
        if ev3.current_platform == 'brickpi':
            self.battery = BrickPiPowerSupply()
            factor = 2
        else:
            self.battery = ev3.PowerSupply()
            factor = 1
        self.__cm_to_deg = factor * int(setting)

    @property
    def l_rope_0(self):
        return self.__l_rope_0

    @l_rope_0.setter
    def l_rope_0(self, length):
        self.__l_rope_0 = float(length)
        self.calc_constants()

    @property
    def r_rope_0(self):
        return self.__r_rope_0

    @r_rope_0.setter
    def r_rope_0(self, length):
        self.__r_rope_0 = float(length)
        self.calc_constants()

    @property
    def att_dist(self):
        return self.__att_dist

    @att_dist.setter
    def att_dist(self,length):
        self.__att_dist = float(length)
        self.calc_constants()

    @staticmethod
    def triangle_area(a, b, c):
        """
        Calculate the area of a triangle by the lengths of it's sides using Heron's formula

        :param a: Length of side a
        :param b: Length of side b
        :param c: Length of side c
        :return: area (float)
        """
        half_p = (a + b + c) / 2
        return (half_p * (half_p - a) * (half_p - b) * (half_p - c)) ** 0.5

    def calc_constants(self):
        # Calculate the height of triangle made up by the two ropes
        self.v_margin = self.triangle_area(self.__l_rope_0, self.__r_rope_0, self.__att_dist) / self.__att_dist * 2

        # Using pythagoras to find distance from bottom triangle point to left doorframe
        self.h_margin = (self.__l_rope_0 ** 2 - self.v_margin ** 2) ** 0.5

        # For convenience, the canvas is square and centered between the attachment points
        self.canvas_size = self.__att_dist - 2 * self.h_margin

    ### Calculations for global (doorframe) to local (canvas) coordinates and back. ###
    def motor_targets_from_norm_coords(self,x_norm, y_norm):
        x,y = self.normalized_to_global_coords(x_norm,y_norm)
        return self.motor_targets_from_coords(x,y)

    def motor_targets_from_coords(self,x, y):
        l_rope = (x ** 2 + y ** 2) ** 0.5
        r_rope = ((self.__att_dist - x) ** 2 + y ** 2) ** 0.5
        l_target = (l_rope - self.__l_rope_0) * self.cm_to_deg
        r_target = (r_rope - self.__r_rope_0) * self.cm_to_deg

        return int(l_target), int(r_target)

    def coords_from_motor_pos(self,l_motor,r_motor):
        l_rope = l_motor / self.cm_to_deg + self.__l_rope_0
        r_rope = r_motor / self.cm_to_deg + self.__r_rope_0
        y = self.triangle_area(l_rope,r_rope,self.att_dist)*2/self.att_dist
        x = (l_rope**2 - y**2)**0.5
        y_norm = (y - self.v_margin)/self.canvas_size
        x_norm = (x - self.h_margin)/self.canvas_size

        return x_norm,y_norm

    def normalized_to_global_coords(self, x_norm, y_norm):
        # convert normalized coordinates to global coordinates
        x = x_norm * self.canvas_size + self.h_margin
        y = y_norm * self.canvas_size + self.v_margin

        return x, y

    ### Movement functions ###

    def set_control_zeroes(self):
        for motor in self.drive_motors:
            motor.position = 0
            #motor.positionPID.zero = motor.position
        self.pen_motor.position = PEN_UP_POS

    def move_to_coord(self,x,y, brake=False, pen=-1):
        motor_b_target, motor_c_target  = self.motor_targets_from_coords(x, y)
        self.move_to_targets((motor_b_target, motor_c_target), brake, pen)

    def move_to_norm_coord(self, x_norm, y_norm, pen=UNCHANGED, brake=False):
        motor_b_target, motor_c_target = self.motor_targets_from_norm_coords(x_norm, y_norm)
        self.move_to_targets((motor_b_target, motor_c_target),pen=pen, brake=brake)

    def move_to_targets(self, targets, brake=False, pen=-1):

        # Set targets
        for motor, tgt in zip(self.drive_motors, targets):
            motor.position_sp = tgt

        if pen == 1:        # Put the pen down
            self.pen_down()
        elif pen == 0:      # Put the pen up
            self.pen_up()


        # Now run the motors and wait for the motors to reach their targets
        # Alas ev3dev's run_to_abs_pos is not usable on BrickPi. So I emulate a PID controller.

        while 1:
            for motor in self.drive_motors:
                motor.run()

            if self.chalk and self.pen_motor.position_sp == PEN_DOWN_POS:
                # Extrude chalk if needed.

                if self.chalk_sensor.is_pressed:
                    self.chalk_motor.run_direct(duty_cycle_sp=60)
                    if self.chalk_motor.position > 20552:
                        self.reload_chalk()
                else:
                    self.chalk_motor.stop()

            if all([motor.positionPID.target_reached for motor in self.drive_motors]):
                if brake: #Run a little while long to stay in position.
                    t=time.time()+0.7
                    while t > time.time():
                        for motor in self.drive_motors:
                            motor.run()
                self.left_motor.stop()
                self.right_motor.stop()
                self.pen_motor.stop()
                break

            #We're done calculating and setting all motor speeds!
            time.sleep(0.016)

    def reload_chalk(self):
        if self.chalk:
            # Drive the loader back and wait for human to insert new chalk and resume
            self.chalk_motor.run_to_abs_pos(position_sp=0, speed_sp=600)
            self.chalk_motor.wait_while('running')
            buttons = ev3.Button()
            while not buttons.enter():
                time.sleep(0.5)

    ### Advanced plotting functions by chaining movement functions ###

    def test_drive(self):
        # A little disturbance in the force
        self.move_to_norm_coord(0.0,0.5)
        self.move_to_norm_coord(0.3,0.3)
        self.move_to_norm_coord(0.0,0.0)

    def plot_from_file(self, filename):
        """
        Generator function for plotting from coords.csv file. After each next() it returns the pct done of the plotting
        This way the plotting can easily be aborted and status can be given. Gotta love python for this.
        Usage:

        gen = plotter.plot_from_file(myfile)
        while 1:
            try:
                pct_done = next(gen)
            except StopIteration:
                break

        :param filename: str
        :return: percentage done: float
        """
        coords = open(filename)
        num_coords = int(coords.readline())  #coords contains it's length on the first line.

        #drive to the first coordinate
        self.pen_up()
        # read from file
        x_norm, y_norm = [float(n) for n in coords.readline().split(",")]
        #move
        self.move_to_norm_coord(x_norm, y_norm)
        self.pen_down()
        for i in range(num_coords - 1):
            # read from file
            x_norm, y_norm = [float(n) for n in coords.readline().split(",")]
            #move
            self.move_to_norm_coord(x_norm, y_norm)
            yield float(i+1)/num_coords*100

        coords.close()
        self.pen_up()
        self.move_to_norm_coord(0, 0)
        yield 100


    ### Calibration & manual movement functions ###

    def pen_up(self):
        self.pen_motor.run_to_abs_pos(position_sp=PEN_UP_POS)
        self.pen_motor.wait_while('running')

    def pen_down(self):
        self.pen_motor.run_to_abs_pos(position_sp=PEN_DOWN_POS)
        self.pen_motor.wait_while('running')
        if self.chalk:
            time.sleep(0.5) # Wait a bit to avoid touch sensor bounce.
            while self.chalk_sensor.is_pressed:
                self.chalk_motor.run_forever(speed_sp=150)
                if self.chalk_motor.position > 20552:
                    self.reload_chalk()
            self.chalk_motor.stop()

    def left_fwd(self):
        self.left_motor.run_direct(duty_cycle_sp=100)

    def left_stop(self):
        self.left_motor.stop()

    def left_back(self):
        self.left_motor.run_direct(duty_cycle_sp=-100)

    def right_fwd(self):
        self.right_motor.run_direct(duty_cycle_sp=100)

    def right_stop(self):
        self.right_motor.stop()

    def right_back(self):
        self.right_motor.run_direct(duty_cycle_sp=-100)

    def stop_all_motors(self):
        for motor in self.all_motors:
            motor.stop()
        print("Motors stopped")

