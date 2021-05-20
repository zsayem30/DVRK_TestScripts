#!/usr/bin/env python

import dvrk
import math
import sys
import rospy
import numpy
import PyKDL
import argparse

if sys.version_info.major < 3:
    input = raw_input

# print with node id
def print_id(message):
    print('%s -> %s' % (rospy.get_caller_id(), message))

# example of application using arm.py
class example_application:

    # configuration
    def configure(self, robot_name, expected_interval):
        print_id('configuring dvrk_psm_test for %s' % robot_name)
        self.expected_interval = expected_interval
        self.arm = dvrk.psm(arm_name = robot_name,
                            expected_interval = expected_interval)

    # homing example
    def home(self):
        print_id('starting enable')
        if not self.arm.enable(10):
            sys.exit('failed to enable within 10 seconds')
        print_id('starting home')
        if not self.arm.home(10):
            sys.exit('failed to home within 10 seconds')
        # get current joints just to set size
        print_id('move to starting position')
        goal = numpy.copy(self.arm.setpoint_jp())
        # go to zero position, make sure 3rd joint is past cannula
        goal.fill(0)
        goal[2] = 0.12
        self.arm.move_jp(goal).wait()

    # utility to position tool/camera deep enough before cartesian examples
    def prepare_cartesian(self):
        # make sure the camera is past the cannula and tool vertical
        goal = numpy.copy(self.arm.setpoint_jp())
        if ((self.arm.name() == 'PSM1') or (self.arm.name() == 'PSM2') or (self.arm.name() == 'PSM3') or (self.arm.name() == 'ECM')):
            # set in position joint mode
            goal[0] = 0.0
            goal[1] = 0.0
            goal[2] = 0.12
            self.arm.move_jp(goal).wait()


    # goal jaw control example
    def run_jaw_move(self):

        print_id('move joints with command')
        choice = int(raw_input(" > Your choice: "))

        # try to open and close with a cartesian goal
        self.prepare_cartesian()

        initial_cartesian_position = PyKDL.Frame()
        initial_cartesian_position.p = self.arm.setpoint_cp().p
        initial_cartesian_position.M = self.arm.setpoint_cp().M
        print(self.arm.measured_jp())
        goal = PyKDL.Frame()
        goal.p = self.arm.setpoint_cp().p
        goal.M = self.arm.setpoint_cp().M

        # motion parameters
        amplitude = 0.5 # 5 cm

        # first motion
        goal.p[choice] =  initial_cartesian_position.p[choice] - amplitude
        print_id('closing and moving right (1)')
        self.arm.move_cp(goal).wait(is_busy = True)

        #self.arm.jaw.close().wait()

        cartesian_position = PyKDL.Frame()
        cartesian_position.M = self.arm.measured_cp().M
        cartesian_position.p = self.arm.measured_cp().p
        print_id('Final position')
        print(self.arm.measured_jp())
        # # second motion
        # goal.p[0] =  initial_cartesian_position.p[0] + amplitude
        # goal.p[1] =  initial_cartesian_position.p[1]
        # print_id('opening and moving left (1)')
        # self.arm.move_cp(goal).wait(is_busy = True)
        # self.arm.jaw.open().wait()
        print_id('close and open with cartesian move command')
        input("    Press Enter to continue...")
        # back to starting point
        goal.M[0][0] =  initial_cartesian_position.M[0][0]
        # goal.p[1] =  initial_cartesian_position.p[1]
        print_id('moving back (3)')
        self.arm.move_cp(goal).wait()
        # print(PyKDL.Frame())


    # goal jaw control example
    def run_jaw_servo(self):
        print_id('starting jaw servo')
        # try to open and close directly, needs interpolation
        print_id('close and open without other servo command')
        input("    Press Enter to continue...")
        start_angle = math.radians(50.0)
        self.arm.jaw.open(angle = start_angle).wait()
        # assume we start at 30 the move +/- 30
        amplitude = math.radians(30.0)
        duration = 5  # seconds
        samples = int(duration / self.expected_interval)
        # create a new goal starting with current position
        for i in range(samples * 4):
            goal = start_angle + amplitude * (math.cos(i * math.radians(360.0) / samples) - 1.0)
            self.arm.jaw.servo_jp(numpy.array(goal))
            rospy.sleep(self.expected_interval)


    # main method
    def run(self):
        self.home()
        self.run_jaw_move()
        # self.run_jaw_servo()
        #self.run_jaw_move() # just to make sure we can transition back to trajectory mode


if __name__ == '__main__':
    # ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
    rospy.init_node('dvrk_psm_test')
    # strip ros arguments
    argv = rospy.myargv(argv=sys.argv)

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type=str, required=True,
                        choices=['PSM1', 'PSM2', 'PSM3'],
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-i', '--interval', type=float, default=0.01,
                        help = 'expected interval in seconds between messages sent by the device')
    args = parser.parse_args(argv[1:]) # skip argv[0], script name

    application = example_application()
    application.configure(args.arm, args.interval)
    application.run()
