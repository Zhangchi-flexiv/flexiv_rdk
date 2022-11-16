#!/usr/bin/env python

"""nrt_cartesian_impedance_control.py

Run non-real-time Cartesian impedance control to hold or sine-sweep the robot TCP.
"""

__copyright__ = "Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

from shutil import move
import time
import math
import argparse
import csv

from tkinter.tix import Tree

# Utility methods
from utility import quat2eulerZYX
from utility import parse_pt_states
from utility import list2str
from press_release import pressRelease
from press_release_joint import pressRelease_joint

# Import Flexiv RDK Python library
# fmt: off
import sys
sys.path.insert(0, "../lib_py")
import flexivrdk

# fmt: on




def main():
    # Parse Arguments
    # =============================================================================
    argparser = argparse.ArgumentParser()
    # Required arguments
    argparser.add_argument("robot_ip", help="IP address of the robot server")
    argparser.add_argument("local_ip", help="IP address of this PC")
    # argparser.add_argument(
    #     "frequency", help="command frequency, 1 to 1000 [Hz]", type=float)
    # Optional arguments
    argparser.add_argument(
        "--hold", action="store_true",
        help="robot holds current TCP pose, otherwise do a sine-sweep")
    args = argparser.parse_args()

    # Check if arguments are valid
    # frequency = args.frequency
    # assert (frequency >= 1 and frequency <= 1000), "Invalid <frequency> input"

    # Define alias
    # =============================================================================
    robot_states = flexivrdk.RobotStates()
    log = flexivrdk.Log()
    mode = flexivrdk.Mode

    try:
        # RDK Initialization
        # =============================================================================
        # Instantiate robot interface
        robot = flexivrdk.Robot(args.robot_ip, args.local_ip)

        # Clear fault on robot server if any
        if robot.isFault():
            log.warn("Fault occurred on robot server, trying to clear ...")
            # Try to clear the fault
            robot.clearFault()
            time.sleep(2)
            # Check again
            if robot.isFault():
                log.error("Fault cannot be cleared, exiting ...")
                return
            log.info("Fault on robot server is cleared")

        # Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...")
        # robot.enable()

        # Wait for the robot to become operational
        seconds_waited = 0
        while not robot.isOperational():
            time.sleep(1)
            seconds_waited += 1
            if seconds_waited == 10:
                log.warn(
                    "Still waiting for robot to become operational, please "
                    "check that the robot 1) has no fault, 2) is booted "
                    "into Auto mode")

        log.info("Robot is now operational")
        
        # Set mode after robot is operational
        robot.setMode(mode.MODE_PRIMITIVE_EXECUTION)

        # Wait for the mode to be switched
        while (robot.getMode() != mode.MODE_PRIMITIVE_EXECUTION):
            time.sleep(1)
        
        log.info("Executing primitive: MoveL")

        # Send command to robot
        robot.executePrimitive(
            "MoveL(target=0.69 -0.11 0.115 180 0 180 WORLD , maxVel=0.2, targetTolLevel=1)")
            # "MoveL(target=0.69 -0.11 0.2 180 0 180 WORLD , maxVel=0.2)")

        # Wait for reached target
        while (parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1"):
            time.sleep(1)
            
            
        press_key= True
        # press_key= False
        if(press_key):
            max_wrench = [5.0, 5.0, 10, 2.0, 2.0, 2.0]
            des_vel = 0.001
            retreat_force=0.8
            num_repeat = 1

            # pressRelease(robot,robot_states,mode,max_wrench,retreat_force,num_repeat,des_vel)
            pressRelease_joint(robot,robot_states,mode,max_wrench,retreat_force,num_repeat,des_vel)

    except Exception as e:
        # Print exception error message
        log.error(str(e))


if __name__ == "__main__":
    main()
