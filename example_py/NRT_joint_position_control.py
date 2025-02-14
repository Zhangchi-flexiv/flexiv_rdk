#!/usr/bin/env python

"""NRT_joint_position_control.py

Non-real-time joint position control to hold or sine-sweep all joints.
"""

__copyright__ = "Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import math
import argparse

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
    argparser.add_argument(
        "frequency", help="command frequency, 1 to 1000 [Hz]", type=float)
    # Optional arguments
    argparser.add_argument(
        "--hold", action="store_true",
        help="robot holds current joint positions, otherwise do a sine-sweep")
    args = argparser.parse_args()

    # Check if arguments are valid
    frequency = args.frequency
    assert (frequency >= 1 and frequency <= 1000), "Invalid <frequency> input"

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
        robot.enable()

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
        robot.setMode(mode.MODE_JOINT_POSITION_NRT)

        # Wait for the mode to be switched
        while (robot.getMode() != mode.MODE_JOINT_POSITION_NRT):
            time.sleep(1)

        # Application-specific Code
        # =============================================================================
        period = 1.0/frequency
        loop_time = 0
        print("Sending command to robot at", frequency,
              "Hz, or", period, "seconds interval")

        # Use current robot joint positions as initial positions
        robot.getRobotStates(robot_states)
        init_pos = robot_states.q.copy()
        print("Initial positions set to: ", init_pos)

        # Robot degrees of freedom
        DOF = len(robot_states.q)

        # Initialize target vectors
        target_pos = init_pos.copy()
        target_vel = [0.0] * DOF
        target_acc = [0.0] * DOF

        # Joint motion constraints
        MAX_VEL = [2.0] * DOF
        MAX_ACC = [3.0] * DOF

        # Joint sine-sweep amplitude [rad]
        SWING_AMP = 0.1

        # TCP sine-sweep frequency [Hz]
        SWING_FREQ = 0.3

        # Send command periodically at user-specified frequency
        while True:
            # Use sleep to control loop period
            time.sleep(period)

            # Monitor fault on robot server
            if robot.isFault():
                raise Exception("Fault occurred on robot server, exiting ...")

            # Sine-sweep all joints
            if not args.hold:
                for i in range(DOF):
                    target_pos[i] = init_pos[i] + SWING_AMP * \
                        math.sin(2 * math.pi * SWING_FREQ * loop_time)
            # Otherwise all joints will hold at initial positions

            # Send command
            robot.sendJointPosition(
                target_pos, target_vel, target_acc, MAX_VEL, MAX_ACC)

            # Increment loop time
            loop_time += period

    except Exception as e:
        # Print exception error message
        log.error(str(e))


if __name__ == "__main__":
    main()
