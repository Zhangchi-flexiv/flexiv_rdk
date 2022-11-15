#!/usr/bin/env python
import time
from utility import parse_pt_states
import csv

def pressRelease(robot,robot_states,mode, max_wrench, retreat_force,num_repeat,des_vel):

    # =============================================================================
    frequency = 1000 # 1K Hz control rate
    period = 1.0/frequency
    MOVE_AXIS =2 # Moving along TCP Z axis
    print("Sending command to robot at", frequency,
        "Hz, or", period, "seconds interval")

    for i in range(num_repeat):

        # Record csv file for each press
        f = open('csv_test'+str(i)+'.csv', 'w')
        writer = csv.writer(f)
        # writer.writerow(['displacement','forceZ','rawforceZ'])
        writer.writerow(['x','y','displacement','forceZ','rawforceZ','theta1','theta2','theta3','theta4','theta5','theta6','theta7','q1','q2','q3','q4','q5','q6','q7','rawq1','rawq2','rawq3','rawq4','rawq5','rawq6','rawq7','lq1','lq2','lq3','lq4','lq5','lq6','lq7'])

        
        # Use current robot TCP pose as initial pose
        robot.getRobotStates(robot_states)
        init_pose = robot_states.tcpPose.copy()
        start_pose = robot_states.tcpPose.copy()
        print("Initial pose set to: ", init_pose)

        # Initialize target vectors
        target_pose = init_pose.copy()
        # Init parameters for each key press
        loop_counter = 0
        exit_flag = False
        move_direction = -1
        
        # Set mode after robot is operational
        robot.setMode(mode.MODE_PRIMITIVE_EXECUTION)

        # Wait for the mode to be switched
        while (robot.getMode() != mode.MODE_PRIMITIVE_EXECUTION):
            time.sleep(1)
        
        # Calibrate force sensor for each press
        robot.executePrimitive("CaliForceSensor(dataCollectionTime=1.0, enableStaticCheck=1)")
        while (parse_pt_states(robot.getPrimitiveStates(), "reachedTarget") != "1"):
            print(robot.getPrimitiveStates()) ##Not necessary to print all status, 
            time.sleep(0.5)


        # Set mode after robot is operational
        robot.setMode(mode.MODE_CARTESIAN_IMPEDANCE_NRT)
        # Wait for the mode to be switched
        while (robot.getMode() != mode.MODE_CARTESIAN_IMPEDANCE_NRT):
            time.sleep(1)
            
        while (exit_flag==False):
        
            # Use sleep to control loop period
            time.sleep(period)

            # Monitor fault on robot server
            if robot.isFault():
                raise Exception("Fault occurred on robot server, exiting ...")

            # Update robot states
            robot.getRobotStates(robot_states)

            # check for the press force with threshold
            if(abs(robot_states.extForceInTcpFrame[2])>abs(retreat_force) and move_direction<0):
                move_direction = 1
                init_pose = target_pose.copy()
                loop_counter=0
                print("Start Release")
            
            # Calculate target pose, only update Z axis position
            target_pose[MOVE_AXIS] = init_pose[MOVE_AXIS] + loop_counter*period*move_direction*des_vel
            
            # Send command
            robot.sendTcpPose(target_pose, max_wrench)
            
            # log data
            # writer.writerow([robot_states.tcpPose[2],robot_states.extForceInTcpFrame[2],robot_states.camPose[2]])
            writer.writerow([robot_states.tcpPose[0],robot_states.tcpPose[1],robot_states.tcpPose[2],robot_states.extForceInTcpFrame[2],robot_states.camPose[2],robot_states.theta[0],robot_states.theta[1],robot_states.theta[2],robot_states.theta[3],robot_states.theta[4],robot_states.theta[5],robot_states.theta[6],robot_states.q[0],robot_states.q[1],robot_states.q[2],robot_states.q[3],robot_states.q[4],robot_states.q[5],robot_states.q[6],robot_states.tauExt[0],robot_states.tauExt[1],robot_states.tauExt[2],robot_states.tauExt[3],robot_states.tauExt[4],robot_states.tauExt[5],robot_states.tauExt[6],robot_states.tauDes[0],robot_states.tauDes[1],robot_states.tauDes[2],robot_states.tauDes[3],robot_states.tauDes[4],robot_states.tauDes[5],robot_states.tauDes[6]])

            # check for exit
            if(robot_states.tcpPose[MOVE_AXIS]>start_pose[MOVE_AXIS] and move_direction>0):
                exit_flag=True
                # print("Finished all the press")
                
            # Increment loop counter
            loop_counter += 1