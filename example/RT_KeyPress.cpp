/**
 * @example joint_position_control.cpp
 * Run joint position control to hold or sine-sweep all joints.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Scheduler.hpp>
#include <flexiv/Utility.hpp>
#include <flexiv/Model.hpp>

#include <iostream>
#include <string>
#include <cmath>
#include <thread>
#include <stack>
#include <fstream>

namespace {
/** Robot DOF */
const int k_robotDofs = 7;

/** RT loop period [sec] */
const double k_loopPeriod = 0.001;
}

void switchMode(flexiv::Robot& robot, flexiv::Mode mode)
{
    // Set mode after robot is operational
    robot.setMode(mode);

    // Wait for the mode to be switched
    while (robot.getMode() != mode) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

/** Callback function for realtime periodic task */
void periodicTask(flexiv::Robot* robot, flexiv::Scheduler* scheduler,
    flexiv::Log* log, flexiv::RobotStates& robotStates, float des_vel,
    float max_bottom_out_force, int* move_direction, flexiv::Model* model,
    std::vector<double>* q_des, std::vector<double>* qd_vector,
    std::stack<std::vector<double>>* q_stack,
    std::stack<std::vector<double>>* qd_stack, std::ofstream* myfile)
{

    try {
        // Monitor fault on robot server
        if (robot->isFault()) {
            throw flexiv::ServerException(
                "periodicTask: Fault occurred on robot server, exiting ...");
        }

        // Read robot states
        robot->getRobotStates(robotStates);

        // update/init control/robot status
        static Eigen::VectorXd base_xd_des = Eigen::VectorXd::Zero(6);
        static Eigen::VectorXd tcp_xd_des = Eigen::VectorXd::Zero(6);
        static std::vector<double> q_acc = {0, 0, 0, 0, 0, 0, 0};

        Eigen::Quaterniond quat(robotStates.flangePose[3],
            robotStates.flangePose[4], robotStates.flangePose[5],
            robotStates.flangePose[6]);
        tcp_xd_des[2] = des_vel * (*move_direction);
        base_xd_des.head(3) = quat.normalized().toRotationMatrix() * tcp_xd_des;
        model->updateModel(*q_des, *qd_vector);
        Eigen::MatrixXd J = model->getJacobian("flange");
        Eigen::MatrixXd J_tmp = J * J.transpose();
        Eigen::MatrixXd J_inv = J.transpose() * J_tmp.inverse();
        Eigen::VectorXd qd = J_inv * base_xd_des;
        if ((*move_direction) > 0
            && fabs(robotStates.extForceInTcpFrame[2])
                   > fabs(max_bottom_out_force)) {
            (*move_direction) = -1;
            std::cout << " revert!!!!!!!!!!!!" << std::endl;
        }
        if ((*move_direction) > 0) {

            for (int i = 0; i < k_robotDofs; i++) {
                (*qd_vector)[i] = qd[i];
                (*q_des)[i] += qd[i] * k_loopPeriod;
            }
            q_stack->push(*q_des);
            qd_stack->push(*qd_vector);
        } else if (q_stack->size() == 0) {
            scheduler->stop();
        } else {
            *q_des = q_stack->top();
            *qd_vector = qd_stack->top();
            for (int i = 0; i < k_robotDofs; i++) {
                (*qd_vector)[i] *= -1;
            }
            q_stack->pop();
            qd_stack->pop();
        }

        robot->streamJointPosition(*q_des, *qd_vector, q_acc);

        // Record data
        *myfile << robotStates.flangePose[0] << ',' << robotStates.flangePose[1]
                << ',' << robotStates.flangePose[2] << ','
                << robotStates.extForceInTcpFrame[2] << '\n';

    } catch (const flexiv::Exception& e) {
        log->error(e.what());
        scheduler->stop();
    }
}

void PressRlease(flexiv::Robot* robot, flexiv::RobotStates& robotStates,
    flexiv::Model* model, flexiv::Log* log, float des_vel,
    float max_bottom_out_force, int repeat_times)
// flexiv::Scheduler &scheduler,
{

    for (int i = 0; i < repeat_times; i++) {

        std::cout << "Press Test:" << i + 1 << std::endl;

        //  only record data
        char nowChar[100];
        time_t now = time(0);
        struct tm* tm_now = localtime(&now);
        strftime(nowChar, 100, "%m-%d-%Y-%H:%M:%S", tm_now);
        std::string nowString(nowChar);
        std::ofstream myfile;
        myfile.open("PressTest" + std::to_string(i) + "_" + nowString + ".csv");
        myfile << "x" << ',' << "y" << ',' << "z" << ',' //
               << "forceZ" << '\n';

        //  <<<<<<<<<<<  CaliForceSensor right before the prese  <<<<<<<<
        // Switch mode for calibrate FT-sensor
        switchMode(*robot, flexiv::MODE_PRIMITIVE_EXECUTION);
        robot->executePrimitive(
            "CaliForceSensor(dataCollectionTime=1.0, enableStaticCheck=1)");
        // Wait for the end of Calibrate force sensor
        while (flexiv::utility::parsePtStates(
                   robot->getPrimitiveStates(), "reachedTarget")
               != "1") {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        //  >>>>>>>>>>  CaliForceSensor right before the prese   >>>>>>>>>>

        //  <<<<<<<<<<<  Motion control part  <<<<<<<<
        // init inputs for real-time robot control
        robot->getRobotStates(robotStates);
        int move_direction = 1;
        std::stack<std::vector<double>> q_stack;
        std::stack<std::vector<double>> qd_stack;
        std::vector<double> q_des = robotStates.q;
        std::vector<double> qd_vector = {0, 0, 0, 0, 0, 0, 0};
        // Switch mode for preprare PressRelease motion control
        switchMode(*robot, flexiv::MODE_JOINT_POSITION);
        flexiv::Scheduler scheduler;
        scheduler.addTask(
            std::bind(periodicTask, robot, &scheduler, log, robotStates,
                des_vel, max_bottom_out_force, &move_direction, model, &q_des,
                &qd_vector, &q_stack, &qd_stack, &myfile),
            "HP periodic", 1, 45);
        // Start all added tasks, this is by default a blocking method
        scheduler.start();
        //  >>>>>>>>>>  Motion control part   >>>>>>>>>>

        std::cout << " end!!!" << std::endl;

        myfile.close();
    }
}

void printHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot IP] [local IP] [press]" << std::endl;
    std::cout << "    robot IP: address of the robot server" << std::endl;
    std::cout << "    local IP: address of this PC" << std::endl;
    std::cout << "    press: 0 for not press, others for press" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

int main(int argc, char* argv[])
{
    // Log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse Parameters
    //=============================================================================
    if (argc < 4
        || flexiv::utility::programArgsExistAny(argc, argv, {"-h", "--help"})) {
        printHelp();
        return 1;
    }

    // IP of the robot server
    std::string robotIP = argv[1];

    // IP of the workstation PC running this program
    std::string localIP = argv[2];
    // std::string argv3 = argv[3];
    int repeat_times = atoi(argv[3]);
    try {
        // RDK Initialization
        //=============================================================================
        // Instantiate robot interface
        flexiv::Robot robot(robotIP, localIP);

        // Create data struct for storing robot states
        flexiv::RobotStates robotStates;
        flexiv::Model model(&robot);

        // Clear fault on robot server if any
        if (robot.isFault()) {
            log.warn("Fault occurred on robot server, trying to clear ...");
            // Try to clear the fault
            robot.clearFault();
            std::this_thread::sleep_for(std::chrono::seconds(2));
            // Check again
            if (robot.isFault()) {
                log.error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            log.info("Fault on robot server is cleared");
        }

        // Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...");
        robot.enable();

        // Wait for the robot to become operational
        int secondsWaited = 0;
        while (!robot.isOperational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (++secondsWaited == 10) {
                log.warn(
                    "Still waiting for robot to become operational, please "
                    "check that the robot 1) has no fault, 2) is booted "
                    "into Auto mode");
            }
        }
        log.info("Robot is now operational");

        switchMode(robot, flexiv::MODE_PRIMITIVE_EXECUTION);
        ///// MoveL to pre press position /////
        log.info("Executing primitive: MoveL");
        // Send command to robot
        robot.executePrimitive(
            "MoveL(target=0.69 -0.11 0.119 180 0 180 WORLD  WORLD_ORIGIN,, "
            "maxVel=0.2, "
            "targetTolLevel=1)");

        // Wait for reached target
        while (flexiv::utility::parsePtStates(
                   robot.getPrimitiveStates(), "reachedTarget")
               != "1") {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        switchMode(robot, flexiv::MODE_JOINT_POSITION);
        flexiv::Scheduler scheduler;
        // some parameter for the press/release test
        double des_vel = 0.01;
        double max_bottom_out_force = 1;
        PressRlease(&robot, robotStates, &model, &log, des_vel,
            max_bottom_out_force, repeat_times);

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 1;
    }

    return 0;
}
