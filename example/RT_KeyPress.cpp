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

/** Sine-sweep trajectory amplitude and frequency */
const std::vector<double> k_sineAmp
    = {0.035, 0.035, 0.035, 0.035, 0.035, 0.035, 0.035};
const std::vector<double> k_sineFreq = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};

/** Type of motion specified by user */
std::string motionType = {};
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

        static Eigen::VectorXd xd_des = Eigen::VectorXd::Zero(6);
        // int move_direction = -1;
        xd_des[2] = des_vel * (*move_direction);
        static std::vector<double> q_des = robotStates.q;
        static std::vector<double> qd_vector = {0, 0, 0, 0, 0, 0, 0};
        static std::vector<double> q_acc = {0, 0, 0, 0, 0, 0, 0};

        // Eigen::VectorXd xd_des = Eigen::VectorXd::Zero(6);
        robot->getRobotStates(robotStates);
        model->updateModel(robotStates.q, robotStates.dtheta);
        auto J = model->getJacobian("flange");
        auto J_inv = J.inverse();

        if ((*move_direction) < 0
            && fabs(robotStates.extForceInTcpFrame[2])
                   > fabs(max_bottom_out_force)) {
            *move_direction = 1;
            xd_des[2] = des_vel * (*move_direction);
            std::cout << " revert!!!!!!!!!!!!" << std::endl;
        }
        if ((*move_direction) < 0) {
            Eigen::VectorXd qd = J_inv * xd_des;
            for (int i = 0; i < 7; i++) {
                qd_vector[i] = qd[i];
                q_des[i] += qd[i] * 0.001;
            }
            q_stack->push(q_des);
            qd_stack->push(qd_vector);
            // std::cout << " q_des " << q_des[1] << "xd_des[2]" << xd_des[2]
            //   << std::endl;
        } else if (q_stack->size() == 0) {
            scheduler->stop();
        } else {
            q_des = q_stack->top();
            qd_vector = qd_stack->top();
            for (int i = 0; i < 7; i++) {
                qd_vector[i] *= -1;
            }
            q_stack->pop();
            qd_stack->pop();
        }

        // robot.sendJointPosition(
        //     q_des, qd_vector, q_acc, max_qd, max_qdd, max_qddd);
        robot->streamJointPosition(q_des, qd_vector, q_acc);

        // Record data
        *myfile << robotStates.flangePose[0] << ',' << robotStates.flangePose[1]
                << ',' << robotStates.flangePose[2] << ','
                << robotStates.extForceInTcpFrame[2] << ','
                << robotStates.camPose[2] << ',' //
                << robotStates.q[0] << ',' << robotStates.q[1] << ','
                << robotStates.q[2] << ',' << robotStates.q[3] << ','
                << robotStates.q[4] << ',' << robotStates.q[5] << ','
                << robotStates.q[6] << ',' << q_des[0] << ',' << q_des[1] << ','
                << q_des[2] << ',' << q_des[3] << ',' << q_des[4] << ','
                << q_des[5] << ',' << q_des[6] << ',' << '\n';

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
        std::cout << "Press " << i + 1 << std::endl;
        //////////////////////////////////////////////////
        Eigen::VectorXd xd_des = Eigen::VectorXd::Zero(6);
        int move_direction = -1;
        xd_des[2] = des_vel * move_direction;
        std::vector<double> q_des = robotStates.q;
        std::vector<double> qd_vector = {0, 0, 0, 0, 0, 0, 0};
        std::vector<double> q_acc = {0, 0, 0, 0, 0, 0, 0};
        // std::vector<double> max_qd = {1, 1, 1, 1, 1, 1, 1};
        // std::vector<double> max_qdd = {5, 5, 5, 5, 5, 5, 5};
        // std::vector<double> max_qddd = {5, 5, 5, 5, 5, 5, 5};
        std::stack<std::vector<double>> q_stack;
        std::stack<std::vector<double>> qd_stack;

        char nowChar[100];
        time_t now = time(0);
        struct tm* tm_now = localtime(&now);

        strftime(nowChar, 100, "%m-%d-%Y-%H:%M:%S", tm_now);
        std::string nowString(nowChar);
        std::ofstream myfile;
        myfile.open("PressTest" + std::to_string(i) + "_" + nowString + ".csv");
        myfile << "x" << ',' << "y" << ',' << "z" << ',' //
               << "forceZ" << ',' << "rawforceZ" << ',' //
               << "q1" << ',' << "q2" << ',' << "q3" << ',' << "q4" << ','
               << "q5" << ',' << "q6" << ',' << "q7" << ',' << "qd1" << ','
               << "qd2" << ',' << "qd3" << ',' << "qd4" << ',' << "qd5" << ','
               << "qd6" << ',' << "qd7" << '\n';

        // Set mode after robot is operational
        switchMode(*robot, flexiv::MODE_PRIMITIVE_EXECUTION);

        robot->executePrimitive(
            "CaliForceSensor(dataCollectionTime=1.0, enableStaticCheck=1)");

        // Wait for reached target
        while (flexiv::utility::parsePtStates(
                   robot->getPrimitiveStates(), "reachedTarget")
               != "1") {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        switchMode(*robot, flexiv::MODE_JOINT_POSITION);
        flexiv::Scheduler scheduler;
        scheduler.addTask(
            std::bind(periodicTask, robot, &scheduler, log, robotStates,
                des_vel, max_bottom_out_force, &move_direction, model, &q_stack,
                &qd_stack, &myfile),
            "HP periodic", 1, 45);

        // Start all added tasks, this is by default a blocking method
        scheduler.start();
        std::cout << " end!!!" << std::endl;
        // bool exit_flag = false;
        // while (!exit_flag) {
        //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
        // }
        myfile.close();
    }
}

void printHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot IP] [local IP]" << std::endl;
    std::cout << "    robot IP: address of the robot server" << std::endl;
    std::cout << "    local IP: address of this PC" << std::endl;
    std::cout << "Optional arguments: [--hold]" << std::endl;
    std::cout << "    --hold: robot holds current joint positions, otherwise do a sine-sweep" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

int main(int argc, char* argv[])
{
    // Log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse Parameters
    //=============================================================================
    if (argc < 3
        || flexiv::utility::programArgsExistAny(argc, argv, {"-h", "--help"})) {
        printHelp();
        return 1;
    }

    // IP of the robot server
    std::string robotIP = argv[1];

    // IP of the workstation PC running this program
    std::string localIP = argv[2];

    // Type of motion specified by user
    if (flexiv::utility::programArgsExist(argc, argv, "--hold")) {
        log.info("Robot holding current pose");
        motionType = "hold";
    } else {
        log.info("Robot running joint sine-sweep");
        motionType = "sine-sweep";
    }

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
        // robot.enable();

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

        ///// MoveL to pre press position //////
        // log.info("Executing primitive: Home");
        // // Send command to robot
        // robot.executePrimitive("Home()");
        // // Wait for robot to reach target location by checking for
        // // "reachedTarget = 1" in the list of current primitive states
        // while (flexiv::utility::parsePtStates(
        //            robot.getPrimitiveStates(), "reachedTarget")
        //        != "1") {
        //     std::this_thread::sleep_for(std::chrono::seconds(1));
        // }

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

        if (1) {
            switchMode(robot, flexiv::MODE_JOINT_POSITION);

            // Periodic Tasks
            //=============================================================================
            flexiv::Scheduler scheduler;
            // Add periodic task with 1ms interval and highest applicable
            // priority
            double des_vel = 0.0005;
            double max_bottom_out_force = 1;
            int repeat_times = 3;
            PressRlease(&robot, robotStates, &model, &log, des_vel,
                max_bottom_out_force, repeat_times);
        }
    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 1;
    }

    return 0;
}
