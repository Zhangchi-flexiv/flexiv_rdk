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
#include <Eigen/Dense>
#define pi 3.14159265358979323846 /* pi */

#include <iostream>
#include <string>
#include <cmath>
#include <thread>
#include <stack>
#include <fstream>

using namespace Eigen;

namespace {
/** Robot DOF */
const int k_robotDofs = 7;

/** RT loop period [sec] */
const double k_loopPeriod = 0.001;

/** desired velocity to press the key [m/s] in flange cooridinate  */
const float k_desiredVelocity = 0.0005;

/** contact force threshold [N]  */
const float k_max_contact_force = 1;
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

Matrix4d transform_MDH(double theta, double alpha, double a_nom, double d_nom)
{
    double stheta = std::sin(theta);
    double ctheta = std::cos(theta);
    double salpha = std::sin(alpha);
    double calpha = std::cos(alpha);
    Matrix4d H;
    H << ctheta, -stheta, 0, a_nom, stheta * calpha, ctheta * calpha, -salpha,
        -salpha * d_nom, stheta * salpha, ctheta * salpha, calpha,
        calpha * d_nom, 0, 0, 0, 1;
    // std::cout<<H<<std::endl;
    return H;
}

// Forward kinematics
std::vector<Matrix4d> getT_MDH(std::vector<double> theta) // in radian
{
    // MDH parameters
    std::vector<double> aph_nom
        = {0., pi / 2, -pi / 2, -pi / 2, pi / 2, pi / 2, pi / 2};
    std::vector<double> a_nom = {0., 0., 0., 0.020, -0.020, 0, 0.110};
    std::vector<double> d_nom = {0.365, 0.065, 0.395, 0.055, 0.385, 0.100, 0.};
    std::vector<Matrix4d> T_MDH {};
    for (int i = 0; i < 7; i++) {
        Matrix4d H = transform_MDH(theta[i], aph_nom[i], a_nom[i], d_nom[i]);
        T_MDH.push_back(H);
    }
    Matrix4d H_flange;
    H_flange << -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0.179, 0, 0, 0, 1;
    T_MDH.push_back(H_flange);
    return T_MDH;
}

// T_DE: Coordinate systems: D and E (origin frame E in MDH along Z )
std::vector<Matrix4d> getT_DE()
{
    std::vector<Matrix4d> T_DE {};
    double z_MDH2E[8] = {-0.1535, 0.0215, -0.1405, 0.0245, -0.1405, -0.0205,
        0.0840, 0.01975}; // z offset between MDH and E frame, in meter
    // consider the link frame should be in the output flange of the joint, an
    // offset should be considered

    for (int i = 0; i < 8; i++) {
        Matrix4d T_DEi = Matrix4d::Identity();
        T_DEi(2, 3) = z_MDH2E[i];
        T_DE.push_back(T_DEi);
    }
    return T_DE;
} // end of T_DE

std::vector<double> deg2rad(std::vector<double> theta_deg)
{
    std::vector<double> theta_rad = {0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 7; i++) {
        theta_rad[i] = theta_deg[i] * pi / 180;
    }
    return theta_rad;
}

/** Callback function for realtime periodic task */
void periodicTask(flexiv::Robot* robot, flexiv::Scheduler* scheduler,
    flexiv::Log* log, flexiv::RobotStates& robotStates, float desired_velocity,
    float max_contact_force, int* move_direction, flexiv::Model* model,
    std::vector<double>* q_des, std::vector<double>* dq_vector,
    std::stack<std::vector<double>>* q_stack,
    std::stack<std::vector<double>>* dq_stack, std::ofstream* myfile)
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

        //  get tcp pose in flange corridinate
        Eigen::Quaterniond quat(robotStates.flangePose[3],
            robotStates.flangePose[4], robotStates.flangePose[5],
            robotStates.flangePose[6]);
        tcp_xd_des[2] = desired_velocity * (*move_direction);
        //    tranform back to world coordinate
        base_xd_des.head(3) = quat.normalized().toRotationMatrix() * tcp_xd_des;
        model->updateModel(*q_des, *dq_vector);
        Eigen::MatrixXd J = model->getJacobian("flange");
        Eigen::MatrixXd J_tmp = J * J.transpose();
        // psuedo inverse
        Eigen::MatrixXd J_inv = J.transpose() * J_tmp.inverse();
        // inverse kinematics to get joint velocity
        Eigen::VectorXd dq = J_inv * base_xd_des;

        ////////////////////////////////

        Matrix4d fk_ori = Matrix4d::Identity();
        // std::vector<Matrix4d> T_i2base_ori {};
        // T_i2base_ori.push_back(fk_ori);robotStates.theta
        std::vector<double> q_rad = {robotStates.theta[0], robotStates.theta[1],
            robotStates.theta[2], robotStates.theta[3], robotStates.theta[4],
            robotStates.theta[5] + 90.0 / 180.0 * pi,
            robotStates.theta[6]}; // home
        // std::vector<double> q_rad = deg2rad(q_deg);
        std::vector<Matrix4d> T_MDH = getT_MDH(q_rad); // basic fkine
        for (int i = 0; i < 8; i++) {

            // No flexibility
            Matrix4d T_cur2pre_ori = T_MDH[i];
            fk_ori = fk_ori * T_cur2pre_ori;
            // T_i2base_ori.push_back(fk_ori);
        }
        // Compute the deformation caused by flexible joints, links, and no
        // flexibility
        Vector4d flange_origin;
        flange_origin << 0, 0, 0, 1;
        Vector4d ee_pos_original = fk_ori * flange_origin; // mm
        Vector3d ee_pos = ee_pos_original.head<3>();
        // std::cout << " x from RCA   " << robotStates.flangePose[0] << ','
        //           << robotStates.flangePose[1] << ','
        //           << robotStates.flangePose[2] << std::endl;
        // std::cout << " x from motor " << ee_pos[0] << ',' << ee_pos[1] << ','
        //           << ee_pos[2] << std::endl;

        //////////////////////////////////

        // Record data
        *myfile << robotStates.flangePose[0] << ',' << robotStates.flangePose[1]
                << ',' << robotStates.flangePose[2] << ','
                << robotStates.extForceInTcpFrame[2] << ',' << robotStates.q[0]
                << ',' << robotStates.q[1] << ',' << robotStates.q[2] << ','
                << robotStates.q[3] << ',' << robotStates.q[4] << ','
                << robotStates.q[5] << ',' << robotStates.q[6] << ','
                << robotStates.theta[0] << ',' << robotStates.theta[1] << ','
                << robotStates.theta[2] << ',' << robotStates.theta[3] << ','
                << robotStates.theta[4] << ',' << robotStates.theta[5] << ','
                << robotStates.theta[6] << ',' << ee_pos[0] << ',' << ee_pos[1]
                << ',' << ee_pos[2] << '\n';
        // check max force exceed certain threshold
        if ((*move_direction) > 0
            && fabs(robotStates.extForceInTcpFrame[2])
                   > fabs(max_contact_force)) {
            (*move_direction) = -1;
            std::cout << " revert!!!!!!!!!!!!" << std::endl;
        }

        // press key
        if ((*move_direction) > 0) {

            for (int i = 0; i < k_robotDofs; i++) {
                (*dq_vector)[i] = dq[i];
                (*q_des)[i] += dq[i] * k_loopPeriod;
            }
            q_stack->push(*q_des);
            dq_stack->push(*dq_vector);
        } else if (q_stack->size() == 0) {
            // exit loop
            scheduler->stop();
        } else { // release key
            *q_des = q_stack->top();
            *dq_vector = dq_stack->top();
            for (int i = 0; i < k_robotDofs; i++) {
                (*dq_vector)[i] *= -1;
            }
            q_stack->pop();
            dq_stack->pop();
        }

        // send joint position command
        robot->streamJointPosition(*q_des, *dq_vector, q_acc);

    } catch (const flexiv::Exception& e) {
        log->error(e.what());
        scheduler->stop();
    }
}

void PressRlease(flexiv::Robot* robot, flexiv::RobotStates& robotStates,
    flexiv::Model* model, flexiv::Log* log, float desired_velocity,
    float max_contact_force, int repeat_times)
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
               << "forceZ,"
               << "q1,"
               << "q2,"
               << "q3,"
               << "q4,"
               << "q5,"
               << "q6,"
               << "q7,"
               << "theta1,"
               << "theta2,"
               << "theta3,"
               << "theta4,"
               << "theta5,"
               << "theta6,"
               << "theta7,"
               << "x_motor,"
               << "y_motor,"
               << "z_motor" << '\n';

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
        std::stack<std::vector<double>> dq_stack;
        std::vector<double> q_des = robotStates.q;
        std::vector<double> dq_vector = {0, 0, 0, 0, 0, 0, 0};
        // Switch mode for preprare PressRelease motion control
        switchMode(*robot, flexiv::MODE_JOINT_POSITION);
        flexiv::Scheduler scheduler;
        scheduler.addTask(
            std::bind(periodicTask, robot, &scheduler, log, robotStates,
                desired_velocity, max_contact_force, &move_direction, model,
                &q_des, &dq_vector, &q_stack, &dq_stack, &myfile),
            "HP periodic", 1, 45);
        // Start all added tasks, this is by default a blocking method
        scheduler.start();

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
        log.info("Executing primitive: MoveJ");
        // Send command to robot
        robot.executePrimitive(
            "MoveJ(target=-90.0 -40.0 0.0 90.0 0.0 50.0 0.0)");

        // Wait for reached target
        while (flexiv::utility::parsePtStates(
                   robot.getPrimitiveStates(), "reachedTarget")
               != "1") {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        switchMode(robot, flexiv::MODE_PRIMITIVE_EXECUTION);
        ///// MoveL to pre press position /////
        log.info("Executing primitive: MoveL");
        // Send command to robot
        robot.executePrimitive(
            "MoveL(target=0.688 -0.25 0.051 180 0 180 WORLD  WORLD_ORIGIN, "
            "maxVel=0.2, "
            "targetTolLevel=1)");

        robot.executePrimitive(
            "MoveL(target=-0.109 -0.687 0.13 180 0 90 WORLD  WORLD_ORIGIN, "
            "maxVel=0.2, "
            "preferJntPos = -90.0 -40.0 0.0 90.0 0.0 50.0 0.0,"
            "targetTolLevel=1)");

        // Wait for reached target
        while (flexiv::utility::parsePtStates(
                   robot.getPrimitiveStates(), "reachedTarget")
               != "1") {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        switchMode(robot, flexiv::MODE_JOINT_POSITION);

        // Periodic Tasks
        //=============================================================================
        flexiv::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable
        // priority

        PressRlease(&robot, robotStates, &model, &log, k_desiredVelocity,
            k_max_contact_force, repeat_times);

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 1;
    }

    return 0;
}
