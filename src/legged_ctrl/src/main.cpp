// stl
#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <chrono>
#include <mutex>

// ROS
#include <ros/ros.h>
#include <ros/console.h>

#include "LeggedParams.h"
#include "LeggedState.h"
#include "interfaces/GazeboInterface.h"
#include "mpc_ctrl/ci_mpc/LciMpc.h"

std::mutex lci_init_mutex;

int main(int argc, char **argv) {
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    // change ros logger
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    
    // read ROS parameters 
    bool use_sim_time;
    int robot_type;
    int mpc_type;
    std::string run_type;
    if (ros::param::get("/use_sim_time", use_sim_time)) {
        if (use_sim_time == false) {
            std::cout << "using hardware!" << std::endl;
            run_type = std::string("hardware");
        } else {
            run_type = std::string("simulation");
        }
    } else {
        std::cout << "use_sim_time not set" << std::endl;
        return -1;
    }
    // if use_sim_time is set we assume the parameter file is loaded and these values are correct
    ros::param::get("/robot_type", robot_type);
    ros::param::get("/mpc_type", mpc_type);

    // confirm run type to let users aware of hardware experiment is happening
    std::cout << "The controller is set to control:" << std::endl;
    std::cout << "Run type: \t" << run_type << std::endl; 
    std::cout << "Controller type: \t" << mpc_type << std::endl; 
    if (use_sim_time == false) {
        std::cout <<  "Press a key to confirm and continue to start hardware run..." << std::endl;
    }
    std::cin.get();


    // Initialize OCS2 -  Since we running things in docker so absolute paths are not a problem 
    std::string taskFile = "/home/REXOperator/legged_ctrl_ws/src/legged_ctrl/config/task.info", 
                urdfFile = "/home/REXOperator/legged_ctrl_ws/src/legged_ctrl/urdf/a1_description/urdf/a1.urdf",
                referenceFile = "/home/REXOperator/legged_ctrl_ws/src/legged_ctrl/config/reference.info";

    // different interface 
    std::unique_ptr<legged::BaseInterface> intef;
    if (use_sim_time == true && robot_type == 0) {
        urdfFile = "/home/REXOperator/legged_ctrl_ws/src/legged_ctrl/urdf/a1_description/urdf/a1.urdf";
        intef = std::unique_ptr<legged::GazeboInterface>(new legged::GazeboInterface(nh, taskFile, urdfFile, referenceFile)); 

    } else if (use_sim_time == false && robot_type == 0) {
        urdfFile = "/home/REXOperator/legged_ctrl_ws/src/legged_ctrl/urdf/a1_description/urdf/a1.urdf";
        intef = std::unique_ptr<legged::GazeboInterface>(new legged::GazeboInterface(nh, taskFile, urdfFile, referenceFile)); 
        std::cout << "not implemented yet now just for testing" << std::endl;

    } else if (use_sim_time == true && robot_type == 1) {
        // TODO: use Go1 urdf
        urdfFile = "/home/REXOperator/legged_ctrl_ws/src/legged_ctrl/urdf/a1_description/urdf/a1.urdf";
        std::cout << "not implemented yet " << std::endl;
        return -1;

    } else if (use_sim_time == false && robot_type == 1) {
        // TODO: use Go1 urdf
        urdfFile = "/home/REXOperator/legged_ctrl_ws/src/legged_ctrl/urdf/a1_description/urdf/a1.urdf";
        std::cout << "not implemented yet " << std::endl;
        return -1;

    } else {
        std::cout << "undefined run type and robot type combination" << std::endl;
        return -1;
    }

    std::atomic<bool> control_execute{};
    control_execute.store(true, std::memory_order_release);

    // Thread 1: MPC
    std::cout << "Enter thread 1: MPC" << std::endl;
    std::thread MPC_thread([&]() {
        // Init LCI MPC 
        lci_init_mutex.lock();
        std::unique_ptr<legged::LciMpc> lciMpc_control = std::unique_ptr<legged::LciMpc>(new legged::LciMpc()); 
        lci_init_mutex.unlock();

        // prepare variables to monitor time and control the while loop
        ros::Time start = ros::Time::now();
        ros::Time prev = ros::Time::now();
        ros::Time now = ros::Time::now();  // bool res = app.exec();
        ros::Duration dt(0);
        ros::Duration dt_solver_time_in_ros(0);

        while (control_execute.load(std::memory_order_acquire) && ros::ok()) {
            // get t and dt
            now = ros::Time::now();
            dt = now - prev;
            prev = now;
            ros::Duration elapsed = now - start;

            auto t1 = std::chrono::high_resolution_clock::now();

            // compute desired ground forces
            bool running = true; 
            lciMpc_control->update(intef->get_legged_state(), elapsed.toSec(), dt.toSec());

            auto t2 = std::chrono::high_resolution_clock::now();
            
            std::chrono::duration<double, std::milli> ms_double = t2 - t1;
            std::cout << "MPC solution is updated in " << ms_double.count() << "ms" << std::endl;

            
            if (!running) {
                std::cout << "Thread 1 loop is terminated because of errors." << std::endl;
                jl_atexit_hook(0);
                ros::shutdown();
                std::terminate();
                break;
            }
            dt_solver_time_in_ros = ros::Time::now() - now;
            if (dt_solver_time_in_ros.toSec() < MPC_UPDATE_FREQUENCY / 1000) {    
                std::cout << "waiting...." << std::endl;
                std::cout << "waiting time :" << MPC_UPDATE_FREQUENCY - dt_solver_time_in_ros.toSec()*1000<< "ms" << std::endl;
                std::cout << "waiting...." << std::endl;
                ros::Duration( MPC_UPDATE_FREQUENCY / 1000 - dt_solver_time_in_ros.toSec() ).sleep();
            }
        }
    });


    // Thread 2: update robot states, run whole body controller, and send commands
    std::cout << "Start thread 2: update robot states, compute desired swing legs forces, compute desired joint torques, and send commands"
              << std::endl;
    std::thread main_thread([&]() {
        // prepare variables to monitor time and control the while loop
        ros::Time start = ros::Time::now();
        ros::Time prev = ros::Time::now();
        ros::Time now = ros::Time::now();  // bool res = app.exec();
        ros::Duration dt(0);

        // wait for MPC controller to load
        ros::Duration(1.5).sleep();
        lci_init_mutex.lock();
        lci_init_mutex.unlock();

        while (control_execute.load(std::memory_order_acquire) && ros::ok()) {
            auto t3 = std::chrono::high_resolution_clock::now();

            ros::Duration(LOW_LEVEL_CTRL_FREQUENCY / 1000).sleep();

            // get t and dt
            now = ros::Time::now();
            dt = now - prev;
            prev = now;
            ros::Duration elapsed = now - start;

            std::cout << "run "  << elapsed.toSec() << std::endl;
            bool main_update_running = intef->update(elapsed.toSec(), dt.toSec());
            bool send_cmd_running = true;
            
            if (!main_update_running || !send_cmd_running) {
                std::cout << "Thread 2 loop is terminated because of errors." << std::endl;
                ros::shutdown();
                std::terminate();
                break;
            }
        }
    });

    ros::AsyncSpinner spinner(12);
    spinner.start();

    main_thread.join();
    return 0;


}