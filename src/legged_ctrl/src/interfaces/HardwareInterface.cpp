#include "interfaces/HardwareInterface.h"

namespace legged
{
    HardwareInterface::HardwareInterface(ros::NodeHandle &_nh, const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile)
    :safe(UNITREE_LEGGED_SDK::LeggedType::Go1),
     udp(UNITREE_LEGGED_SDK::LOWLEVEL, 8090, "192.168.123.10", 8007),
     BaseInterface(_nh, taskFile, urdfFile, referenceFile) {

        // for state estimation
        pub_joint_angle = nh.advertise<sensor_msgs::JointState>("/hardware_a1/joint_foot", 100);
        joint_foot_msg.name = {"FL0", "FL1", "FL2",
                            "FR0", "FR1", "FR2",
                            "RL0", "RL1", "RL2",
                            "RR0", "RR1", "RR2",
                            "FL_foot", "FR_foot", "RL_foot", "RR_foot"};
        joint_foot_msg.position.resize(NUM_DOF + NUM_LEG);
        joint_foot_msg.velocity.resize(NUM_DOF + NUM_LEG);
        joint_foot_msg.effort.resize(NUM_DOF + NUM_LEG);
        // imu data
        pub_imu = nh.advertise<sensor_msgs::Imu>("/hardware_a1/imu", 100);     
        
        udp.InitCmdData(cmd);
        udp_init_send();

        //init swap order, very important because unitree use a different order then ours
        swap_joint_indices << 3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8;
        swap_foot_indices << 1, 0, 3, 2;

        for (int i = 0; i < NUM_LEG; i++) {
            foot_force_filters[i] = MovingWindowFilter(FOOT_FILTER_WINDOW_SIZE);
        }

        for (int i = 0; i < NUM_DOF; i++) {
            joint_vel_filters[i] = MovingWindowFilter(10);
        }
    }

    bool HardwareInterface::update(double t, double dt) {

        // let callbacks run a little bit
        if (t < 0.1) {
            legged_state.estimation_inited = false;
            return true;
        }
        if (t >= 0.1) {
            // this variable make the MPC control loop wait for sensor data to fill up and estimator runs 
            legged_state.estimation_inited = true;
        }
        bool joy_run = joy_update(t, dt);
        // debug print some variables 
        // std::cout << legged_state.joy.ctrl_state << std::endl;

        /*
        * get sensor feedback & update state estimator
        */
        receive_low_state(dt);

        bool sensor_run = sensor_update(t, dt);

        // run low level control 
        bool basic_run = tau_ctrl_update(t, dt);
            
        // bool wbc_run = wbc_update(t, dt);
        

        // std::cout << "debug"<< std::endl;
        // std::cout << legged_state.fbk.root_pos.transpose() << std::endl;
        // std::cout << legged_state.ctrl.root_pos_d.transpose() << std::endl;


        // send to hardware
        bool safe_flag = safety_checker.is_safe(legged_state);
        if (safe_flag) {
            send_cmd(t);
        } else {
            std::cout << "safety check failed, terminate the controller! " << std::endl;
        }

        return joy_run && sensor_run && basic_run && safe_flag;
    }


    bool HardwareInterface::send_cmd(double t) {
        // send control cmd to robot via unitree hardware interface
        // notice a1_ctrl_states.joint_torques uses order FL, FR, RL, RR
        // notice cmd uses order FR, FL, RR, RL

        cmd.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;
        for (int i = 0; i < NUM_DOF; i++) {
            int swap_i = swap_joint_indices(i);
            int swap_leg = swap_foot_indices(i/NUM_DOF_PER_LEG);
            cmd.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
            // cmd.motorCmd[i].q = UNITREE_LEGGED_SDK::PosStopF;
            // cmd.motorCmd[i].Kp = 0;
            // cmd.motorCmd[i].dq = UNITREE_LEGGED_SDK::VelStopF;
            // cmd.motorCmd[i].Kd = 0;
            
            cmd.motorCmd[i].q = legged_state.ctrl.joint_ang_tgt(swap_i); // shut down position control
            cmd.motorCmd[i].Kp = legged_state.param.kp_foot(swap_i%NUM_DOF_PER_LEG, swap_leg);
            cmd.motorCmd[i].dq = legged_state.ctrl.joint_vel_tgt(swap_i); // shut down velocity control
            cmd.motorCmd[i].Kd = legged_state.param.kd_foot(swap_i%NUM_DOF_PER_LEG, swap_leg);

            cmd.motorCmd[i].tau = legged_state.ctrl.joint_tau_tgt(swap_i);
        }

        if (legged_state.ctrl.movement_mode == 1) {
            
        }

        safe.PositionLimit(cmd);
        // TODO: make power level configurable
        safe.PowerProtect(cmd, unitree_state, 10);
        udp.SetSend(cmd);
        udp.Send();

        return true;
    }

    void HardwareInterface::udp_init_send() {
        cmd.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;
        for (int i = 0; i < NUM_DOF; i++) {
            cmd.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
            cmd.motorCmd[i].q = UNITREE_LEGGED_SDK::PosStopF;        // shut down position control
            cmd.motorCmd[i].Kp = 0;
            cmd.motorCmd[i].dq = UNITREE_LEGGED_SDK::VelStopF;       // shut down velocity control
            cmd.motorCmd[i].Kd = 0;
            cmd.motorCmd[i].tau = 0;
        }
        safe.PositionLimit(cmd);
        udp.SetSend(cmd);
        udp.Send();
    }

    void HardwareInterface::receive_low_state(double dt) {
        udp.Recv();
        udp.GetRecv(unitree_state);        

        // load state from unitree_state to legged_state
        legged_state.fbk.root_quat = Eigen::Quaterniond(unitree_state.imu.quaternion[0],
                                                        unitree_state.imu.quaternion[1],
                                                        unitree_state.imu.quaternion[2],
                                                        unitree_state.imu.quaternion[3]);
        legged_state.fbk.imu_acc = Eigen::Vector3d(unitree_state.imu.accelerometer[0], unitree_state.imu.accelerometer[1], unitree_state.imu.accelerometer[2]);
        legged_state.fbk.imu_ang_vel = Eigen::Vector3d(unitree_state.imu.gyroscope[0], unitree_state.imu.gyroscope[1], unitree_state.imu.gyroscope[2]);

        for (int i = 0; i < NUM_DOF; ++i) {
            int swap_i = swap_joint_indices(i);
            legged_state.fbk.joint_vel[i] = joint_vel_filters[i].CalculateAverage(unitree_state.motorState[swap_i].dq);
            // legged_state.fbk.joint_vel[i] = (unitree_state.motorState[swap_i].q - legged_state.fbk.joint_pos[i])/dt;
            legged_state.fbk.joint_pos[i] = unitree_state.motorState[swap_i].q;
            legged_state.fbk.joint_tauEst[i] = unitree_state.motorState[swap_i].tauEst;
        }

        if (legged_state.fbk.foot_force_bias_record == false) {
            for (int i = 0; i < NUM_LEG; ++i) {
                int swap_i = swap_foot_indices(i);
                legged_state.fbk.foot_force_bias[i] = unitree_state.footForce[swap_i];
            }
            legged_state.fbk.foot_force_bias_record = true;
        }
        // foot force, add a filter here
        for (int i = 0; i < NUM_LEG; ++i) {
            int swap_i = swap_foot_indices(i);
            double value = static_cast<double>(unitree_state.footForce[swap_i]-legged_state.fbk.foot_force_bias[i]);
            legged_state.fbk.foot_force[i] = foot_force_filters[i].CalculateAverage(value);
        }


        // publish state to ros for other nodes to use
        // publish joint angle and foot force
        for (int i = 0; i < NUM_DOF; ++i) {
            joint_foot_msg.position[i] = legged_state.fbk.joint_pos[i];
            joint_foot_msg.velocity[i] = legged_state.fbk.joint_vel[i];
        }
        for (int i = 0; i < NUM_LEG; ++i) {
            // publish plan contacts to help state estimation
            joint_foot_msg.velocity[NUM_DOF + i] = legged_state.ctrl.plan_contacts[i];
            joint_foot_msg.effort[NUM_DOF + i] = legged_state.fbk.foot_force[i];
        }
        imu_msg.angular_velocity.x = unitree_state.imu.gyroscope[0];
        imu_msg.angular_velocity.y = unitree_state.imu.gyroscope[1];
        imu_msg.angular_velocity.z = unitree_state.imu.gyroscope[2];

        imu_msg.linear_acceleration.x = unitree_state.imu.accelerometer[0];
        imu_msg.linear_acceleration.y = unitree_state.imu.accelerometer[1];
        imu_msg.linear_acceleration.z = unitree_state.imu.accelerometer[2]; 

        joint_foot_msg.header.stamp = ros::Time::now();
        imu_msg.header.stamp = joint_foot_msg.header.stamp;

        pub_joint_angle.publish(joint_foot_msg);
        pub_imu.publish(imu_msg);
    }


} // namespace legged