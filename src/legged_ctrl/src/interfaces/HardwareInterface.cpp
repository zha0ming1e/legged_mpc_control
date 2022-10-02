#include "interfaces/HardwareInterface.h"

namespace legged
{
    HardwareInterface::HardwareInterface(ros::NodeHandle &_nh, const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile)
    :safe(UNITREE_LEGGED_SDK::LeggedType::A1), 
     udp(UNITREE_LEGGED_SDK::LOWLEVEL),
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
        std::cout << legged_state.joy.ctrl_state << std::endl;

        /*
        * get sensor feedback & update state estimator
        */
        receive_low_state();

        bool sensor_run = sensor_update(t, dt);

        // run low level control 
        bool basic_run = tau_ctrl_update(t, dt);
            
        // bool wbc_run = wbc_update(t, dt);
        
        // send to hardware
        send_cmd();

        return joy_run;
    }


    bool HardwareInterface::send_cmd() {
        // send control cmd to robot via unitree hardware interface
        // notice a1_ctrl_states.joint_torques uses order FL, FR, RL, RR
        // notice cmd uses order FR, FL, RR, RL
        cmd.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;
        for (int i = 0; i < NUM_DOF; i++) {
            cmd.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
            cmd.motorCmd[i].q = UNITREE_LEGGED_SDK::PosStopF; // shut down position control
            cmd.motorCmd[i].Kp = 0;
            cmd.motorCmd[i].dq = UNITREE_LEGGED_SDK::VelStopF; // shut down velocity control
            cmd.motorCmd[i].Kd = 0;
            int swap_i = swap_joint_indices(i);
            cmd.motorCmd[i].tau = legged_state.ctrl.joint_tau_tgt(swap_i);
        }

        safe.PositionLimit(cmd);
        // TODO: make power level configurable
        safe.PowerProtect(cmd, state, 10);
        udp.SetSend(cmd);
        udp.Send();

        return true;
    }

    void HardwareInterface::udp_init_send() {
        cmd.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;
        for (int i = 0; i < NUM_DOF; i++) {
            cmd.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
            cmd.motorCmd[i].q = UNITREE_LEGGED_SDK::PosStopF;        // 禁止位置环
            cmd.motorCmd[i].Kp = 0;
            cmd.motorCmd[i].dq = UNITREE_LEGGED_SDK::VelStopF;        // 禁止速度环
            cmd.motorCmd[i].Kd = 0;
            cmd.motorCmd[i].tau = 0;
        }
        safe.PositionLimit(cmd);
        udp.SetSend(cmd);
        udp.Send();
    }

    void HardwareInterface::receive_low_state() {
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
            legged_state.fbk.joint_vel[i] = unitree_state.motorState[swap_i].dq;
            // legged_state.fbk.joint_vel[i] = (unitree_state.motorState[swap_i].q - legged_state.fbk.joint_pos[i])/dt_s;
            legged_state.fbk.joint_pos[i] = unitree_state.motorState[swap_i].q;
        }

        // foot force, add a filter here
        for (int i = 0; i < NUM_LEG; ++i) {
            int swap_i = swap_foot_indices(i);
            double value = static_cast<double>(state.footForce[swap_i]);
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
            joint_foot_msg.velocity[NUM_DOF + i] = legged_state.fbk.plan_contacts[i];
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