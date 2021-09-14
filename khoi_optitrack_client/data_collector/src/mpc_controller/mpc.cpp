#define _USE_MATH_DEFINES
#include "mpc_controller/mpc.hpp"
#include <iomanip>      // std::setprecision
#include <iostream>     // std::cout, std::fixed

float avg_velocity2D(std::vector<std::vector<float>> vector2D) {
    float avg = 0.0;
    for (int i = 0; i < vector2D.size(); i++) {
        avg = avg + vector2D[i][1];
    }
    return avg / vector2D.size();
}

// This is not in used right now
float avg_velocity(std::vector<float> vector_arr, int start_idx, int end_idx) {
    float avg = 0.0;

    if (start_idx == 0 && end_idx == 0 ) {
        for (int i = 0; i < vector_arr.size(); i++) {
            avg = avg + vector_arr[i];
        }
        return avg / vector_arr.size();
    }

    for (int i = start_idx; i <= end_idx; i++) {
        avg = avg + vector_arr[i];
    }
    return avg / vector_arr.size();
}

MPC_out mpc(std::vector<float>& current_Teensy_data, std::vector<float>& previous_Teensy_data, std::vector<float>& angular_vel_wheel, const int first_act_idx, MPC_out MPC_Output, std::vector<float>& ref){
// The following contains the information of the states of the system
    // x1 = theta;
    // x2 = theta_dot;
    // x3 = q (The switch state of the optodiode) {0,1}
    // x4 = DELTA (A counter at which the actuator is activated) rad
    // x5 = delta (A counter at which the actuator is deactivated) rad
    // x6 = Memory term for the input

    float hs = 0.008; // MPC computation rate
    float Ts = 0.0033333;
    //float ref = 2.7; // Reference angular velocity (rad/s), multiply to 0.15677 (r) to get m/s
    float K = 0.2; // Need tuning

    //auto start = std::chrono::high_resolution_clock::now();
    
    if (current_Teensy_data[1] - first_act_idx == 5 && current_Teensy_data[3] - previous_Teensy_data[3] == 1 && MPC_Output.first_cycle == true) {  // First: we check for if actuator index is above 5 && the wheel just switches from 0 to 1
            // if the current actuator index is the last of the 6 initial actuators
            MPC_Output.currentInput[0] = current_Teensy_data[1]; // then it should have the same input as the first 5 actuators
            MPC_Output.currentInput[1] = 3 * M_PI / 180;
            MPC_Output.currentInput[2] = 20 * M_PI / 180;
            MPC_Output.first_cycle = false;
    }

    //  Teensy transmits current_Teensy_data[1]:          1 2 3 4 5 6 7 .... 13 14 15 16
    //  current actuator idx :                            0 1 2 3 4 5 6 .... 12 13 14 15
    //  idx_pre = Current actuator idx + 1                1 2 3 4 5 6 7 .... 13 14 15  0
    MPC_Output.idx_pre = int(current_Teensy_data[1]) % 16;

    // Handling steady - state disturbance rejection % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if (abs(ref[0] - MPC_Output.vel_sim) <= 0.3 && MPC_Output.vel_sim != 0) { // If the simulated result is close to the reference and UserData.vel_sim exists, toggle the steady_state rejection
        //MPC_Output.err_ss = MPC_Output.err_ss + K * (avg_velocity(angular_vel_wheel[0], angular_vel_wheel.size() - start_idx, angular_vel_wheel.size()) - avg_velocity(MPC_Output.vel_sim,0,0));  // we need to notify the Hybrid dynamic model to account for this.
        MPC_Output.err_ss = MPC_Output.err_ss + K * (angular_vel_wheel[0]- MPC_Output.vel_sim);
        //MPC_Output.err_ss = 0; // Testing code without the error accumulator
    }
    else { MPC_Output.err_ss = 0;}
    
    // initial conditions
    int j = 10;
    float t = (current_Teensy_data[0] - current_Teensy_data[0]) / 1000000; // Obtaining the current timestamp from microcontroller(us times 1e6)
    // The hardware must toggles actuators 1 to 16 because 0 means no actuators activated
    // However, the Contact angle, the wheel angle map 0-360 degree to actuators 0 to 15, not 1 to 16. Therefore, whatever we receives from Serial for the actuator index, it must be subtracted by 1 as follow:
    std::vector<float> x_meas{ current_Teensy_data[2] * float(M_PI / 180), angular_vel_wheel[0], 1, 0, MPC_Output.currentInput[2] - MPC_Output.currentInput[1], current_Teensy_data[1] - 1};

    // Initializing predicted output from the MPC
    prdt_out optimal_out; // this replaces x_op, t_op, and j_op so check this

    // Initializing output set for data plotting
    state_out t_j_x_out;

    // Initializing output set for data plotting
    std::vector<std::vector<float>> x_sim;

    // handling initial dynamics. The index will always reset when the wheel
    // stops rolling
    int idx_actuation = 0; // This is the physical value

    if (x_meas[1] == 0.0) {
        int idx_actuation = 0; // This is the physical value
        int idx_pre = 0;
    }

    // There will be an initialization of the rotary encoder device to correct
    // for the initial angular position


    /*################################################################################*/
    /*################## Real-time MPC for the HASEL wheel ###########################*/
    float avg_x2 = 0.0; // This is what we want to be equal to the reference angular velocity
    float end_x2 = 0.0; // This is what we want to be the same as the initial value entering the MPC
    float u[3];

    /*#########################################################################*/
    /*######################### Big LOOP HERE #################################*/
    /*#########################################################################*/
    int stopping_counter;
    state_out t_j_x_sim;

    /*########################################################################*/
    /*### Simulate the dynamics to fill up the remaining input segment #######*/
    t_j_x_sim.j = j;
    t_j_x_sim.t = t;
    t_j_x_sim.xout = x_meas;
    x_sim.push_back(x_meas);
    stopping_counter = j;
    int i_sim = 1;

    //auto end = std::chrono::high_resolution_clock::now();
    //double time_taken = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count(); time_taken *= 1e-9;
    //std::cout << "Time taken is : " << std::fixed << time_taken << std::setprecision(9); std::cout << " sec" << std::endl;
    while (t_j_x_sim.j <= stopping_counter) {
        t_j_x_sim = Hybrid(t_j_x_sim.xout, MPC_Output.currentInput, t_j_x_sim.t, t_j_x_sim.j, hs, MPC_Output.err_ss); //deal with u_star here
        x_sim.push_back(t_j_x_sim.xout); i_sim = i_sim + 1;
    }
    // Use MPC for q = 0 and q = 1 (4 - 5 and 5 - 6) 
    // Compute the optimal input for the next OFF - ON - OFF cycle %
    // Check the passive decceleration result(it uses greedy code but with zero delta_a) 
    int current_sim_idx = i_sim - 1;
    optimal_out = PSO_prediction(x_sim[current_sim_idx - 1], ref[0], t_j_x_sim.t, t_j_x_sim.j - 1, 1 * M_PI / 180, 20 * M_PI / 180, hs, MPC_Output.idx_pre, MPC_Output.err_ss); // Use PSO to find the optimal solution
    if (avg_velocity2D(optimal_out.x_pre) < ref[0] + 0.3) {
        std::cout << "average velocity2D is " << avg_velocity2D(optimal_out.x_pre) << std::endl;
            // Need PSO acceleration, so do nothing here
    }
    else { // if >= ref[0] +0.3
        // Need deceleration
        if (MPC_Output.idx_pre > 14.2 || MPC_Output.idx_pre < 0.2) { //Haven't written code to handle roll-over bug for braking, so just ignore braking if idx_pre ==15
        }
        else {
            std::cout << "deceleration entered!" << std::endl;
            std::cout << MPC_Output.idx_pre << std::endl;
            MPC_Output.idx_pre = (MPC_Output.idx_pre + 1) % 16; // Must skip an index of actuation for the deceleration
            optimal_out = greedy_prediction(x_sim[current_sim_idx - 1], t_j_x_sim.t, t_j_x_sim.j - 1, -20 * M_PI / 180, -8 * M_PI / 180, hs, MPC_Output.idx_pre, 0); // Use greedy to find the optimal solution
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Obtain the average MPC velocity data(3 - 4 and 4 - 5) for late
    // Double check these two while loops down here, do we really need them?
    MPC_Output.vel_sim = optimal_out.x_pre[optimal_out.x_pre.size()-1][1];

    // Send the new input set to arduino
    MPC_Output.currentInput[0] = optimal_out.u_pre[0]; // Update the current input for the next simulation cycle
    //optimal_out.u_pre[1] = 7 * 3.14159 / 180; //Run this line if we want to fix the input command for debugging (this way the MPC doesn't do anything)
    MPC_Output.currentInput[1] = optimal_out.u_pre[1];
    MPC_Output.currentInput[2] = optimal_out.u_pre[2];

    return MPC_Output;
}