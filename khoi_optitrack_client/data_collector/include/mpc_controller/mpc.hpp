#ifndef MODELPREDICTIVECONTROL_H
#define MODELPREDICTIVECONTROL_H

#include "mpc_controller/hybrid.hpp"
#include "mpc_controller/pso_prediction.hpp"
#include "mpc_controller/greedy_prediction.hpp"

struct MPC_out
{
    int idx_pre;
    float vel_sim;
    float currentInput[3];
    float err_ss;
    bool first_cycle;
};

MPC_out mpc(std::vector<float>& current_Teensy_data, std::vector<float>& previous_Teensy_data, std::vector<float>& angular_vel_wheel, const int first_act_idx, MPC_out MPC_Output, std::vector<float>& ref);
#endif