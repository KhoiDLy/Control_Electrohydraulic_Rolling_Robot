#ifndef GREEDY_PREDICTION_H
#define GREEDY_PREDICTION_H

#include "mpc_controller/hybrid.hpp"
prdt_out greedy_prediction(std::vector<float> x, float t, int j, float theta_a,float delta_a,float hs,int idx_pre, float err_ss);

#endif