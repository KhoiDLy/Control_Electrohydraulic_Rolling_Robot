#ifndef PSO_PREDICTION_H
#define PSO_PREDICTION_H

#include "mpc_controller/hybrid.hpp"

prdt_out PSO_prediction(std::vector<float> x, float r, float t, int j,float Angle_min,float Angle_max,float hs,int idx_pre, float err_ss);

#endif