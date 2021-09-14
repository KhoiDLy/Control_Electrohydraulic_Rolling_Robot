#include "mpc_controller/greedy_prediction.hpp"

prdt_out greedy_prediction(std::vector<float> x, float t, int j, float theta_a,float theta_d,float hs,int idx_pre, float err_ss){

// This function will predict the state trajectory with greedy input
// There are three mode used in this function:


    int stopping_counter = j; // This, together with the while condition, prevent the hybrid simulation runs more than 2 values of j   
    int idx = 0;

    state_out t_j_x_out;
    prdt_out  predicted_out;

    std::vector<float> x0 {0,0,0,0,0,0};

    predicted_out.x_pre.push_back( x );
    predicted_out.u_pre[0] = (float) idx_pre;
    predicted_out.u_pre[1] = theta_a;
    predicted_out.u_pre[2] = theta_d;

    if (j == 0 || theta_a == theta_d) {idx = 2;}
    else {idx = 3;}
    
    t_j_x_out.t = t;
    t_j_x_out.j = j;
    t_j_x_out.xout = x;

    int i = 0;
    while (t_j_x_out.j < stopping_counter + idx){
        t_j_x_out = Hybrid(t_j_x_out.xout, predicted_out.u_pre, t_j_x_out.t, t_j_x_out.j, hs, err_ss);
        predicted_out.x_pre.push_back(t_j_x_out.xout); i++;
    }
    predicted_out.time_t = t_j_x_out.t;
    predicted_out.event_j = t_j_x_out.j;
    predicted_out.last_idx = i - 1;

    return predicted_out;
}