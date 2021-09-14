#define _USE_MATH_DEFINES
#include "mpc_controller/pso_prediction.hpp"
#include <iomanip>      // std::setprecision
#include <iostream>     // std::cout, std::fixed

prdt_out PSO_prediction(std::vector<float> x, float ref, float t, int j,float Angle_min,float Angle_max,float hs,int idx_pre, float err_ss){

// This function will predict the state trajectory by choosing an optimal input inside the swarm

    // initialization
    float inertia = 0.9;
    float Convergence_tolerance = 0.01;
    int correction_factor[2] = {1, 2};

    float t_in = t;
    std::vector<float> x_in = x;
    int j_in = j;

    int stopping_counter = j;  // This, together with the while condition, prevent the hybrid simulation runs more than 2 values of j

    prdt_out predicted_out;

    std::vector<std::vector<float>> x_period;

    // set the position of the initial swarm
    std::vector<float> activation_angle;
    //std::vector<float> deactivation_angle;

    // Initializing activation_duration vector with starting angle = 0.5, step angle= 4, and ending angle = Angle_max-Angle_min-0.5
    int i = 0;
    while (Angle_min + i*(1*M_PI/180) <= (15 * M_PI / 180) ){
        activation_angle.push_back(Angle_min + i*(1*M_PI/180));
        //deactivation_angle.push_back(Angle_max);
        i++;
    }   

    // Use swarm[i][j] to access the element of swarm
    // i: the outer most dimension, same number of particles used in the algorithm
    // j: 4, corresponding to the position, the velocity, the personal best coordinates, and the personal best results
    // 
    // To access the size of the vector:
    //        swarm.size() returns the dimension of i (outer most)
    //        swarm[i].size() returns the dimension of j (inner most)
    std::vector<std::vector<float>> swarm;
    std::vector<float> fval;

    for (i = 0; i < activation_angle.size(); i++){
        swarm.push_back(std::vector<float>());
        
        swarm[i].push_back( activation_angle[i] ); // Setting up the current positions of the particless
        swarm[i].push_back( 0 );                   // These particles have initial velocities of zero
        swarm[i].push_back( activation_angle[i] ); // Setting the current personal best positions
        swarm[i].push_back( 1000 );                // Setting the current personal best result

        fval.push_back(swarm[i][3]);
    }


    float fmin = 1000;
    int igbest = 0;
    int opti_loop = 0;
    state_out t_j_x_out;
    float input[3];
    // The main loop of PSO
    while ((fmin > Convergence_tolerance) & (opti_loop < 1)){
        opti_loop++;
        // Computing the average velocity of the wheel based on the given particles
        for (i = 0; i < swarm.size(); i++){
            if ( swarm[i][0] >= Angle_min && swarm[i][0] <= 15 ){ // Making sure that new activation angle is within 1 to 9 degree (less than 1 like negative value will break the optimization) 
                t_j_x_out.t = t_in;
                t_j_x_out.xout = x_in;
                t_j_x_out.j = j_in;

                x_period.push_back(x); // Concatenating the entire x_period just to get the velocity at each time step doesn't make sense. It is better to use vector of velocity for this
               
                input[0] = idx_pre;
                input[1] = swarm[i][0];
                input[2] = Angle_max;

                while (t_j_x_out.j < stopping_counter + 3){
                    t_j_x_out = Hybrid(t_j_x_out.xout, input, t_j_x_out.t, t_j_x_out.j, hs,err_ss);
                    x_period.push_back(t_j_x_out.xout);
                }

                float sumofsquare = 0;
                for ( j = 1; j < x_period.size(); j++){    
                    sumofsquare = sumofsquare + ( x_period[j][1]-ref )*(x_period[j][1] - ref);
                }                                                    
                fval[i] = sumofsquare / (x_period.size() - 1);
                x_period.clear();

                if (fval[i] < swarm[i][3]){
                    swarm[i][2] = swarm[i][0];       // update best y postions
                    swarm[i][3] = fval[i];            // update the best value so far
                }
            }
        }

        for (int k = 0; k < swarm.size(); ++k){ // find the best function value in total
            if (fmin > swarm[k][3]){
                fmin = swarm[k][3];
                igbest = k;
            }
        }

        // update the velocity of the particles
        for (i = 0; i < swarm.size(); i++){
            swarm[i][1] = inertia*rand()*swarm[i][1] + correction_factor[0]*rand()*(swarm[i][2] - swarm[i][0]) + correction_factor[1]*rand()*(swarm[igbest][2] - swarm[i][0]);
            
            swarm[i][0] = swarm[i][0] + swarm[i][1];       // update delta_a position with the velocity and old position
        }
    }

    std::vector<std::vector<float>> x_op;
    
    predicted_out.u_pre[0] = idx_pre;
    predicted_out.u_pre[1] = swarm[igbest][2];
    predicted_out.u_pre[2] = Angle_max;
    predicted_out.x_pre.push_back(t_j_x_out.xout);

    t_j_x_out.t = t_in;
    t_j_x_out.xout = x_in;
    t_j_x_out.j = j_in;
    int idx = 1;
    while (t_j_x_out.j < stopping_counter + 3){
        t_j_x_out = Hybrid(t_j_x_out.xout, predicted_out.u_pre, t_j_x_out.t, t_j_x_out.j, hs,err_ss);
        predicted_out.x_pre.push_back(t_j_x_out.xout); idx = idx + 1;
    }

    predicted_out.time_t = t_j_x_out.t;
    predicted_out.event_j = t_j_x_out.j;
    predicted_out.last_idx = idx - 2;

    return predicted_out;
}