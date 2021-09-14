#ifndef HYBRID_H
#define HYBRID_H

#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <math.h>
#include <vector>
#include <chrono> 
#include <algorithm>

struct state_out
{
    float t;
    int j;
    std::vector<float> xout {0,0,0,0,0,0};
};

struct prdt_out
{
    float time_t;
    int event_j;
    std::vector< std::vector<float> > x_pre;
    float u_pre[3];
    int last_idx;
};

state_out Hybrid(std::vector<float> x,float u[3],float t, int j,float h, float err_ss);

#endif