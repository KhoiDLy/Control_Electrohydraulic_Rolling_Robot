#define _USE_MATH_DEFINES
#include "mpc_controller/hybrid.hpp"

// Parameter definition
const double r = 0.15677;// Radius of the wheel(m)
const double L = 0.64173;// Distance from the pivot arm to the wheel center of mass(m)
const double I1 = 0.007166302;// Moment of inertia about the e1 axis(kg - m ^ 2)
const double I33 = 0.075887038;// Moment of inertia from position 33 in the tensor of the arm
const double M = 0.979176;// Weight of the wheel
const double tf = 0.0104;


////////////////////////////////////////////////////////////////
// supporting functions for computations and array manipulations
int signum(float x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

float elementWiseSum(int a[4], float b[4]) {
    float sum = 0;
    for (int i = 0; i < 4; i++) {
        sum = sum + a[i] * b[i];
    }
    return sum;
}

// Computing the actuation force
float Fa(float x0, float x1, float x2, float x5, float u1, double err_ss) {
    // Check the paper to get the updated sigma values
    float sig1 = 1.214;
    float sig2 = 1.997;
    float sig3 = 3.836;
    float sig4 = -1.004;
    float sig5 = -0.1009;
    float sig6 = 22.357;
    if (u1 < 0) { // in braking motion, the effect of activation function doesn't apply
        u1 = 0;
    }
    float phi_c = abs(x0 - x5 * 22.5*M_PI/180);
    return 1 / std::pow(phi_c,sig1) * 1 / (sig2 * x1 + sig3) * exp(-std::pow((log(x2 * phi_c) - sig4),2) / std::pow(sig5,2)) * (x2 * sig6 * (phi_c - u1)) + err_ss;
}

// Initialise derivative functions
float x0dot(float t, float x0, float x1, float x2, float x3, float x4, float x5) {
    return x1;
}
float x1dot(float t, float x0, float x1, float x2, float x3, float x4, float x5, float u1, float err_ss) {
    return 1 / (L * I1 + L * M * r*r + I33 * r*r / L) * (L * r * Fa(x0, x1, x2, x5, u1, err_ss) * sin(x0 - x5 * 22.5*M_PI/180) - (r * tf + L * tf) * signum(x1));
}
float x2dot(float t, float x0, float x1, float x2, float x3, float x4, float x5) {
    return 0;
}
float x3dot(float t, float x0, float x1, float x2, float x3, float x4, float x5) {
    return -(1 - x2) * x1;
}
float x4dot(float t, float x0, float x1, float x2, float x3, float x4, float x5) {
    return -x1;
}
float x5dot(float t, float x0, float x1, float x2, float x3, float x4, float x5) {
    return 0;
}


std::vector<float> FlowMapF(std::vector<float> x,float u[3],float t, float h, float err_ss) {

    float x0 = x[0];
    float x1 = x[1];
    float x2 = x[2];
    float x3 = x[3];
    float x4 = x[4];
    float x5 = x[5];

    float u0 = u[0];
    float u1 = u[1];
    float u2 = u[2];

    // Initialize K vectors
    float k0[4] = {0.0,0.0,0.0,0.0}; // to store K values for x1
    float k1[4] = {0.0,0.0,0.0,0.0}; // to store K values for x2
    float k2[4] = {0.0,0.0,0.0,0.0}; // to store K values for x3
    float k3[4] = {0.0,0.0,0.0,0.0}; // to store K values for x4
    float k4[4] = {0.0,0.0,0.0,0.0}; // to store K values for x5
    float k5[4] = {0.0,0.0,0.0,0.0}; // to store K values for x6
    int b[4] = {1, 2, 2, 1};   // RK4 coefficients

    std::vector<float> xplus {0.0,0.0,0.0,0.0,0.0,0.0};

    k0[0] = x0dot(t, x0, x1, x2, x3, x4, x5);
    k1[0] = x1dot(t, x0, x1, x2, x3, x4, x5, u1, err_ss);
    k2[0] = x2dot(t, x0, x1, x2, x3, x4, x5);
    k3[0] = x3dot(t, x0, x1, x2, x3, x4, x5);
    k4[0] = x4dot(t, x0, x1, x2, x3, x4, x5);
    k5[0] = x5dot(t, x0, x1, x2, x3, x4, x5);

    k0[1] = x0dot(t + (h/2), x0 + (h/2)*k0[0], x1 + (h/2)*k1[0], x2 + (h/2)*k2[0], x3 + (h/2)*k3[0], x4 + (h/2)*k4[0], x5 + (h/2)*k5[0]);
    k1[1] = x1dot(t + (h/2), x0 + (h/2)*k0[0], x1 + (h/2)*k1[0], x2 + (h/2)*k2[0], x3 + (h/2)*k3[0], x4 + (h/2)*k4[0], x5 + (h/2)*k5[0], u1, err_ss);
    k2[1] = x2dot(t + (h/2), x0 + (h/2)*k0[0], x1 + (h/2)*k1[0], x2 + (h/2)*k2[0], x3 + (h/2)*k3[0], x4 + (h/2)*k4[0], x5 + (h/2)*k5[0]);
    k3[1] = x3dot(t + (h/2), x0 + (h/2)*k0[0], x1 + (h/2)*k1[0], x2 + (h/2)*k2[0], x3 + (h/2)*k3[0], x4 + (h/2)*k4[0], x5 + (h/2)*k5[0]);
    k4[1] = x4dot(t + (h/2), x0 + (h/2)*k0[0], x1 + (h/2)*k1[0], x2 + (h/2)*k2[0], x3 + (h/2)*k3[0], x4 + (h/2)*k4[0], x5 + (h/2)*k5[0]);
    k5[1] = x5dot(t + (h/2), x0 + (h/2)*k0[0], x1 + (h/2)*k1[0], x2 + (h/2)*k2[0], x3 + (h/2)*k3[0], x4 + (h/2)*k4[0], x5 + (h/2)*k5[0]);

    k0[2] = x0dot(t + (h/2), x0 + (h/2)*k0[1], x1 + (h/2)*k1[1], x2 + (h/2)*k2[1], x3 + (h/2)*k3[1], x4 + (h/2)*k4[1], x5 + (h/2)*k5[1]);
    k1[2] = x1dot(t + (h/2), x0 + (h/2)*k0[1], x1 + (h/2)*k1[1], x2 + (h/2)*k2[1], x3 + (h/2)*k3[1], x4 + (h/2)*k4[1], x5 + (h/2)*k5[1], u1, err_ss);
    k2[2] = x2dot(t + (h/2), x0 + (h/2)*k0[1], x1 + (h/2)*k1[1], x2 + (h/2)*k2[1], x3 + (h/2)*k3[1], x4 + (h/2)*k4[1], x5 + (h/2)*k5[1]);
    k3[2] = x3dot(t + (h/2), x0 + (h/2)*k0[1], x1 + (h/2)*k1[1], x2 + (h/2)*k2[1], x3 + (h/2)*k3[1], x4 + (h/2)*k4[1], x5 + (h/2)*k5[1]);
    k4[2] = x4dot(t + (h/2), x0 + (h/2)*k0[1], x1 + (h/2)*k1[1], x2 + (h/2)*k2[1], x3 + (h/2)*k3[1], x4 + (h/2)*k4[1], x5 + (h/2)*k5[1]);
    k5[2] = x5dot(t + (h/2), x0 + (h/2)*k0[1], x1 + (h/2)*k1[1], x2 + (h/2)*k2[1], x3 + (h/2)*k3[1], x4 + (h/2)*k4[1], x5 + (h/2)*k5[1]);

    k0[3] = x0dot(t + h, x0 + h*k0[2], x1 + h*k1[2], x2 + h*k2[2], x3 + h*k3[2], x4 + h*k4[2], x5 + h*k5[2]);
    k1[3] = x1dot(t + h, x0 + h*k0[2], x1 + h*k1[2], x2 + h*k2[2], x3 + h*k3[2], x4 + h*k4[2], x5 + h*k5[2], u1, err_ss);
    k2[3] = x2dot(t + h, x0 + h*k0[2], x1 + h*k1[2], x2 + h*k2[2], x3 + h*k3[2], x4 + h*k4[2], x5 + h*k5[2]);
    k3[3] = x3dot(t + h, x0 + h*k0[2], x1 + h*k1[2], x2 + h*k2[2], x3 + h*k3[2], x4 + h*k4[2], x5 + h*k5[2]);
    k4[3] = x4dot(t + h, x0 + h*k0[2], x1 + h*k1[2], x2 + h*k2[2], x3 + h*k3[2], x4 + h*k4[2], x5 + h*k5[2]);
    k5[3] = x5dot(t + h, x0 + h*k0[2], x1 + h*k1[2], x2 + h*k2[2], x3 + h*k3[2], x4 + h*k4[2], x5 + h*k5[2]);

    xplus[0] = x0 + (h/6)*elementWiseSum(b,k0); 
    xplus[1] = x1 + (h/6)*elementWiseSum(b,k1);     
    xplus[2] = x2 + (h/6)*elementWiseSum(b,k2);
    xplus[3] = x3 + (h/6)*elementWiseSum(b,k3);      
    xplus[4] = x4 + (h/6)*elementWiseSum(b,k4);       
    xplus[5] = x5 + (h/6)*elementWiseSum(b,k5);  

    // This is to handle the angle roll over behavior (0 to 360 rolls oer)
    // input_command rolls over from 15 to 0 -> current actuator index n rolls over from 15 to 0
    // If xplus[0] doesn't roll over, it will cause error in the contact angle zeta_c which then ruins the force function Fa (because zeta_c = xplus[0] - n*22.5)
    if (xplus[0] > 360) {
        xplus[0] = xplus[0] - 360;
    }
    
    return xplus;    
}

std::vector<float> JumpMapG(std::vector<float> x, float u[3]){

    std::vector<float> xplus {0.0,0.0,0.0,0.0,0.0,0.0};

    xplus[0] = x[0];
    xplus[1] = x[1];
    xplus[2] = 1 - x[2];
    xplus[5] = x[2] * u[0] + (1 - x[2]) * x[5];

    // The current actuator idx n rolls over from 15 to 0
    // But the activation and deactivation angle down counters change based on the angular velocity and q state, not angular position nor the current actuator idx
    // q state toggles when activation and deactivation angle down counters = 0
    if (u[0] == 0) {
        xplus[3] = x[2] * (16 * M_PI / 8 + u[1] - x[0]);
        xplus[4] = 16 * M_PI / 8 + u[2] - x[0];
    }
    else {
        xplus[3] = x[2] * (u[0] * M_PI / 8 + u[1] - x[0]);
        xplus[4] = u[0] * M_PI / 8 + u[2] - x[0];
    }

    return xplus;
}


state_out Hybrid(std::vector<float> x, float u[3], float t, int j, float h, float err_ss) {

    // h is the step size of the simulation
    // t is the initial time from the previous time step
    // j0 is the initial jump event from the previous time step
    // x is the value of states from the previous time step

    float tol = 2.2e-2; // tolerance constant: This needs to be adative (at lower speed, we need lower tolerance. Higher speed require higher tolerance)
    state_out s_out;
    if (((abs(x[3]) < tol) && (abs(x[4]) > tol) && (x[2] == 0)) || ((abs(x[3]) < tol) && (abs(x[4]) < tol) && (x[2] == 1)) || ((abs(x[3]) < tol) && (abs(x[4]) < tol) && (x[2] == 0)) ) { // jump condition
        s_out.j = j + 1;
        s_out.xout = JumpMapG(x, u);
        s_out.t = t;
        return s_out;
    }
    else {
        s_out.xout = FlowMapF(x, u, t, h, err_ss);
        s_out.t = t + h;
        s_out.j = j;
        return s_out;
    }
}