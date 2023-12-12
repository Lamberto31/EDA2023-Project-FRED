#include "Estimation.h"
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

// MODEL MATRICES

// bMT = T*(1 - ((b/M) * (T/2)));

/*
F = [1 bMT;
    0 1-(b/M)*bMT];
*/
void computeMatrixF(double T, double b, double M, BLA::Matrix<STATE_DIM, STATE_DIM> FF) {
    double bMT = T*(1 - ((b/M) * (T/2)));
    FF(0, 0) = 1;
    FF(0, 1) = bMT;
    FF(1, 0) = 0;
    FF(1, 1) = 1-(b/M)*bMT;
}
/*
G = ((1/M) * eta_V * (Vp/255)) * [T^2/2;
    bMT];
*/
void computeMatrixG(double T, double b, double M, double eta_V, double V_p , BLA::Matrix<STATE_DIM, INPUT_DIM> G) {
    double bMT = T*(1 - ((b/M) * (T/2)));
    G(0, 0) = (1/M) * eta_V * (V_p/255) * T*T/2;
    G(1, 0) = (1/M) * eta_V * (V_p/255) * bMT;
}
/*
H = [1 0;
    0 IPR/(pi*D)];
*/
void computeMatrixH(int ipr, double D, BLA::Matrix<MEASURE_DIM, STATE_DIM> H) {
    H(0, 0) = 1;
    H(0, 1) = 0;
    H(1, 0) = 0;
    H(1, 1) = ipr/(PI*D);
} 
/*
L = [T (1/2)*T^2;
    0 bmT];
*/
void computeMatrixL(double T, double b, double M, BLA::Matrix<STATE_DIM, INPUT_DIM> L) {
    double bMT = T*(1 - ((b/M) * (T/2)));
    L(0, 0) = T;
    L(0, 1) = (1/2)*T*T;
    L(1, 0) = 0;
    L(1, 1) = bMT;
}
/*
Q = [sigmaQP^2 0;
    0 sigmaQV^2];
*/
void computeMatrixQ(double sigmaQP, double sigmaQV, BLA::Matrix<STATE_DIM, STATE_DIM> Q) {
    Q(0, 0) = sigmaQP*sigmaQP;
    Q(0, 1) = 0;
    Q(1, 0) = 0;
    Q(1, 1) = sigmaQV*sigmaQV;
}
/*
R = [sigmaRP^2 0;
    0 sigmaRV^2];
*/
void computeMatrixR(double sigmaRP, double sigmaRV, BLA::Matrix<MEASURE_DIM, MEASURE_DIM> R) {
    R(0, 0) = sigmaRP*sigmaRP;
    R(0, 1) = 0;
    R(1, 0) = 0;
    R(1, 1) = sigmaRV*sigmaRV;
}

// INITIAL CONDITIONS
void computeMatrixP0(double sigma_0P, double sigma_0V, double **P0) {
    P0[0][0] = sigma_0P*sigma_0P;
    P0[0][1] = 0;
    P0[1][0] = 0;
    P0[1][1] = sigma_0V*sigma_0V;
}