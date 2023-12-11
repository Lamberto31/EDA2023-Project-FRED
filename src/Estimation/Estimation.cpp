#include "Estimation.h"
#include <Arduino.h>


// bMT = T*(1 - ((b/M) * (T/2)));

/*
F = [1 bMT;
    0 1-(b/M)*bMT];
*/
void computeMatrixF(double T, double b, double M, double **F) {
    double bMT = T*(1 - ((b/M) * (T/2)));
    F[0][0] = 1;
    F[0][1] = bMT;
    F[1][0] = 0;
    F[1][1] = 1-(b/M)*bMT;
}
/*
G = ((1/M) * eta_V * (Vp/255)) * [T^2/2;
    bMT];
*/
void computeMatrixG(double T, double b, double M, double eta_V, double V_p ,double **G) {
    double bMT = T*(1 - ((b/M) * (T/2)));
    G[0][0] = (1/M) * eta_V * (V_p/255) * T*T/2;
    G[1][0] = (1/M) * eta_V * (V_p/255) * bMT;
}
/*
H = [1 0;
    0 IPR/(pi*D)];
*/
void computeMatrixH(int ipr, double D, double **H) {
    H[0][0] = 1;
    H[0][1] = 0;
    H[1][0] = 0;
    H[1][1] = ipr/(PI*D);
}
/*
L = [T (1/2)*T^2;
    0 bmT];
*/
void computeMatrixL(double T, double b, double M, double **L) {
    double bMT = T*(1 - ((b/M) * (T/2)));
    L[0][0] = T;
    L[0][1] = (1/2)*T*T;
    L[1][0] = 0;
    L[1][1] = bMT;
}
/*
Q = [sigmaQP^2 0;
    0 sigmaQV^2];
*/
void computeMatrixQ(double sigmaQP, double sigmaQV, double **Q) {
    Q[0][0] = sigmaQP*sigmaQP;
    Q[0][1] = 0;
    Q[1][0] = 0;
    Q[1][1] = sigmaQV*sigmaQV;
}
/*
R = [sigmaRP^2 0;
    0 sigmaRV^2];
*/
void computeMatrixR(double sigmaRP, double sigmaRV, double **R) {
    R[0][0] = sigmaRP*sigmaRP;
    R[0][1] = 0;
    R[1][0] = 0;
    R[1][1] = sigmaRV*sigmaRV;
}