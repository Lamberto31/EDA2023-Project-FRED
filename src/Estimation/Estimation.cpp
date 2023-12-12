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
void computeMatrixF(double T, double b, double M, BLA::Matrix<STATE_DIM, STATE_DIM> *FF) {
    double bMT = T*(1 - ((b/M) * (T/2)));
    FF->operator()(0, 0) = 1;
    FF->operator()(0, 1) = bMT;
    FF->operator()(1, 0) = 0;
    FF->operator()(1, 1) = 1-(b/M)*bMT;
}
/*
G = ((1/M) * eta_V * (Vp/255)) * [T^2/2;
    bMT];
*/
void computeMatrixG(double T, double b, double M, double eta_V, double V_p , BLA::Matrix<STATE_DIM, INPUT_DIM> *G) {
    double bMT = T*(1 - ((b/M) * (T/2)));
    G->operator()(0, 0) = (1/M) * eta_V * (V_p/255) * T*T/2;
    G->operator()(1, 0) = (1/M) * eta_V * (V_p/255) * bMT;
}
/*
H = [1 0;
    0 IPR/(pi*D)];
*/
void computeMatrixH(int ipr, double D, BLA::Matrix<MEASURE_DIM, STATE_DIM> *H) {
    H->operator()(0, 0) = 1;
    H->operator()(0, 1) = 0;
    H->operator()(1, 0) = 0;
    H->operator()(1, 1) = ipr/(PI*D);
} 
/*
L = [T (1/2)*T^2;
    0 bmT];
*/
void computeMatrixL(double T, double b, double M, BLA::Matrix<STATE_DIM, STATE_DIM> *L) {
    double bMT = T*(1 - ((b/M) * (T/2)));
    L->operator()(0, 0) = T;
    L->operator()(0, 1) = T*T/2;
    L->operator()(1, 0) = 0;
    L->operator()(1, 1) = bMT;
}
/*
Q = [sigmaQP^2 0;
    0 sigmaQV^2];
*/
void computeMatrixQ(double sigmaQP, double sigmaQV, BLA::Matrix<STATE_DIM, STATE_DIM> *Q) {
    Q->operator()(0, 0) = sigmaQP*sigmaQP;
    Q->operator()(0, 1) = 0;
    Q->operator()(1, 0) = 0;
    Q->operator()(1, 1) = sigmaQV*sigmaQV;
}
/*
R = [sigmaRP^2 0;
    0 sigmaRV^2];
*/
void computeMatrixR(double sigmaRP, double sigmaRV, BLA::Matrix<MEASURE_DIM, MEASURE_DIM> *R) {
    R->operator()(0, 0) = sigmaRP*sigmaRP;
    R->operator()(0, 1) = 0;
    R->operator()(1, 0) = 0;
    R->operator()(1, 1) = sigmaRV*sigmaRV;
}

// INITIAL CONDITIONS
void computeMatrixP0(double sigma_0P, double sigma_0V, double **P0) {
    P0[0][0] = sigma_0P*sigma_0P;
    P0[0][1] = 0;
    P0[1][0] = 0;
    P0[1][1] = sigma_0V*sigma_0V;
}

// SHOW RESULTS
void showMatrixF(BLA::Matrix<STATE_DIM, STATE_DIM> *F, int r, int c) {
    Serial.println("F = ");
    for (int i = 0; i < r; i++) {
        Serial.print("[");
        for (int j = 0; j < c; j++) {
            Serial.print(F->operator()(i, j), DECIMALS);
            if (j < c-1) {
                Serial.print(", ");
            }
        }
        Serial.println("]");
    }
    Serial.println();
}
