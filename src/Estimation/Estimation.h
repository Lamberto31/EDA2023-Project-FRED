#ifndef _ESTIMATION_H_
#define _ESTIMATION_H_

#include <Arduino.h>

// MODEL MATRICES
void computeMatrixF(double T, double b, double M, double **F);
void computeMatrixG(double T, double b, double M, double eta_V, double V_p ,double **G);
void computeMatrixH(int ipr, double D, double **H);
void computeMatrixL(double T, double b, double M, double **L);
void computeMatrixQ(double sigmaQP, double sigmaQV, double **Q);
void computeMatrixR(double sigmaRP, double sigmaRV, double **R);

// INITIAL CONDITIONS
void computeMatrixP0(double sigma_0P, double sigma_0V, double **P0);

#endif