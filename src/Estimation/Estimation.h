#ifndef _ESTIMATION_H_
#define _ESTIMATION_H_

#include <Arduino.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

#define STATE_DIM 2  // [adim] Dimension of state vector
#define INPUT_DIM 1  // [adim] Dimension of input vector
#define MEASURE_DIM 2  // [adim] Dimension of measure vector

// MODEL MATRICES
void computeMatrixF(double T, double b, double M, BLA::Matrix<STATE_DIM,STATE_DIM> F);
void computeMatrixG(double T, double b, double M, double eta_V, double V_p , BLA::Matrix<STATE_DIM, INPUT_DIM> G);
void computeMatrixH(int ipr, double D, BLA::Matrix<MEASURE_DIM, STATE_DIM> H);
void computeMatrixL(double T, double b, double M, BLA::Matrix<STATE_DIM, INPUT_DIM> L);
void computeMatrixQ(double sigmaQP, double sigmaQV, BLA::Matrix<STATE_DIM, STATE_DIM> Q);
void computeMatrixR(double sigmaRP, double sigmaRV, BLA::Matrix<MEASURE_DIM, MEASURE_DIM> R);

// INITIAL CONDITIONS
void computeMatrixP0(double sigma_0P, double sigma_0V, double **P0);

#endif