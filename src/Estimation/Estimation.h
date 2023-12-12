#ifndef _ESTIMATION_H_
#define _ESTIMATION_H_

#include <Arduino.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

#define STATE_DIM 2  // [adim] Dimension of state vector
#define INPUT_DIM 1  // [adim] Dimension of input vector
#define MEASURE_DIM 2  // [adim] Dimension of measure vector
#define DECIMALS 4

// MODEL MATRICES
void computeMatrixF(double T, double b, double M, BLA::Matrix<STATE_DIM,STATE_DIM> *F);
void computeMatrixG(double T, double b, double M, double eta_V, double V_p , BLA::Matrix<STATE_DIM, INPUT_DIM> *G);
void computeMatrixH(int ipr, double D, BLA::Matrix<MEASURE_DIM, STATE_DIM> *H);
void computeMatrixL(double T, double b, double M, BLA::Matrix<STATE_DIM, STATE_DIM> *L);
void computeMatrixQ(double sigmaQP, double sigmaQV, BLA::Matrix<STATE_DIM, STATE_DIM> *Q);
void computeMatrixR(double sigmaRP, double sigmaRV, BLA::Matrix<MEASURE_DIM, MEASURE_DIM> *R);

// INITIAL CONDITIONS
void computeMatrixP0(double sigma_0P, double sigma_0V, double **P0);

// SHOW RESULTS
void showMatrixF(BLA::Matrix<STATE_DIM, STATE_DIM> *F, int r, int c);

template <int rows, int cols, typename DType = float>
void printMatrix(BLA::Matrix<rows, cols, DType> M, const char *name, int decimals) {
    Serial.print(name);
    Serial.println(" = ");
    for (int i = 0; i < M.Rows; i++) {
        Serial.print("[");
        for (int j = 0; j < M.Cols; j++) {
            Serial.print(M(i, j), decimals);
            if (j < M.Cols-1) {
                Serial.print(", ");
            }
        }
        Serial.println("]");
    }
    Serial.println();
}

#endif