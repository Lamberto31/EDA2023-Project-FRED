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
// Matrix correction if fixed position
void corretMatricesFGL(BLA::Matrix<STATE_DIM, STATE_DIM> *F, BLA::Matrix<STATE_DIM, INPUT_DIM> *G, BLA::Matrix<STATE_DIM, STATE_DIM> *L);

// INITIAL CONDITIONS
void initializeVectorX(double Xp0, double Xv0, BLA::Matrix<STATE_DIM> *X);
void initializeMatrixP(double sigma_0P, double sigma_0V, BLA::Matrix<STATE_DIM, STATE_DIM> *P);

// MODEL VECTORS
void computeVectorU(int input, BLA::Matrix<INPUT_DIM> *U);
void computeVectorZ(double d0, double pulses, BLA::Matrix<MEASURE_DIM> *Z);

// KALMAN FILTER
// Predictor
void KalmanPredictor(BLA::Matrix<STATE_DIM, STATE_DIM> F, BLA::Matrix<STATE_DIM> x_hat, BLA::Matrix<STATE_DIM, INPUT_DIM> G, BLA::Matrix<INPUT_DIM> U, BLA::Matrix<STATE_DIM, STATE_DIM> P_hat, BLA::Matrix<STATE_DIM, STATE_DIM> Q,\
                    BLA::Matrix<STATE_DIM> *x_ped, BLA::Matrix<STATE_DIM, STATE_DIM> *P_pred);
// Corrector
void KalmanCorrector(BLA::Matrix<STATE_DIM, STATE_DIM> P_pred, BLA::Matrix<MEASURE_DIM, STATE_DIM> H, BLA::Matrix<MEASURE_DIM, MEASURE_DIM> R, BLA::Matrix<MEASURE_DIM> Z, BLA::Matrix<STATE_DIM> x_pred,\
                    BLA::Matrix<STATE_DIM, STATE_DIM> *W, BLA::Matrix<STATE_DIM> *x_hat, BLA::Matrix<STATE_DIM, STATE_DIM> *P_hat, BLA::Matrix<MEASURE_DIM> *innovation, BLA::Matrix<STATE_DIM, STATE_DIM> *S);

// MATRIX SUPPORT
// Create Identity Matrix I of size dim x dim
template<int dim, typename DType = float>
void matrixIdentity(BLA::Matrix<dim, dim> *I) {
    int n = I->Rows;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (i == j) {
                I->operator()(i, j) = 1;
            } else {
                I->operator()(i, j) = 0;
            }
        }
    }
}
// Print Matrix
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
template <int rows, int cols, typename DType = float>
void bluetoothSendMatrix(BLA::Matrix<rows, cols, DType> M, const char *name, int decimals) {
  //BDT: Bluetooth Data Transmission
  Serial.println(F("BDT 1.0 MATRIX"));

  int elements = rows * cols;

  Serial.print(name);
  Serial.print(F("="));
  Serial.print(rows);
  Serial.print(F("x"));
  Serial.println(cols);

  Serial.print(F("Data:"));
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++)
    {
      Serial.print(M(i, j), decimals);
      if (i * cols + j < elements - 1)
        Serial.print(F(","));
      else
        Serial.println();
    }
  }

  Serial.println(F("BDT 1.0 END"));
}
#endif