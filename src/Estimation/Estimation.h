#ifndef _ESTIMATION_H_
#define _ESTIMATION_H_

#include <Arduino.h>

void computeMatrixF(double T, double b, double M, double **F);
void computeMatrixG(double T, double b, double M, double eta_V, double V_p ,double **G);
void computeMatrixH(int ipr, double D, double **H);
void computeMatrixL(double T, double b, double M, double **L);
#endif