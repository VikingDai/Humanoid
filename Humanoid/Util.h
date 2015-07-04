#ifndef UTIL_H_
#define _UTIL_H_

#include <iostream>
#include <fstream>
#include <math.h>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;


double round(double number);

void EigenWriteFile( VectorXd& matrix, string file, string path );
void EigenWriteFile( MatrixXd& matrix, string file, string path );










#endif /*_UTIL_H_*/