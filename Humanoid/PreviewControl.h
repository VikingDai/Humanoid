#ifndef PREIVIEWCONTROL_H
#define PREIVIEWCONTROL_H

#include<cmath>
#include<vector>
#include<iostream>

#include <Eigen/Dense>

#include "Trajectories.h"
#include "Steps.h"

using namespace std;
using namespace Eigen;


void PreviewControl(struct Trajectories *trajectories);


static double PC_Qe;
static double PC_R;
static Vector3d PC_Qx;

static Matrix3d PC_A;
static Vector3d PC_C;
static Vector3d PC_B;
static Matrix4d PC_Q;

static Matrix4d New_A;
static Vector4d New_B;
static Vector4d New_I;
static Vector3d PC_CA;

static Matrix4d I, A1, G1, Q1, A, G, Q, P, temp_IQG, temp_IQG2;
static Vector4d Gain;
static Vector4d New_X;

static double PC_Gi;
static vector<double> PC_Gs;
static vector<double> PC_Gp;

static double PC_X_Old_Up, PC_Y_Old_Up;

static vector<double> PC_Error_Zmpx, PC_Error_Zmpy;
static vector<double> PC_Sum_Error_Zmpx, PC_Sum_Error_Zmpy;
static vector<double> PC_X_Ue, PC_Y_Ue;
static vector<double> PC_X_Us, PC_Y_Us;
static vector<double> PC_X_Up, PC_Y_Up;
static vector<double> PC_Ux, PC_Uy;
static vector<double> PC_Real_Zmpx, PC_Real_Zmpy;

static double SDA_limit;

static vector<double> PC_Xp, PC_Xv, PC_Xa;
static vector<double> PC_Yp, PC_Yv, PC_Ya;

//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//PreviewControl(Trajectories *trajectories);


#endif