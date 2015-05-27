#ifndef _TRAJECTORIES_H_
#define _TRAJECTORIES_H_

#include <fstream> 
#include <math.h>
#include <Eigen/Dense>
#include <qpOASES.hpp>
#include "example4CP.cpp"
#include "Steps.h"
using namespace std;
using namespace Eigen;

const double	SampleTime = 0.002;
const double	SinglePhaseTimeRatio = 0.8;
const double	DoublePhaseTimeRatio = 0.2;
const double	SinglePhaseTime = StepTime * SinglePhaseTimeRatio;
const double	DoublePhaseTime = StepTime * DoublePhaseTimeRatio;


const double	PrepareTime = 1;
const double	HoldTime = 1;

const int		SinglePhaseTimeFrame = (int)(floor(SinglePhaseTime / SampleTime));
const int		DoublePhaseTimeFrame = (int)(floor(DoublePhaseTime / SampleTime));
const int		PrepareTimeFrame = (int)(floor(PrepareTime / SampleTime));
const int		HoldTimeFrame = (int)(floor(HoldTime / SampleTime));
static int		TotalTimeFrame;

const int		ZMP_SplineType = 5;				// 1: first order spline, 3: third order spline, 5: fifth order spline
const int		FootTrajectorySplineType = 3;	// 1: first order spline, 3: third order spline, 5: fifth order spline

const int		SwingLegHeight = 15;

const string	WriteFilePath = "C:\\Users\\iCeiRA_7Bot\\Desktop\\Dropbox\\Matlab\\iCeiRA_Biped_Robot\\cpp_plot\\";


struct Trajectories{
	MatrixXd zmp;
	MatrixXd com;
	MatrixXd left_foot;
	MatrixXd right_foot;

	VectorXd zmp_spline_vec;
	VectorXd foot_spline_vec;
	VectorXd foot_swing_vec;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//constructor
	Trajectories(Steps *steps);
};

string SetDiscreteZMP( Trajectories *trajectories, Steps *steps );
string SetDiscreteCOM( Trajectories *trajectories );
string SetFootTrajectory( Trajectories *trajectories, Steps *steps );
void GetSplineVec( VectorXd *spline_vec, int vec_length, int spline_type );
void GetSwingVec( VectorXd *swing_vec, int vec_length);
void TrajectoriesWriteFile( Trajectories *trajectories );
void EigenWriteFile( VectorXd matrix, string file, string path );
void EigenWriteFile( MatrixXd matrix, string file, string path );

#endif /*_TRAJECTORIES_H_*/