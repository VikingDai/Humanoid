#ifndef _STEPS_H_
#define _STEPS_H_

#include <iostream>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include "Util.h"
using namespace std;
using namespace Eigen;


const double	pi = 3.14159265;

// Robot Parameters
const double	FootLength = 20;
const double	FootWidth = 10;

// SetAllCP() Parameters
const int		MaxPreviewSteps = 3;
const int		CP_Offset = 5;
// SetAllZmp() Pamameters
const double	StepTime = 0.8;
const double	GravityConst = 980.665;		// 9.80665 m/s2 = 980.665 cm/s2
const double	ComHeight = 100;
const double	Omega = pow(GravityConst/ComHeight, 0.5);


struct Step {
	bool		is_right_foot_;	// left: 0, right: 1
	double		direction_;
	Vector2d	position_;
	Vector2d	cp_init_;
	Vector2d	cp_end_;
	Vector2d	zmp_des_;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//constructor
	Step(bool is_right_foot, double direction, Vector2d& position);
};

typedef vector<Step*> Steps;

string SetFootsteps( Steps *steps );
string ReviseStepDirection( Steps *steps );
string SetAllCP( Steps *steps );
string SetAllZmp( Steps *steps );
void PrintAllSteps( const Steps *steps );

#endif /*_STEPS_H_*/