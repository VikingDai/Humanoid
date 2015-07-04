#include <iostream>
#include <vector>
#include "Steps.h"
#include "Trajectories.h"
#include "Kinematics.h"
using namespace std;

void main()
{	
	//Path Planning
	Steps steps;

	CreateFootsteps( &steps );
	ReviseStepDirection( &steps );
	UpdateAllCP( &steps );
	UpdateAllZmp( &steps );


	//Trajectory Calculation
	Trajectories trajectories( &steps );
	
	DesignZmpTrajectory( &trajectories, &steps );
	DesignFootTrajectory( &trajectories, &steps );
	DesignFootDirection( &trajectories, &steps );
	DesignComTrajectory( &trajectories );
	DesignComDirection( &trajectories );
	//TrajectoryWriteFiles( &trajectories );


	//Inverse Kinematics Calculation
	Configurations configurations( &trajectories );

	DesignAngleTrunkYaw( &configurations );
	DesignAngleWaist( &configurations );
	LowerBodyInvKin( &configurations, &trajectories );
	ConfigurationWriteFiles( &configurations );


	system("pause");
}