#include "PreviewControl.h"

void PreviewControl(Trajectories *trajectories)
{
	//In MATLAB called function [Gi Gs Gp] = preview_method(A,B,C,Q,R,T_Sample,Preview_Point)
	
	PC_Qe = 1;
	PC_R = 0.000001;
	PC_Qx << 0, 0, 0;

	PC_A << 1, SamplingTime, SamplingTime * SamplingTime / 2,
			0,	1, SamplingTime,
			0, 0, 1;

	PC_B << pow(SamplingTime, 3) / 6, SamplingTime * SamplingTime / 2, SamplingTime;

	PC_Q << PC_Qe, 0, 0, 0,
			0, PC_Qx(0), 0, 0,
			0, 0, PC_Qx(1), 0,
			0, 0, 0, PC_Qx(2);


	PC_C << 1, 0, -ComHeight / GravityConst;

	PC_CA.transpose() = PC_C.transpose() * PC_A; // 1*3 * 3*3 = 1*3 transpose -> 3*1

	New_A << 1, PC_CA(0), PC_CA(1), PC_CA(2),
			0, 1, SamplingTime, SamplingTime*SamplingTime / 2,
			0, 0, 1, SamplingTime,
			0, 0, 0, 1;

	New_B << PC_C.transpose() * PC_B, PC_B(0), PC_B(1), PC_B(2);

	New_I << 1, 0, 0, 0;

	I << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	//Solving for Riccati equation, equivalent to the the MATLAB function dare.
	A1 = New_A;
	G1 = New_B * pow(PC_R, -1) * New_B.transpose();
	Q1 = PC_Q;
	SDA_limit = 20;

	for (int i = 0; i < SDA_limit; i++)
	{
		A = A1 * (I + G1 * Q1).inverse() * A1;
		temp_IQG = I + Q1 * G1;
		temp_IQG2 = temp_IQG.inverse();
		G = G1 + A1 * G1 * temp_IQG2*A1.transpose();
		Q = Q1 + A1.transpose() * temp_IQG2 * Q1 * A1;

		A1 = A;
		G1 = G;
		Q1 = Q;

	}

	P = Q;
	// Here we get Riccati Gain and find Gi Gs Gp.
	Gain.transpose() = pow(PC_R + New_B.transpose() * P * New_B, -1) * New_B.transpose() * P * New_A;
	// Gi
	PC_Gi = Gain(0);
	// Gs
	for (int i = 0; i < 3; i++) PC_Gs.push_back(Gain(i + 1));
	// Gp
	New_A = New_A - New_B * Gain.transpose();

	for (int i = 0; i < trajectories->TotalTimeFrame; i++)
	{
		if (i == 0)
		{
			PC_Gp.push_back(-1 * PC_Gi);
			New_X = -1 * New_A.transpose() * P * New_I;
		}
		else
		{
			PC_Gp.push_back(pow(PC_R + New_B.transpose() * P * New_B, -1) * New_B.transpose() * New_X);
			New_X = New_A.transpose() * New_X;
		}
	}
	/* In MATLAB called  function [ PC_COGX PC_COGY ] = preview_control( PC_Desired_Zmpx, 
	PC_Desired_Zmpy, PC_Sample_Time, PC_Sum_Sample_Point, PC_Preview_Period, PC_COG_Z_Const, 
	PC_COG_Origin) */

	// Set up initial COM x,y 
	PC_Xp.push_back(0.5*(trajectories->left_foot(0) + trajectories->right_foot(0)));
	PC_Xv.push_back(0);
	PC_Xa.push_back(0);
	PC_Yp.push_back(0.5*(trajectories->left_foot(1) + trajectories->right_foot(1)));
	PC_Yv.push_back(0);
	PC_Ya.push_back(0);
	PC_Ux.push_back(0);
	PC_Uy.push_back(0);
	
	for(int i = 0; i < trajectories->TotalTimeFrame; i++)
	{

		if(i == 0)
		{
			PC_Real_Zmpx.push_back(PC_C(0) * PC_Xp.back() + PC_C(1) * PC_Xv.back() + PC_C(2) * PC_Xa.back());
			PC_Real_Zmpy.push_back(PC_C(0) * PC_Yp.back() + PC_C(1) * PC_Yv.back() + PC_C(2) * PC_Ya.back());
			
			PC_Error_Zmpx.push_back(PC_Real_Zmpx.back() - trajectories->zmp(0, i));
			PC_Error_Zmpy.push_back(PC_Real_Zmpy.back() - trajectories->zmp(1, i));
			PC_Sum_Error_Zmpx.push_back(PC_Error_Zmpx.back());
			PC_Sum_Error_Zmpy.push_back(PC_Error_Zmpy.back());
			
			PC_X_Ue.push_back(PC_Gi * PC_Sum_Error_Zmpx.back());
			PC_Y_Ue.push_back(PC_Gi * PC_Sum_Error_Zmpy.back());
			PC_X_Us.push_back(PC_Gs[0] * PC_Xp.back() + PC_Gs[1] * PC_Xv.back() + PC_Gs[2] * PC_Xa.back());
			PC_Y_Us.push_back(PC_Gs[0] * PC_Yp.back() + PC_Gs[1] * PC_Yv.back() + PC_Gs[2] * PC_Ya.back());
			
			PC_X_Old_Up = 0;
			PC_Y_Old_Up = 0;
			
			for(int j = 0; j < HoldTimeFrame; j++)
			{
				PC_X_Old_Up += PC_Gp[j] * trajectories->zmp(0, i+j);
				PC_Y_Old_Up += PC_Gp[j] * trajectories->zmp(1, i+j);
			}

			PC_X_Up.push_back(PC_X_Old_Up);
			PC_Y_Up.push_back(PC_Y_Old_Up);
			
			PC_Xp.push_back(PC_Xp.back() + PC_Xv.back() * SamplingTime + PC_Xa.back() * pow(SamplingTime, 2) / 2 + PC_Ux.back()*pow(SamplingTime, 3) / 6);
			PC_Xv.push_back(PC_Xv.back() + PC_Xa.back() * SamplingTime + PC_Ux.back()*pow(SamplingTime, 2) / 2);
			PC_Xa.push_back(PC_Xa.back() + PC_Ux.back() * SamplingTime);
			
			PC_Yp.push_back(PC_Yp.back() + PC_Yv.back() * SamplingTime + PC_Ya.back() * pow(SamplingTime, 2) / 2 + PC_Uy.back()*pow(SamplingTime, 3) / 6);
			PC_Yv.push_back(PC_Yv.back() + PC_Ya.back() * SamplingTime + PC_Uy.back() * pow(SamplingTime, 2) / 2);
			PC_Ya.push_back(PC_Ya.back() + PC_Uy.back() * SamplingTime);
			
			PC_Ux.push_back(-PC_X_Ue.back() - PC_X_Us.back() - PC_X_Up.back());
			PC_Uy.push_back(-PC_Y_Ue.back() - PC_Y_Us.back() - PC_Y_Up.back());
		}
		else
		{
			
			PC_Real_Zmpx.push_back(PC_C(0)*PC_Xp.back()+PC_C(1)*PC_Xv.back()+PC_C(2)*PC_Xa.back());
			PC_Real_Zmpy.push_back(PC_C(0)*PC_Yp.back()+PC_C(1)*PC_Yv.back()+PC_C(2)*PC_Ya.back());
			
			PC_Error_Zmpx.push_back(PC_Real_Zmpx.back() - trajectories->zmp(0, i));
			PC_Error_Zmpy.push_back(PC_Real_Zmpy.back() - trajectories->zmp(1, i));
			
			PC_Sum_Error_Zmpx.push_back(PC_Error_Zmpx.back()+PC_Sum_Error_Zmpx.back());
			PC_Sum_Error_Zmpy.push_back(PC_Error_Zmpy.back()+PC_Sum_Error_Zmpy.back());
			
			PC_X_Ue.push_back(PC_Gi * PC_Sum_Error_Zmpx.back());
			PC_Y_Ue.push_back(PC_Gi * PC_Sum_Error_Zmpy.back());
			PC_X_Us.push_back(PC_Gs[0] * PC_Xp.back() + PC_Gs[1] * PC_Xv.back() + PC_Gs[2] * PC_Xa.back());
			PC_Y_Us.push_back(PC_Gs[0] * PC_Yp.back() + PC_Gs[1] * PC_Yv.back() + PC_Gs[2] * PC_Ya.back());
			PC_X_Old_Up=0;
			PC_Y_Old_Up=0;

			for (int j = 0; j < HoldTimeFrame; j++)
			{
				if(j==0)
				{
					PC_X_Old_Up += PC_Gp[j] * trajectories->zmp(0, i+j);
					PC_Y_Old_Up += PC_Gp[j] * trajectories->zmp(1, i+j);
				}
				else
				{
					if(i < trajectories->TotalTimeFrame - HoldTimeFrame - 1)
					{
						PC_X_Old_Up += PC_Gp[j] * trajectories->zmp(0, i + j);
						PC_Y_Old_Up += PC_Gp[j] * trajectories->zmp(1, i + j);
					}
					else
					{
						PC_X_Old_Up=PC_X_Up.back();
						PC_Y_Old_Up=PC_Y_Up.back();
					}
				}
				
			}

			PC_X_Up.push_back(PC_X_Old_Up);
			PC_Y_Up.push_back(PC_Y_Old_Up);

			if(i < trajectories->TotalTimeFrame - 1)
			{
				PC_Xp.push_back(PC_Xp.back() + PC_Xv.back()*SamplingTime + PC_Xa.back()*pow(SamplingTime,2)/2 + PC_Ux.back()*pow(SamplingTime,3)/6);
				PC_Xv.push_back(PC_Xv.back() + PC_Xa.back()*SamplingTime + PC_Ux.back()*pow(SamplingTime, 2) / 2);
				PC_Xa.push_back(PC_Xa.back() + PC_Ux.back()*SamplingTime);
				
				PC_Yp.push_back(PC_Yp.back() + PC_Yv.back()*SamplingTime + PC_Ya.back()*pow(SamplingTime, 2) / 2 + PC_Uy.back()*pow(SamplingTime, 3) / 6);
				PC_Yv.push_back(PC_Yv.back() + PC_Ya.back()*SamplingTime + PC_Uy.back()*pow(SamplingTime, 2) / 2);
				PC_Ya.push_back(PC_Ya.back() + PC_Uy.back()*SamplingTime);
				
				PC_Ux.push_back(-PC_X_Ue.back()-PC_X_Us.back()-PC_X_Up.back());
				PC_Uy.push_back(-PC_Y_Ue.back()-PC_Y_Us.back()-PC_Y_Up.back());
			}

			trajectories->com(0, i) = PC_Xp[i];
			trajectories->com(1, i) = PC_Yp[i];
			trajectories->com.row(2).setConstant(ComHeight);
		}

		// 不加這個會有error...
		trajectories->com(0, 0) = trajectories->com(0, 1);
		trajectories->com(1, 0) = trajectories->com(1, 1);

		
	}

}
