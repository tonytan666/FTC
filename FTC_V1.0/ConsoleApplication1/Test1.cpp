// ConsoleApplication1.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include<math.h>
#include<thread>
#include<algorithm>
#include<Eigen/Dense>
using namespace Eigen;
using namespace std;


// second order low pass filter
class SecOrderLPF
{
public:
	double Xout[6][3];
	double Xin[6][3];//Xin[i][0] is current value, Xin[i][1] is last value, Xin[i][2] is previous value
	void GetLPFedValue(double SensorData[6])
	{
		for (int i = 0;i < 6;i++)
		{
			Xin[i][0] = SensorData[i];
			Xout[i][0] = b[0] * Xin[i][0] + b[1] * Xin[i][1] + b[2] * Xin[i][2] - (a[1] * Xout[i][1] + a[2] * Xout[i][2]);
			Xin[i][2] = Xin[i][1];
			Xin[i][1] = Xin[i][0];
			Xout[i][2] = Xout[i][1];
			Xout[i][1] = Xout[i][0];
		}
	};
private:
	const double a[3] = { 1,-1.561,0.6414 };
	const double b[3] = { 0.0201,0.0402,0.0201 };
};

//admittance model
class AdmitModel
{
public:
	double ddx[6];
	void GetCorrAcc(double dx[6], double x[6], double FTd[6]);
private:
	struct MBKParm
	{
		double M[6][6] = {  {1,0,0,0,0,0},
							{0,1,0,0,0,0 },
							{0,0,1,0,0,0},
							{0,0,0,1,0,0},
							{0,0,0,0,1,0},
							{0,0,0,0,0,1}};
		double B[6][6]= {   {1,0,0,0,0,0},
							{0,1,0,0,0,0 },
							{0,0,1,0,0,0},
							{0,0,0,1,0,0},
							{0,0,0,0,1,0},
							{0,0,0,0,0,1} };
		double K[6][6]= {   {1,0,0,0,0,0},
							{0,1,0,0,0,0 },
							{0,0,1,0,0,0},
							{0,0,0,1,0,0},
							{0,0,0,0,1,0},
							{0,0,0,0,0,1} };
	}MBKParms;
};
void AdmitModel::GetCorrAcc(double dx[6], double x[6], double FTd[6])
{
	for (int i = 0;i < 6;i++)
	{
		/*if (i != 2 || i != 5)
		{
			ddx[i] = (FTd[i] - MBKParms.K[i][i] * x[i] - MBKParms.B[i][i]*dx[i])/MBKParms.M[i][i];
		}*/
		if (i == 2 || i == 5)
		{
			continue;
			ddx[i] = (FTd[i] - MBKParms.K[i][i] * x[i] - MBKParms.B[i][i] * dx[i]) / MBKParms.M[i][i];
		}
	}
}
//angle translation
class AngleTrans
{
public:
	void Euler2Quat(double Wx, double Wy, double Wz,double dt);
	void Quat2Euler();
	double q[4];
	double w[3];
	//double dt = 0.004;//interval of the period
private:
	double theta;
};
void AngleTrans::Euler2Quat(double Wx, double Wy, double Wz,double dt)
{
	theta = sqrt(pow(Wx,2)+pow(Wy,2)+pow(Wz,2))*dt;
	q[0] = cos(theta / 2);
	q[1] = Wx * sin(theta / 2);
	q[2] = Wy * sin(theta / 2);
	q[3] = Wz * sin(theta / 2);

}
void AngleTrans::Quat2Euler()
{
	w[0] = atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (pow(q[1], 2) + pow(q[2], 2)));
	w[1] = asin(2 * (q[0] * q[2] - q[3] * q[1]));
	w[2] = atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (pow(q[2], 2) + pow(q[3], 2)));
}


int main()
{
	double FTData[6];
	double FTData0[6];
	double FTFiltedData[6];
	double RobotPos[6];
	double RobotVel[6];
	double MoveCorrPos[6];
	double MoveCorrVel[6];
	double dt = 0.004;// interval
	bool StartFlag = 0;
	bool FTInitFlag =0 ;
	bool FTInitedFlag = 0;
	double UpLimit = 5;
	double LowLimt = -5;
	//create the filter and excecute the filting
	SecOrderLPF SecOrderLPF1;
	AdmitModel AdmitModel1;
	AngleTrans AngleTrans1;

	// initialization of the sensor: calculate the offset

	// calculate the real contact data of the control
	while (true)
	{
		SecOrderLPF1.GetLPFedValue(FTData);
		for (int i = 0;i < 6;i++)
		{
			FTFiltedData[i] = SecOrderLPF1.Xout[i][0];
		}
		if (FTInitFlag)
		{
			for (int i = 0;i < 6;i++)
			{
				FTData0[i] = FTFiltedData[i];
			}
			FTInitedFlag = true;
			FTInitFlag = false;
		}
		if (FTInitedFlag && StartFlag)
		{
			for (int i = 0;i < 6;i++)
			{
				FTFiltedData[i] = FTFiltedData[i] - FTData0[i];
			}
			AdmitModel1.GetCorrAcc(RobotVel, RobotPos, FTFiltedData);
			for (int i = 0;i < 3;i++)
			{
				MoveCorrPos[i] = (AdmitModel1.ddx[i] * dt)*dt;
			}
			for (int i = 3;i < 6;i++)
			{
				MoveCorrVel[i] = AdmitModel1.ddx[i] * dt;
			}
			AngleTrans1.Euler2Quat(MoveCorrVel[3], MoveCorrVel[4], MoveCorrVel[5], dt);
			AngleTrans1.Quat2Euler();
			for (int i = 0;i < 3;i++)
			{
				MoveCorrPos[i + 3] = AngleTrans1.w[i];
			}
			// 位姿变换到法兰坐标系，如若已FT sensor为工具坐标系，则不需要变换。
			
			//
			for (int i = 0;i < 6;i++)
			{
				MoveCorrPos[i] = min(MoveCorrPos[i], UpLimit);
				MoveCorrPos[i] = max(MoveCorrPos[i], LowLimt);
			}

		}

		this_thread::sleep_for(std::chrono::milliseconds(10));
	}



	







}








// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
