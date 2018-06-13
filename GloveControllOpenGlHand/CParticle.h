#pragma once
#include<iostream>
#include<vector>
#include<math.h>
using namespace std;

class CParticle
{
public:
	CParticle(int dim, const double* rangeLow, const double* rangeUp) : dimension(dim), rangeLow(rangeLow), rangeUp(rangeUp)
	{
		position = new double[dimension];
		velocity = new double[dimension];
		pbest = new double[dimension];
		Vmax = new double[dimension];
		for (int i = 0; i < dimension; i++)
		{
			Vmax[i] = (rangeUp[i] - rangeLow[i]) / 4.0;   /*论文Adaptive particle swarm optimization，IEEE Transactions中采用的值*/
		}
	};
	~CParticle()
	{
		delete[] position;
		delete[] velocity;
		delete[] pbest;
		delete[] Vmax;
	}
private:
	const int dimension;         /*the number of dimensions,类中的常数据成员只能通过构造函数的初始化表对其初始化*/
	const double *rangeLow;      /*rangeLow[dimension]数组，position空间的第i维分量的范围的最小取值*/
	const double *rangeUp;       /*rangeUp[dimension]数组，position空间的第i维分量的范围的最大取值*/
	const double *initPosition;
	double *Vmax; /*粒子速度第i维分量的范围：闭区间[-Vmax[i],Vmax[i]]*/

public:
	double *position;
	double *velocity;
	double fitness;
	double *pbest;//个体极值点
	double fitness_pbest;//个体极值

public:
	void particleInit(const double* posit);
	void setPbest(double fit);   /*如果判断出当前粒子位置的fitness值大于pbest，则调用该函数更新pbest和fitness_pbest*/
	void velocityUpdate(double weight, double factor1, double factor2, const double *gbest);   /*更新速度*/
	void positionUpdate();  /*更新位置*/
	bool isPositionValid();
private:
	double gaussianRand(double m, double sigma); //均值为m，标准差为sigma
};







