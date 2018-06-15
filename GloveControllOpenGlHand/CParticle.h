#pragma once
#include<iostream>
#include<vector>
#include<math.h>
using namespace std;

class CParticle
{
public:
	CParticle(int dim, float* rangeLow, float* rangeUp) : dimension(dim), rangeLow(rangeLow), rangeUp(rangeUp)
	{
		position = new float[dimension];
		velocity = new float[dimension];
		pbest = new float[dimension];
		Vmax = new float[dimension];
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
	int dimension;         /*the number of dimensions,类中的常数据成员只能通过构造函数的初始化表对其初始化*/
	float *rangeLow;      /*rangeLow[dimension]数组，position空间的第i维分量的范围的最小取值*/
	float *rangeUp;       /*rangeUp[dimension]数组，position空间的第i维分量的范围的最大取值*/
	float *initPosition;
	float *Vmax; /*粒子速度第i维分量的范围：闭区间[-Vmax[i],Vmax[i]]*/

public:
	float *position;
	float *velocity;
	float fitness;
	float *pbest;//个体极值点
	float fitness_pbest;//个体极值

public:
	void particleInit(const float* posit);
	void setPbest(float fit);   /*如果判断出当前粒子位置的fitness值大于pbest，则调用该函数更新pbest和fitness_pbest*/
	void velocityUpdate(float weight, float factor1, float factor2, const float *gbest);   /*更新速度*/
	void positionUpdate();  /*更新位置*/
	bool isPositionValid();
private:
	float gaussianRand(float m, float sigma); //均值为m，标准差为sigma
};







