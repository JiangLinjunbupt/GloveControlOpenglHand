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
			Vmax[i] = (rangeUp[i] - rangeLow[i]) / 4.0;   /*����Adaptive particle swarm optimization��IEEE Transactions�в��õ�ֵ*/
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
	const int dimension;         /*the number of dimensions,���еĳ����ݳ�Աֻ��ͨ�����캯���ĳ�ʼ��������ʼ��*/
	const double *rangeLow;      /*rangeLow[dimension]���飬position�ռ�ĵ�iά�����ķ�Χ����Сȡֵ*/
	const double *rangeUp;       /*rangeUp[dimension]���飬position�ռ�ĵ�iά�����ķ�Χ�����ȡֵ*/
	const double *initPosition;
	double *Vmax; /*�����ٶȵ�iά�����ķ�Χ��������[-Vmax[i],Vmax[i]]*/

public:
	double *position;
	double *velocity;
	double fitness;
	double *pbest;//���弫ֵ��
	double fitness_pbest;//���弫ֵ

public:
	void particleInit(const double* posit);
	void setPbest(double fit);   /*����жϳ���ǰ����λ�õ�fitnessֵ����pbest������øú�������pbest��fitness_pbest*/
	void velocityUpdate(double weight, double factor1, double factor2, const double *gbest);   /*�����ٶ�*/
	void positionUpdate();  /*����λ��*/
	bool isPositionValid();
private:
	double gaussianRand(double m, double sigma); //��ֵΪm����׼��Ϊsigma
};







