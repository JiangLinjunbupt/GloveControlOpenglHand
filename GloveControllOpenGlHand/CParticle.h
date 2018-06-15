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
	int dimension;         /*the number of dimensions,���еĳ����ݳ�Աֻ��ͨ�����캯���ĳ�ʼ��������ʼ��*/
	float *rangeLow;      /*rangeLow[dimension]���飬position�ռ�ĵ�iά�����ķ�Χ����Сȡֵ*/
	float *rangeUp;       /*rangeUp[dimension]���飬position�ռ�ĵ�iά�����ķ�Χ�����ȡֵ*/
	float *initPosition;
	float *Vmax; /*�����ٶȵ�iά�����ķ�Χ��������[-Vmax[i],Vmax[i]]*/

public:
	float *position;
	float *velocity;
	float fitness;
	float *pbest;//���弫ֵ��
	float fitness_pbest;//���弫ֵ

public:
	void particleInit(const float* posit);
	void setPbest(float fit);   /*����жϳ���ǰ����λ�õ�fitnessֵ����pbest������øú�������pbest��fitness_pbest*/
	void velocityUpdate(float weight, float factor1, float factor2, const float *gbest);   /*�����ٶ�*/
	void positionUpdate();  /*����λ��*/
	bool isPositionValid();
private:
	float gaussianRand(float m, float sigma); //��ֵΪm����׼��Ϊsigma
};







