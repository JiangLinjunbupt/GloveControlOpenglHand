#pragma once
#include<functional>
#include<vector>
#include <iostream>
#include<Windows.h>
#include <algorithm>
#include <fstream>
#include "CParticle.h"
#include "sut.h"
#include"extern.h"
using namespace std;

class APSO
{
public:
	SUT *sut;    /*const������ܵ���const��Ա���������ܵ��÷�const��Ա�������ڳ�Ա��������public���͵�ǰ���£�*/
	const int population;  /*��������*/
	const int iteration;   /*������*/
	float* bestPosition;  /*���ֵ��*/
	const float* posit_initializer;  //��ʼ�����ṩ������λ��
public:
	APSO(SUT *s, int popu, int iter, const float* pos_initializer) :sut(s), population(popu), iteration(iter), posit_initializer(pos_initializer)
	{
		bestPosition = new float[sut->dimension];
	};
	~APSO()
	{
		delete[]bestPosition;
	}
	void  getBestPosByEvolve();

private:
	float fCalculate(const vector<CParticle*>& swarm, const float* gbest, float gbestfit, int mode);

	/*�ж�λ��pos[]�Ƿ���Ч*/
	bool checkValidity(const float* pos);

	int fuzzyDecision(float f, int previous);

	/*���ɸ�˹�����������N(m,sigma*sigma)����ֵΪm����׼��Ϊsigma*/
	float gaussianRand(float m, float sigma);

	void swarmInit(vector<CParticle*>& swarm);
};