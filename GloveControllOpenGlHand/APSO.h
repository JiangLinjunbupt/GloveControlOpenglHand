#pragma once
#include<functional>
#include<vector>
#include<Windows.h>
#include <algorithm>
#include <fstream>
#include"threadPool.h"
#include "CParticle.h"
#include "sut.h"
//#include"extern.h"

class APSO
{
public:
	SUT *sut;    /*const������ܵ���const��Ա���������ܵ��÷�const��Ա�������ڳ�Ա��������public���͵�ǰ���£�*/
	const int population;  /*��������*/
	const int iteration;   /*������*/
	double* bestPosition;  /*���ֵ��*/
	const double* posit_initializer;  //��ʼ�����ṩ������λ��
	const double* posit_preframe;     //��һ֡���ٽ��
public:
	APSO(SUT *s, int popu, int iter, const double* pos_initializer, const double* posit_pre) :sut(s), population(popu), iteration(iter), posit_initializer(pos_initializer), posit_preframe(posit_pre)
	{
		bestPosition = new double[sut->dimension];
	};
	~APSO()
	{
		delete[]bestPosition;
	}
	void  getBestPosByEvolve();

private:
	double fCalculate(const vector<CParticle*>& swarm, const double* gbest, double gbestfit, int mode);

	/*�ж�λ��pos[]�Ƿ���Ч*/
	bool checkValidity(const double* pos);

	int fuzzyDecision(double f, int previous);

	/*���ɸ�˹�����������N(m,sigma*sigma)����ֵΪm����׼��Ϊsigma*/
	double gaussianRand(double m, double sigma);

	void swarmInit(vector<CParticle*>& swarm);
};