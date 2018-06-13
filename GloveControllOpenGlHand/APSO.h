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
	SUT *sut;    /*const对象仅能调用const成员函数，不能调用非const成员函数（在成员函数都是public类型的前提下）*/
	const int population;  /*粒子总数*/
	const int iteration;   /*迭代数*/
	double* bestPosition;  /*最大值点*/
	const double* posit_initializer;  //初始化器提供的粒子位置
	const double* posit_preframe;     //上一帧跟踪结果
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

	/*判断位置pos[]是否有效*/
	bool checkValidity(const double* pos);

	int fuzzyDecision(double f, int previous);

	/*生成高斯随机数，服从N(m,sigma*sigma)，均值为m，标准差为sigma*/
	double gaussianRand(double m, double sigma);

	void swarmInit(vector<CParticle*>& swarm);
};