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
	SUT *sut;    /*const对象仅能调用const成员函数，不能调用非const成员函数（在成员函数都是public类型的前提下）*/
	const int population;  /*粒子总数*/
	const int iteration;   /*迭代数*/
	float* bestPosition;  /*最大值点*/
	const float* posit_initializer;  //初始化器提供的粒子位置
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

	/*判断位置pos[]是否有效*/
	bool checkValidity(const float* pos);

	int fuzzyDecision(float f, int previous);

	/*生成高斯随机数，服从N(m,sigma*sigma)，均值为m，标准差为sigma*/
	float gaussianRand(float m, float sigma);

	void swarmInit(vector<CParticle*>& swarm);
};