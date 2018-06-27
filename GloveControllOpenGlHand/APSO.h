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
#include"MergeSort.h"

using namespace std;

class APSO
{
public:
	SUT *sut;    /*const对象仅能调用const成员函数，不能调用非const成员函数（在成员函数都是public类型的前提下）*/
	MergeSort *mergesort;
	const int population;  /*粒子总数*/
	const int iteration;   /*迭代数*/
	float* bestPosition;  /*最大值点*/
	const float* posit_initializer;  //初始化器提供的粒子位置

	float *particle_fitness_save;
public:
	APSO(SUT *s, int popu, int iter, const float* pos_initializer) :sut(s), population(popu), iteration(iter), posit_initializer(pos_initializer)
	{
		bestPosition = new float[sut->dimension];
		particle_fitness_save = new float[popu];
	};
	APSO(MergeSort *ms,SUT *s, int popu, int iter, const float* pos_initializer) :mergesort(ms),sut(s), population(popu), iteration(iter), posit_initializer(pos_initializer)
	{
		bestPosition = new float[sut->dimension];
	};
	~APSO()
	{
		delete[]bestPosition;
		delete[]particle_fitness_save;
	}
	void  getBestPosByEvolve();

private:

	/*判断位置pos[]是否有效*/
	bool checkValidity(const float* pos);

	void swarmInit(vector<CParticle*>& swarm);
};