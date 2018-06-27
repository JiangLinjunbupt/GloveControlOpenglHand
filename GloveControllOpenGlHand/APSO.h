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
	SUT *sut;    /*const������ܵ���const��Ա���������ܵ��÷�const��Ա�������ڳ�Ա��������public���͵�ǰ���£�*/
	MergeSort *mergesort;
	const int population;  /*��������*/
	const int iteration;   /*������*/
	float* bestPosition;  /*���ֵ��*/
	const float* posit_initializer;  //��ʼ�����ṩ������λ��

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

	/*�ж�λ��pos[]�Ƿ���Ч*/
	bool checkValidity(const float* pos);

	void swarmInit(vector<CParticle*>& swarm);
};