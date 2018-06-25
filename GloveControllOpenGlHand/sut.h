#pragma once
#define DBL_MAX          1.7976931348623158e+308 // max value
#include<float.h>
#include"extern.h"
class SUT
{
public:
	int dimension;        /*粒子维度*/
	float* rangeLow;     /*待优化目标函数自变量的取值范围：第i维分量属于闭区间[rangeLow[i],rangeUp[i]]*/
	float* rangeUp;
	float expected_fitness;
	float(*pointerToObjFunc)(float*, float&);           /*目标函数*/
public:

	SUT(int dim, float *rLow, float* rUp, float(*p)(float*, float&)) :dimension(dim), pointerToObjFunc(p), expected_fitness(FLT_MAX)
	{
		rangeLow = new float[dim];
		rangeUp = new float[dim];
		for (int i = 0; i < dim; i++)
		{
			rangeLow[i] = rLow[i];
			rangeUp[i] = rUp[i];
		}
	}

	~SUT()
	{
		delete[]rangeLow;
		delete[]rangeUp;
	}
	/*要优化（求最大值）的目标函数*/
	float get_fitness(float *position, float& fitness)
	{
		float value = (this->pointerToObjFunc)(position, fitness);
		return -1 * value;  /*手势跟踪需要优化目标函数最小，但程序中APSO求的是最大值*/
	}
};