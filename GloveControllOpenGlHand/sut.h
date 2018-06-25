#pragma once
#define DBL_MAX          1.7976931348623158e+308 // max value
#include<float.h>
#include"extern.h"
class SUT
{
public:
	int dimension;        /*����ά��*/
	float* rangeLow;     /*���Ż�Ŀ�꺯���Ա�����ȡֵ��Χ����iά�������ڱ�����[rangeLow[i],rangeUp[i]]*/
	float* rangeUp;
	float expected_fitness;
	float(*pointerToObjFunc)(float*, float&);           /*Ŀ�꺯��*/
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
	/*Ҫ�Ż��������ֵ����Ŀ�꺯��*/
	float get_fitness(float *position, float& fitness)
	{
		float value = (this->pointerToObjFunc)(position, fitness);
		return -1 * value;  /*���Ƹ�����Ҫ�Ż�Ŀ�꺯����С����������APSO��������ֵ*/
	}
};