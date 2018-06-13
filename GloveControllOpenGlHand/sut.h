#pragma once
#define DBL_MAX          1.7976931348623158e+308 // max value

class SUT
{
public:
	int dimension;        /*粒子维度*/
	double* rangeLow;     /*待优化目标函数自变量的取值范围：第i维分量属于闭区间[rangeLow[i],rangeUp[i]]*/
	double* rangeUp;
	double expected_fitness;
	double(*pointerToObjFunc)(double*, double&, double&, double&, double&);           /*目标函数*/
public:
	SUT(int dim, double *rLow, double* rUp, double(*p)(double*, double&, double&, double&, double&)) :dimension(dim), pointerToObjFunc(p), expected_fitness(DBL_MAX)
	{
		rangeLow = new double[dim];
		rangeUp = new double[dim];
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
	double get_single_fitness(double *position, double& fitness, double& y1_penalty, double& y2_penalty, double& y3_penalty)
	{
		double value = (this->pointerToObjFunc)(position, fitness, y1_penalty, y2_penalty, y3_penalty);
		return -1 * value;  /*手势跟踪需要优化目标函数最小，但程序中APSO求的是最大值*/
	}
	void cal_single_fitness_5_param(double *position, double& fitness, double& y1_penalty, double& y2_penalty, double& y3_penalty)
	{
		double value = (this->pointerToObjFunc)(position, fitness, y1_penalty, y2_penalty, y3_penalty);
		fitness = -1.0*fitness;
	}
	void cal_single_fitness_2_param(double *position, double& fitness)
	{
		double a, b, c;
		double value = (this->pointerToObjFunc)(position, fitness, a, b, c);
		fitness = -1.0*fitness;
	}
};