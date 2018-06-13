#pragma once
#define DBL_MAX          1.7976931348623158e+308 // max value

class SUT
{
public:
	int dimension;        /*����ά��*/
	double* rangeLow;     /*���Ż�Ŀ�꺯���Ա�����ȡֵ��Χ����iά�������ڱ�����[rangeLow[i],rangeUp[i]]*/
	double* rangeUp;
	double expected_fitness;
	double(*pointerToObjFunc)(double*, double&, double&, double&, double&);           /*Ŀ�꺯��*/
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
	/*Ҫ�Ż��������ֵ����Ŀ�꺯��*/
	double get_single_fitness(double *position, double& fitness, double& y1_penalty, double& y2_penalty, double& y3_penalty)
	{
		double value = (this->pointerToObjFunc)(position, fitness, y1_penalty, y2_penalty, y3_penalty);
		return -1 * value;  /*���Ƹ�����Ҫ�Ż�Ŀ�꺯����С����������APSO��������ֵ*/
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