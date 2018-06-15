#include"CParticle.h"
#include"stdlib.h"  /*stdlib.h中包含rand()函数*/

void CParticle::particleInit(const float* posit)    /*初始位置、初始速度、fitness = 0、个体极值点设为自身、fitness_pbest = -DBL_MAX*/
{

	/*修正position各维度值，使落在预设的参数取值范围内*/
	for (int i = 0; i != dimension; i++)
	{
		if (posit[i] < rangeLow[i])
		{
			position[i] = rangeLow[i];
		}
		else if (posit[i] > rangeUp[i])
		{
			position[i] = rangeUp[i];
		}
		else
		{
			position[i] = posit[i];
		}
	}

	for (int i = 0; i != dimension; i++)
		velocity[i] = (2 * ((float)(rand() % 1001) / 1000.0) - 1)*Vmax[i];     //初始速度是[-Vmax,Vmax]闭区间内的均匀采样,论文中采用的方式

	memcpy(pbest, position, dimension * sizeof(float));

	fitness_pbest = -FLT_MAX;
	fitness = 0.0;
}

/*如果判断出当前粒子位置的fitness值大于pbest，则调用该函数更新pbest和fitness_pbest*/
void CParticle::setPbest(float fit)
{
	memcpy(pbest, position, dimension * sizeof(float));
	fitness_pbest = fit;
}

/*速度更新，weight表示惯性权重（对CFPSO为收敛因子）、factor1为认知因子、factor2为社会因子*/
void CParticle::velocityUpdate(float weight, float factor1, float factor2, const float *gbest)
{
	for (int v = 0; v != dimension; v++)
	{
		float r1 = (float)(rand() % 1001) / 1000.0;   /*[0、1]之间的随机数*/
		float r2 = (float)(rand() % 1001) / 1000.0;
		velocity[v] = weight*velocity[v] + factor1 * r1 * (pbest[v] - position[v] + factor2 * r2 * (gbest[v] - position[v]));

		/*最大速度限制，与位置限制(即反射墙)配合使用*/
		if (velocity[v] > Vmax[v]) { velocity[v] = Vmax[v]; }
		if (velocity[v] < -1.0*Vmax[v]) { velocity[v] = -1.0*Vmax[v]; }
	}
}

void CParticle::positionUpdate()
{
	for (int v = 0; v != dimension; v++)
	{
		position[v] = position[v] + velocity[v];

		/*反射墙Reflecting Walls*/
		if (position[v] > rangeUp[v]) { position[v] = 2 * rangeUp[v] - position[v]; }
		if (position[v] < rangeLow[v]) { position[v] = 2 * rangeLow[v] - position[v]; }
	}
}

bool CParticle::isPositionValid()
{
	for (int i = 0; i != dimension; i++)
	{
		bool inRange = (position[i] >= rangeLow[i]) && (position[i] <= rangeUp[i]);
		if (inRange == false)
		{
			return false;
		}
	}
	return true;
}

/*生成高斯随机数，服从N(m,sigma*sigma)，均值为m，标准差为sigma*/
float CParticle::gaussianRand(float m, float sigma)
{
	static double V1, V2, S;
	static int phase = 0;
	double X;
	if (phase == 0)
	{
		do
		{
			double U1 = (double)rand() / RAND_MAX;
			double U2 = (double)rand() / RAND_MAX;
			V1 = 2 * U1 - 1;
			V2 = 2 * U2 - 1;
			S = V1 * V1 + V2 * V2;
		} while (S >= 1 || S == 0);
		X = V1 * sqrt(-2 * log(S) / S);
	}
	else
	{
		X = V2 * sqrt(-2 * log(S) / S);
	}
	phase = 1 - phase;
	return X * sigma + m;  //X服从N(0,1)，  转化为N(m,sigma*sigma)
}