#include"CParticle.h"
#include"stdlib.h"  /*stdlib.h�а���rand()����*/

void CParticle::particleInit(const double* posit)    /*��ʼλ�á���ʼ�ٶȡ�fitness = 0�����弫ֵ����Ϊ����fitness_pbest = -DBL_MAX*/
{
	memcpy(position, posit, dimension * sizeof(double));

	/*����position��ά��ֵ��ʹ����Ԥ��Ĳ���ȡֵ��Χ��*/
	for (int i = 0; i != dimension; i++)
	{
		double len = rangeUp[i] - rangeLow[i];
		if (position[i] < rangeLow[i])
		{
			int n = static_cast<int>((rangeLow[i] - position[i]) / len);
			position[i] = position[i] + n*len;   /*ʹ��position[i]С��rangeLow[i]����rangeLow[i]-position[i]<len*/
			position[i] = 2 * rangeLow[i] - position[i];  /*����ǽ*/
		}
		if (position[i] > rangeUp[i])
		{
			int n = static_cast<int>((position[i] - rangeUp[i]) / len);
			position[i] = position[i] - n*len;
			position[i] = 2 * rangeUp[i] - position[i];
		}
	}

	for (int i = 0; i != dimension; i++)
		velocity[i] = (2 * ((double)(rand() % 1001) / 1000.0) - 1)*Vmax[i];     //��ʼ�ٶ���[-Vmax,Vmax]�������ڵľ��Ȳ���,�����в��õķ�ʽ

	memcpy(pbest, position, dimension * sizeof(double));

	fitness_pbest = -DBL_MAX;
	fitness = 0.0;
}

/*����жϳ���ǰ����λ�õ�fitnessֵ����pbest������øú�������pbest��fitness_pbest*/
void CParticle::setPbest(double fit)
{
	memcpy(pbest, position, dimension * sizeof(double));
	fitness_pbest = fit;
}

/*�ٶȸ��£�weight��ʾ����Ȩ�أ���CFPSOΪ�������ӣ���factor1Ϊ��֪���ӡ�factor2Ϊ�������*/
void CParticle::velocityUpdate(double weight, double factor1, double factor2, const double *gbest)
{
	for (int v = 0; v != dimension; v++)
	{
		double r1 = (double)(rand() % 1001) / 1000.0;   /*[0��1]֮��������*/
		double r2 = (double)(rand() % 1001) / 1000.0;
		velocity[v] = weight*(velocity[v] + factor1 * r1 * (pbest[v] - position[v]) + factor2 * r2 * (gbest[v] - position[v]));

		/*����ٶ����ƣ���λ������(������ǽ)���ʹ��*/
		if (velocity[v] > Vmax[v]) { velocity[v] = Vmax[v]; }
		if (velocity[v] < -1.0*Vmax[v]) { velocity[v] = -1.0*Vmax[v]; }
	}
}

void CParticle::positionUpdate()
{
	for (int v = 0; v != dimension; v++)
	{
		position[v] = position[v] + velocity[v];

		/*����ǽReflecting Walls*/
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

/*���ɸ�˹�����������N(m,sigma*sigma)����ֵΪm����׼��Ϊsigma*/
double CParticle::gaussianRand(double m, double sigma)
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
	return X * sigma + m;  //X����N(0,1)��  ת��ΪN(m,sigma*sigma)
}