#include"APSO.h"   //ʹ��APSO::


void  APSO::getBestPosByEvolve()
{
	float inertia = 0.9;    /*����Ȩ��*/
	float factor1 = 2.0;      /*��֪����*/
	float factor2 = 2.0;      /*�������*/
	float factor_max = 2.5;      
	float factor_min = 1.5;     
	float factorSum_max = 4.0;     

	float fit_best = -FLT_MAX;   /*FLT_MAX����float����С��ֵ��-FLT_MAX����*/

	//��������Ⱥ����������Ⱥ�е����ӳ�ʼ��
	vector<CParticle*> swarm(population);
	for (int i = 0; i != population; i++)
	{
		swarm[i] = new CParticle(sut->dimension, sut->rangeLow, sut->rangeUp);   //ά�ȡ������ռ�߽�
	}
	swarmInit(swarm);


	int it = 0;  /*��������*/
	float f_value = 0;   /*״̬����*/
	int state = 1;        /*���ӵ��ĸ��׶Σ���ʼʱ״̬����Ϊ0���������׶�*/
	const double sigma_max = 1.0;  /*����gbestʱ�õ��Ĳ���sigma_max��sigma_min*/
	const double sigma_min = 0.1;

	while (true)
	{
		for (int i = 0; i != population; ++i)
		{
			function<void()> task = bind(&SUT::get_fitness, sut, swarm[i]->position, ref(swarm[i]->fitness)); //function����ķǾ�̬��Ա����
			threadPool.run(task);
		}
		threadPool.ensureTaskCompleted(population);


		//����ÿ�����ӵ�pbest��fitness_pbest��fitness������ȫ�ּ�ֵbestPosition[]���顢ȫ�ּ�ֵfit_best
		for (int i = 0; i != population; i++)
		{
			float fit = swarm[i]->fitness;
			if (fit >= sut->expected_fitness)
			{
				cout << "�ﵽ�����Ӧ�ȣ�ֹͣ����" << endl;
				memcpy(bestPosition, swarm[i]->position, sut->dimension * sizeof(float));
				for_each(swarm.begin(), swarm.end(), [](CParticle* p) {delete p; });  /*for_eachģ�庯����lambda���ʽ*/
				return;
			}
			if (fit > swarm[i]->fitness_pbest)  /*��������i��pbest��fitness_pbest*/
			{
				swarm[i]->setPbest(fit);
			}
			if (fit > fit_best)
			{
				fit_best = fit;
				memcpy(bestPosition, swarm[i]->position, sut->dimension * sizeof(float));
			}
		}


		/*Elitist Learning Strategy*/
		f_value = fCalculate(swarm, bestPosition, fit_best, 2);  //mode=1:ȫ��λ��L2����+�Ƕ�L1����; mode=2��L2����; mode=3����Ӧֵ֮��ľ���ֵ
		state = fuzzyDecision(f_value, state);

		/*����������׶�stage=3��ִ�ж�������Ա�����������*/
		if (state == 3)
		{
			if (true)/*��ȫ���������ӽ���n���ݶ�����*/
			{
				const float delta = 1e-5;
				const int gradient_ascent_times = 5;  //�ݶ���������
				int dim = rand() % sut->dimension;
				float step = sut->rangeUp[dim] - sut->rangeLow[dim];
				step = step / 10.0 * (1 - (float)it / (float)iteration / 2.0);  //�������ŵ����������½�

				for (unsigned int i = 0; i != gradient_ascent_times; i++)
				{
					float oldValue = bestPosition[dim];
					bestPosition[dim] += delta;  //x+delta
					float fxadd;
					sut->get_fitness(bestPosition, fxadd);
					bestPosition[dim] = oldValue;   //����λ�ûָ�ԭ����ֵ
					float grad = (fxadd - fit_best) / delta;

					bestPosition[dim] += step*grad;
					if (bestPosition[dim] > sut->rangeUp[dim])
					{
						bestPosition[dim] = sut->rangeUp[dim];
					}
					if (bestPosition[dim] < sut->rangeLow[dim])
					{
						bestPosition[dim] = sut->rangeLow[dim];
					}

					//������λ�ø��º����Ӧ��,����֮ǰ��Ӧ�ȱȽ�
					float fitness_gradascent;
					sut->get_fitness(bestPosition, fitness_gradascent);
					if (fitness_gradascent > fit_best)  //�ݶ����������ӵ���Ӧ��
					{
						fit_best = fitness_gradascent;
					}
					else
					{
						bestPosition[dim] = oldValue;
					}
				}
			}
		}  //���˵�ǰ������Ⱥ��ȫ�ּ�ֵbestPosition[]���顢ȫ�ּ�ֵfit_bestҲ�������


		if (it == iteration)
		{
			printf("��%d������Ⱥ��������Ӧֵfit_best�ֱ��ǣ�% 8.4lf\n", it, fit_best);
			break;
		}

		/*���£���ʼ��һ�ε���*/
		inertia = 1.0 / (1.0 + 1.5 * exp(-2.6 * f_value));

		if (state == 1) /*̽��*/
		{
			factor1 = factor1 + (0.05 + (float)(rand() % 51) / 1000.0);  /*[0.05,0.1]*/
			factor2 = factor2 - (0.05 + (float)(rand() % 51) / 1000.0);
		}
		else if (state == 2) /*����*/
		{
			factor1 = factor1 + 0.5 * (0.05 + (float)(rand() % 51) / 1000.0);    /*[0.025,0.05]*/
			factor2 = factor2 - 0.5 * (0.05 + (float)(rand() % 51) / 1000.0);
		}
		else if (state == 3) /*����*/
		{
			factor1 = factor1 + 0.5 * (0.05 + (float)(rand() % 51) / 1000.0);
			factor2 = factor2 + 0.5 * (0.05 + (float)(rand() % 51) / 1000.0);
		}
		else if (state == 4)  /*����*/
		{
			factor1 = factor1 - (0.05 + (float)(rand() % 51) / 1000.0);
			factor2 = factor2 + (0.05 + (float)(rand() % 51) / 1000.0);
		}
		else
		{
			cout << "����Ⱥ����״̬������" << endl;
			exit(EXIT_FAILURE);
		}

		if (factor1 > factor_max) { factor1 = factor_max; }
		if (factor1 < factor_min) { factor1 = factor_min; }
		if (factor2 > factor_max) { factor2 = factor_max; }
		if (factor2 < factor_min) { factor2 = factor_min; }

		if (factor1 + factor2 > factorSum_max)
		{
			factor1 = factorSum_max / (factor1 + factor2)*factor1;
			factor2 = factorSum_max / (factor1 + factor2)*factor2;
		}

		for (auto i : swarm)
		{
			i->velocityUpdate(inertia, factor1, factor2, bestPosition);
			i->positionUpdate();
		}

		it++;
	} // end while

	for_each(swarm.begin(), swarm.end(), [](CParticle* p) {delete p; });
}


float APSO::fCalculate(const vector<CParticle*>& swarm, const float* gbest, float gbestfit, int mode)
{
	/*
	mode=1:ȫ��λ��L2����+�Ƕ�L1����
	mode=2��L2����
	mode=3����Ӧֵ֮��ľ���ֵ
	*/
	float min_dis = FLT_MAX;  /*ƽ���������Сֵ*/
	float max_dis = -FLT_MAX;  /*ƽ����������ֵ*/
	float g_dis = 0;          /*ȫ���������ӵ������������ӵ�ƽ������*/

	int row = (int)swarm.size();
	int col = (int)swarm.size();
	float ** a = new float *[row];
	for (int i = 0; i < row; i++)
		a[i] = new float[col];

	if (mode == 1)
	{
		for (int i = 0; i < row; i++)   /*����a[][]�����������Ԫ��*/
		{
			for (int j = i + 1; j < col; j++)
			{
				float tmp = 0;
				for (int k = 0; k < 3; k++)
				{
					tmp += (swarm[i]->position[k] - swarm[j]->position[k])*(swarm[i]->position[k] - swarm[j]->position[k]);
				}
				a[i][j] = sqrt(tmp);
				for (int k = 3; k < sut->dimension; k++)
				{
					a[i][j] += abs(swarm[i]->position[k] - swarm[j]->position[k]);
				}
			}
		}
	}
	else if (mode == 2)
	{
		for (int i = 0; i < row; i++)   /*����a[][]�����������Ԫ��*/
		{
			for (int j = i + 1; j < col; j++)
			{
				float tmp = 0;
				for (int k = 0; k < sut->dimension; k++)
				{
					tmp += (swarm[i]->position[k] - swarm[j]->position[k])*(swarm[i]->position[k] - swarm[j]->position[k]);
				}
				a[i][j] = sqrt(tmp);
			}
		}
	}
	else if (mode == 3)
	{
		for (int i = 0; i < row; i++)   /*����a[][]�����������Ԫ��*/
		{
			for (int j = i + 1; j < col; j++)
			{
				a[i][j] = abs(swarm[i]->fitness - swarm[j]->fitness);
			}
		}
	}
	else
	{
		cout << "�����ڸ����Ӿ������ģʽ" << endl;
		exit(EXIT_FAILURE);
	}

	for (int i = 0; i < row; i++)
	{
		float meanDist = 0;
		for (int j = 0; j < i; j++)
			meanDist += a[j][i];
		for (int j = i + 1; j < col; j++)
			meanDist += a[i][j];
		meanDist = meanDist / (float)(col - 1);

		if (meanDist < min_dis) { min_dis = meanDist; }
		if (meanDist > max_dis) { max_dis = meanDist; }
	}

	/*����ȫ���������ӵ��������ӵ�ƽ������*/
	if (mode == 1)
	{
		for (auto i : swarm)
		{
			float tmp = 0;
			for (int k = 0; k < 3; k++)
			{
				tmp += (i->position[k] - gbest[k])*(i->position[k] - gbest[k]);
			}
			g_dis += sqrt(tmp);
			for (int k = 3; k < sut->dimension; k++)
			{
				g_dis += abs(i->position[k] - gbest[k]);
			}
		}
		g_dis = g_dis / (float)(swarm.size());
	}
	else if (mode == 2)
	{
		for (auto i : swarm)
		{
			float tmp = 0;
			for (int k = 0; k < sut->dimension; k++)
			{
				tmp += (i->position[k] - gbest[k])*(i->position[k] - gbest[k]);
			}
			g_dis += sqrt(tmp);
		}
		g_dis = g_dis / (float)(swarm.size());
	}
	else if (mode == 3)
	{
		for (auto i : swarm)
		{
			g_dis += abs(i->fitness - gbestfit);
		}
		g_dis = g_dis / (double)(swarm.size());
	}
	else
	{
		cout << "�����ڸ����Ӿ������ģʽ" << endl;
		exit(EXIT_FAILURE);
	}


	if (g_dis < min_dis) { min_dis = g_dis; }
	if (g_dis > max_dis) { max_dis = g_dis; }

	/*�ͷſռ�*/
	for (int i = 0; i < row; i++)
		delete[col] a[i];
	delete[row] a;

	return (g_dis - min_dis) / (max_dis - min_dis);
}

/*�ж�λ��pos[]�Ƿ���Ч*/
bool APSO::checkValidity(const float* pos)
{
	for (int i = 0; i < sut->dimension; i++)
	{
		bool inRange = (pos[i] >= sut->rangeLow[i]) && (pos[i] <= sut->rangeUp[i]);
		if (inRange == false)
			return false;
	}
	return true;
}

int APSO::fuzzyDecision(float f, int previous)
{
	/*
	�����׶� S1 = exploration
	���ý׶� S2 = exploitation
	�����׶� S3 = convergence
	�����׶� S4 = jumping-out
	*/
	if (f <= 0.2)
		return 3;

	if (0.2 < f && f < 0.3)
	{
		if (f < (float)7.0 / (float)30.0)
			return (previous == 1 || previous == 2) ? 2 : 3;
		else
			return (previous == 3) ? 3 : 2;
	}

	if (0.3 <= f && f <= 0.4)
		return 2;

	if (0.4 < f && f < 0.6)
	{
		if (f < 0.5)
			return (previous == 1 || previous == 4) ? 1 : 2;
		else
			return (previous == 2) ? 2 : 1;
	}

	if (0.6 <= f && f <= 0.7)
		return 1;

	if (0.7 < f && f < 0.8)
	{
		if (f < (float)23.0 / (float)30.0)
			return (previous == 4 || previous == 3) ? 4 : 1;
		else
			return  (previous == 1) ? 1 : 4;
	}

	if (f >= 0.8)
		return 4;

	return -1;
}

/*���ɸ�˹�����������N(m,sigma*sigma)����ֵΪm����׼��Ϊsigma*/
float APSO::gaussianRand(float m, float sigma)
{
	static float V1, V2, S;
	static int phase = 0;
	float X;
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

void APSO::swarmInit(vector<CParticle*>& particleSwarm)
{
	if (27 != sut->dimension)
	{
		cout << "��������Ⱥ��ʼ���ı�׼������ά������" << endl;
		exit(EXIT_FAILURE);
	}


	float* recommend = new float[sut->dimension];

	particleSwarm[0]->particleInit(posit_initializer);
	for (int i = 1; i < population; i++)
	{
		for (int v = 0; v < sut->dimension; v++)
		{
			//������׼��ʹ�þ�������������ʼ��
			recommend[v] = sut->rangeLow[v] + (float)i*(sut->rangeUp[v] - sut->rangeLow[v]) / (float)population;
		}
		particleSwarm[i]->particleInit(recommend);
	}

	delete[]recommend;

}
