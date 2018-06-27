#include"APSO.h"   //ʹ��APSO::
#include <random>

void  APSO::getBestPosByEvolve()
{

	default_random_engine e;
	uniform_real_distribution<double> u(-0.5, 0.5);

	float inertia = 0.5;    /*����Ȩ��*/
	float factor1 = 1.5;      /*��֪����*/
	float factor2 = 2.5;      /*�������*/
 
	float fit_best = -FLT_MAX;   /*FLT_MAX����float����С��ֵ��-FLT_MAX����*/

	//��������Ⱥ����������Ⱥ�е����ӳ�ʼ��
	vector<CParticle*> swarm(population);
	for (int i = 0; i != population; i++)
	{
		swarm[i] = new CParticle(sut->dimension, sut->rangeLow, sut->rangeUp);   //ά�ȡ������ռ�߽�
	}
	swarmInit(swarm);


	int it = 0;  /*��������*/
	const double sigma_max = 1.0;  /*����gbestʱ�õ��Ĳ���sigma_max��sigma_min*/
	const double sigma_min = 0.1;

	while (true)
	{

		if (it == iteration)
		{
			printf("��%d������Ⱥ��������Ӧֵfit_best�ֱ��ǣ�% 8.4lf\n", it, fit_best);
			break;
		}

		for (int i = 0; i != population; ++i)
		{
			sut->get_fitness(swarm[i]->position, ref(swarm[i]->fitness));
			this->particle_fitness_save[i] = swarm[i]->fitness;
		}

		this->mergesort->merge_sort(this->particle_fitness_save, this->population);


		//����ÿ�����ӵ�pbest��fitness_pbest��fitness������ȫ�ּ�ֵbestPosition[]���顢ȫ�ּ�ֵfit_best
		for (int j = this->population / 2 +1; j < this->population; j++)
		{
			int i = this->mergesort->index[j];
			cout << swarm[i]->fitness << endl;
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


		if (true)/*��ȫ���������ӽ���n���ݶ�����*/
		{
			const float delta = 1e-5;
			const int gradient_ascent_times = 5;  //�ݶ���������

			for (unsigned int i = 0; i != gradient_ascent_times; i++)
			{
				int dim = rand() % sut->dimension;
				cout << "ά���ǣ�" << dim << endl;
				float step = sut->rangeUp[dim] - sut->rangeLow[dim];
				step = step / 10.0 * (1 - (float)it / (float)iteration / 2.0);  //�������ŵ����������½�

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


		for (int j = this->population / 2 + 1; j < this->population; j++)
		{
			int i = this->mergesort->index[j];
			swarm[i]->velocityUpdate(inertia, factor1, factor2, bestPosition);
			swarm[i]->positionUpdate();
		}


		for (int j = 0; j < this->population+1; j++)
		{
			//�ⲿ������ʹ��bestPosition���³�ʼ��
			int i = this->mergesort->index[j];
			int v = rand() % sut->dimension;
			float tmp = bestPosition[v];
			bestPosition[v] = bestPosition[v] + (float)u(e)*(sut->rangeUp[v] - sut->rangeLow[v]);
			swarm[i]->particleInit(bestPosition);
			bestPosition[v] = tmp;
		}


		it++;
	} // end while

	for_each(swarm.begin(), swarm.end(), [](CParticle* p) {delete p; });
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



void APSO::swarmInit(vector<CParticle*>& particleSwarm)
{
	if (27 != sut->dimension)
	{
		cout << "��������Ⱥ��ʼ���ı�׼������ά������" << endl;
		exit(EXIT_FAILURE);
	}

	particleSwarm[0]->particleInit(posit_initializer);
	float* recommend = new float[sut->dimension];
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
