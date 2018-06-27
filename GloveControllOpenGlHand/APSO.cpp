#include"APSO.h"   //使用APSO::
#include <random>

void  APSO::getBestPosByEvolve()
{

	default_random_engine e;
	uniform_real_distribution<double> u(-0.5, 0.5);

	float inertia = 0.5;    /*惯性权重*/
	float factor1 = 1.5;      /*认知因子*/
	float factor2 = 2.5;      /*社会因子*/
 
	float fit_best = -FLT_MAX;   /*FLT_MAX不是float的最小负值，-FLT_MAX才是*/

	//生成粒子群，并对粒子群中的粒子初始化
	vector<CParticle*> swarm(population);
	for (int i = 0; i != population; i++)
	{
		swarm[i] = new CParticle(sut->dimension, sut->rangeLow, sut->rangeUp);   //维度、参数空间边界
	}
	swarmInit(swarm);


	int it = 0;  /*迭代次数*/
	const double sigma_max = 1.0;  /*更新gbest时用到的参数sigma_max、sigma_min*/
	const double sigma_min = 0.1;

	while (true)
	{

		if (it == iteration)
		{
			printf("第%d代粒子群的最优适应值fit_best分别是：% 8.4lf\n", it, fit_best);
			break;
		}

		for (int i = 0; i != population; ++i)
		{
			sut->get_fitness(swarm[i]->position, ref(swarm[i]->fitness));
			this->particle_fitness_save[i] = swarm[i]->fitness;
		}

		this->mergesort->merge_sort(this->particle_fitness_save, this->population);


		//计算每个粒子的pbest、fitness_pbest、fitness，更新全局极值bestPosition[]数组、全局极值fit_best
		for (int j = this->population / 2 +1; j < this->population; j++)
		{
			int i = this->mergesort->index[j];
			cout << swarm[i]->fitness << endl;
			float fit = swarm[i]->fitness;
			if (fit >= sut->expected_fitness)
			{
				cout << "达到最大适应度，停止迭代" << endl;
				memcpy(bestPosition, swarm[i]->position, sut->dimension * sizeof(float));
				for_each(swarm.begin(), swarm.end(), [](CParticle* p) {delete p; });  /*for_each模板函数，lambda表达式*/
				return;
			}
			if (fit > swarm[i]->fitness_pbest)  /*更新粒子i的pbest、fitness_pbest*/
			{
				swarm[i]->setPbest(fit);
			}
			if (fit > fit_best)
			{
				fit_best = fit;
				memcpy(bestPosition, swarm[i]->position, sut->dimension * sizeof(float));
			}
		}


		if (true)/*对全局最优粒子进行n次梯度上升*/
		{
			const float delta = 1e-5;
			const int gradient_ascent_times = 5;  //梯度上升次数

			for (unsigned int i = 0; i != gradient_ascent_times; i++)
			{
				int dim = rand() % sut->dimension;
				cout << "维度是：" << dim << endl;
				float step = sut->rangeUp[dim] - sut->rangeLow[dim];
				step = step / 10.0 * (1 - (float)it / (float)iteration / 2.0);  //步长随着迭代数线性下降

				float oldValue = bestPosition[dim];
				bestPosition[dim] += delta;  //x+delta
				float fxadd;
				sut->get_fitness(bestPosition, fxadd);
				bestPosition[dim] = oldValue;   //粒子位置恢复原来的值
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

				//求粒子位置更新后的适应度,并与之前适应度比较
				float fitness_gradascent;
				sut->get_fitness(bestPosition, fitness_gradascent);
				if (fitness_gradascent > fit_best)  //梯度上升后粒子的适应度
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
			//这部分例子使用bestPosition重新初始化
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


/*判断位置pos[]是否有效*/
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
		cout << "用于粒子群初始化的标准差数组维度有误" << endl;
		exit(EXIT_FAILURE);
	}

	particleSwarm[0]->particleInit(posit_initializer);
	float* recommend = new float[sut->dimension];
	for (int i = 1; i < population; i++)
	{
		for (int v = 0; v < sut->dimension; v++)
		{
			//这里我准备使用均匀撒点来做初始化
			recommend[v] = sut->rangeLow[v] + (float)i*(sut->rangeUp[v] - sut->rangeLow[v]) / (float)population;
		}
		particleSwarm[i]->particleInit(recommend);
	}
	delete[]recommend;
}
