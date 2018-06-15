#include"APSO.h"   //使用APSO::


void  APSO::getBestPosByEvolve()
{
	float inertia = 0.9;    /*惯性权重*/
	float factor1 = 2.0;      /*认知因子*/
	float factor2 = 2.0;      /*社会因子*/
	float factor_max = 2.5;      
	float factor_min = 1.5;     
	float factorSum_max = 4.0;     

	float fit_best = -FLT_MAX;   /*FLT_MAX不是float的最小负值，-FLT_MAX才是*/

	//生成粒子群，并对粒子群中的粒子初始化
	vector<CParticle*> swarm(population);
	for (int i = 0; i != population; i++)
	{
		swarm[i] = new CParticle(sut->dimension, sut->rangeLow, sut->rangeUp);   //维度、参数空间边界
	}
	swarmInit(swarm);


	int it = 0;  /*迭代次数*/
	float f_value = 0;   /*状态因子*/
	int state = 1;        /*粒子的四个阶段，初始时状态因子为0，在搜索阶段*/
	const double sigma_max = 1.0;  /*更新gbest时用到的参数sigma_max、sigma_min*/
	const double sigma_min = 0.1;

	while (true)
	{
		for (int i = 0; i != population; ++i)
		{
			function<void()> task = bind(&SUT::get_fitness, sut, swarm[i]->position, ref(swarm[i]->fitness)); //function绑定类的非静态成员函数
			threadPool.run(task);
		}
		threadPool.ensureTaskCompleted(population);


		//计算每个粒子的pbest、fitness_pbest、fitness，更新全局极值bestPosition[]数组、全局极值fit_best
		for (int i = 0; i != population; i++)
		{
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


		/*Elitist Learning Strategy*/
		f_value = fCalculate(swarm, bestPosition, fit_best, 2);  //mode=1:全局位移L2范数+角度L1范数; mode=2：L2范数; mode=3：适应值之差的绝对值
		state = fuzzyDecision(f_value, state);

		/*如果在收敛阶段stage=3，执行额外策略以避免粒子早熟*/
		if (state == 3)
		{
			if (true)/*对全局最优粒子进行n次梯度上升*/
			{
				const float delta = 1e-5;
				const int gradient_ascent_times = 5;  //梯度上升次数
				int dim = rand() % sut->dimension;
				float step = sut->rangeUp[dim] - sut->rangeLow[dim];
				step = step / 10.0 * (1 - (float)it / (float)iteration / 2.0);  //步长随着迭代数线性下降

				for (unsigned int i = 0; i != gradient_ascent_times; i++)
				{
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
		}  //至此当前代粒子群的全局极值bestPosition[]数组、全局极值fit_best也更新完毕


		if (it == iteration)
		{
			printf("第%d代粒子群的最优适应值fit_best分别是：% 8.4lf\n", it, fit_best);
			break;
		}

		/*以下，开始新一次迭代*/
		inertia = 1.0 / (1.0 + 1.5 * exp(-2.6 * f_value));

		if (state == 1) /*探索*/
		{
			factor1 = factor1 + (0.05 + (float)(rand() % 51) / 1000.0);  /*[0.05,0.1]*/
			factor2 = factor2 - (0.05 + (float)(rand() % 51) / 1000.0);
		}
		else if (state == 2) /*开发*/
		{
			factor1 = factor1 + 0.5 * (0.05 + (float)(rand() % 51) / 1000.0);    /*[0.025,0.05]*/
			factor2 = factor2 - 0.5 * (0.05 + (float)(rand() % 51) / 1000.0);
		}
		else if (state == 3) /*收敛*/
		{
			factor1 = factor1 + 0.5 * (0.05 + (float)(rand() % 51) / 1000.0);
			factor2 = factor2 + 0.5 * (0.05 + (float)(rand() % 51) / 1000.0);
		}
		else if (state == 4)  /*跳出*/
		{
			factor1 = factor1 - (0.05 + (float)(rand() % 51) / 1000.0);
			factor2 = factor2 + (0.05 + (float)(rand() % 51) / 1000.0);
		}
		else
		{
			cout << "粒子群所处状态不合理" << endl;
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
	mode=1:全局位移L2范数+角度L1范数
	mode=2：L2范数
	mode=3：适应值之差的绝对值
	*/
	float min_dis = FLT_MAX;  /*平均距离的最小值*/
	float max_dis = -FLT_MAX;  /*平均距离的最大值*/
	float g_dis = 0;          /*全局最优粒子到其他所有粒子的平均距离*/

	int row = (int)swarm.size();
	int col = (int)swarm.size();
	float ** a = new float *[row];
	for (int i = 0; i < row; i++)
		a[i] = new float[col];

	if (mode == 1)
	{
		for (int i = 0; i < row; i++)   /*计算a[][]矩阵的上三角元素*/
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
		for (int i = 0; i < row; i++)   /*计算a[][]矩阵的上三角元素*/
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
		for (int i = 0; i < row; i++)   /*计算a[][]矩阵的上三角元素*/
		{
			for (int j = i + 1; j < col; j++)
			{
				a[i][j] = abs(swarm[i]->fitness - swarm[j]->fitness);
			}
		}
	}
	else
	{
		cout << "不存在该粒子距离计算模式" << endl;
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

	/*计算全局最优粒子到其它粒子的平均距离*/
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
		cout << "不存在该粒子距离计算模式" << endl;
		exit(EXIT_FAILURE);
	}


	if (g_dis < min_dis) { min_dis = g_dis; }
	if (g_dis > max_dis) { max_dis = g_dis; }

	/*释放空间*/
	for (int i = 0; i < row; i++)
		delete[col] a[i];
	delete[row] a;

	return (g_dis - min_dis) / (max_dis - min_dis);
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

int APSO::fuzzyDecision(float f, int previous)
{
	/*
	搜索阶段 S1 = exploration
	利用阶段 S2 = exploitation
	收敛阶段 S3 = convergence
	跳出阶段 S4 = jumping-out
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

/*生成高斯随机数，服从N(m,sigma*sigma)，均值为m，标准差为sigma*/
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
	return X * sigma + m;  //X服从N(0,1)，  转化为N(m,sigma*sigma)
}

void APSO::swarmInit(vector<CParticle*>& particleSwarm)
{
	if (27 != sut->dimension)
	{
		cout << "用于粒子群初始化的标准差数组维度有误" << endl;
		exit(EXIT_FAILURE);
	}


	float* recommend = new float[sut->dimension];

	particleSwarm[0]->particleInit(posit_initializer);
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
