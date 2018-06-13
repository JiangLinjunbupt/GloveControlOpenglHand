#include"APSO.h"   //使用APSO::


void  APSO::getBestPosByEvolve()
{
	double inertia = 0.9;    /*惯性权重*/
	double factor1 = 2;      /*认知因子*/
	double factor2 = 2;      /*社会因子*/
	double factor_max = 2.5;
	double factor_min = 1.5;
	double factorSum_max = 4;

	double fit_best = -DBL_MAX;   /*DBL_MIN不是double的最小负值，-DBL_MAX才是*/

								  //生成粒子群，并对粒子群中的粒子初始化
	vector<CParticle*> swarm(population);
	for (int i = 0; i != population; i++)
	{
		swarm[i] = new CParticle(sut->dimension, sut->rangeLow, sut->rangeUp);   //维度、参数空间边界
	}
	swarmInit(swarm);

	if (false)
	{
		cout << "粒子初始化结果(仅显示10个粒子的position)：" << endl;
		for (int n = 0; n != 10; n++)
		{
			cout << "粒子" << n << ":" << endl;
			for (int i = 0; i < sut->dimension; i++)
			{
				printf("  % 8.4lf", swarm[n]->position[i]);
				if (i == 5 || i == 8 || i == 11 || i == 14 || i == 17 || i == 20)
				{
					cout << endl;
				}
			}
		}
	}

	int it = 0;  /*迭代次数*/
	double f_value = 0;   /*状态因子*/
	int state = 1;        /*粒子的四个阶段，初始时状态因子为0，在搜索阶段*/
	const double sigma_max = 1.0;  /*更新gbest时用到的参数sigma_max、sigma_min*/
	const double sigma_min = 0.1;

	double y1_penalty, y2_penalty, y3_penalty;  //目标函数的第一、二、三项取值
	unique_ptr<double[]> y1_pen = make_unique<double[]>(population);//等价于new double[population]
	unique_ptr<double[]> y2_pen = make_unique<double[]>(population);
	unique_ptr<double[]> y3_pen = make_unique<double[]>(population);
	while (true)
	{
		for (int i = 0; i != population; ++i)
		{
			function<void()> task = bind(&SUT::cal_single_fitness_5_param, sut, swarm[i]->position, ref(swarm[i]->fitness), ref(y1_pen[i]), ref(y2_pen[i]), ref(y3_pen[i])); //function绑定类的非静态成员函数
			threadPool.run(task);
		}
		threadPool.ensureTaskCompleted(population);


		//计算每个粒子的pbest、fitness_pbest、fitness，更新全局极值bestPosition[]数组、全局极值fit_best
		for (int i = 0; i != population; i++)
		{
			double fit = swarm[i]->fitness;
			if (fit >= sut->expected_fitness)
			{
				cout << "达到最大适应度，停止迭代" << endl;
				memcpy(bestPosition, swarm[i]->position, sut->dimension * sizeof(double));
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
				memcpy(bestPosition, swarm[i]->position, sut->dimension * sizeof(double));
				y1_penalty = y1_pen[i];
				y2_penalty = y2_pen[i];
				y3_penalty = y3_pen[i];
			}
		}


		/*Elitist Learning Strategy*/
		f_value = fCalculate(swarm, bestPosition, fit_best, 1);  //mode=1:全局位移L2范数+角度L1范数; mode=2：L2范数; mode=3：适应值之差的绝对值
		state = fuzzyDecision(f_value, state);

		if (state == 4) { inJumpout_times++; }
		/*如果在收敛阶段stage=3，执行额外策略以避免粒子早熟*/
		if (state == 3)
		{
			inConvergenc_times++;

			if (false)
			{
				double *gbest_tmp = new double[sut->dimension];
				memcpy(gbest_tmp, bestPosition, sut->dimension * sizeof(double));

				/*原始APSO之更新gbest*/
				int dim = (int)(((double)(rand() % 1000) / 1000.0) * sut->dimension);  /*任选一个维度*/
				double gr = gaussianRand(0, sigma_max - (sigma_max - sigma_min) * ((double)it / (double)iteration));
				gbest_tmp[dim] = gbest_tmp[dim] + (sut->rangeUp[dim] - sut->rangeLow[dim]) * gr;
				if (gbest_tmp[dim] > sut->rangeUp[dim]) gbest_tmp[dim] = sut->rangeUp[dim];
				if (gbest_tmp[dim] <sut->rangeLow[dim]) gbest_tmp[dim] = sut->rangeLow[dim];

				double fit, y1_pen, y2_pen, y3_pen;
				double fit_tmp = sut->get_single_fitness(gbest_tmp, fit, y1_pen, y2_pen, y3_pen);
				if (fit_tmp > fit_best && checkValidity(gbest_tmp))
				{
					fit_best = fit_tmp;
					memcpy(bestPosition, gbest_tmp, sut->dimension * sizeof(double));
					y1_penalty = y1_pen;
					y2_penalty = y2_pen;
					y3_penalty = y3_pen;

					performELS_times++;
				}

				delete[] gbest_tmp;
			}
			if (false)
			{
				double *gbest_tmp = new double[sut->dimension];
				memcpy(gbest_tmp, bestPosition, sut->dimension * sizeof(double));

				//将手部参数向量的自由度分成全局位移、全局角度、关节角度3类，分别随机选取3类自由度中的某个维度进行更新
				int dim[3];
				dim[0] = (int)(((double)(rand() % 1000) / 1000.0) * 3);        /*全局位移[0][1][2]中任选一个维度*/
				dim[1] = 3 + (int)(((double)(rand() % 1000) / 1000.0) * 3);      /*全局角度[3][4][5]中任选一个维度*/
				dim[2] = 6 + (int)(((double)(rand() % 1000) / 1000.0) * 15);    /*关节角度[6][7]……[20]中任选一个维度*/
				for (int i = 0; i != 3; i++)
				{
					double gr = gaussianRand(0, (sigma_max - (sigma_max - sigma_min) * ((double)it / (double)iteration)));
					gbest_tmp[dim[i]] = gbest_tmp[dim[i]] + (sut->rangeUp[dim[i]] - sut->rangeLow[dim[i]]) * gr;/*sut->value[dim] - 1*/
					if (gbest_tmp[dim[i]] > sut->rangeUp[dim[i]]) { gbest_tmp[dim[i]] = sut->rangeUp[dim[i]]; }
					if (gbest_tmp[dim[i]] < sut->rangeLow[dim[i]]) { gbest_tmp[dim[i]] = sut->rangeLow[dim[i]]; }
				}

				double fit, y1_pen, y2_pen, y3_pen;
				double fit_tmp = sut->get_single_fitness(gbest_tmp, fit, y1_pen, y2_pen, y3_pen);
				if (fit_tmp > fit_best && checkValidity(gbest_tmp))
				{
					fit_best = fit_tmp;
					memcpy(bestPosition, gbest_tmp, sut->dimension * sizeof(double));
					y1_penalty = y1_pen;
					y2_penalty = y2_pen;
					y3_penalty = y3_pen;

					performELS_times++;
				}

				delete[] gbest_tmp;
			}
			if (true)/*对全局最优粒子进行n次梯度上升*/
			{
				const double delta = 1e-5;
				const int gradient_ascent_times = 5;  //梯度上升次数
				int dim = rand() % sut->dimension;
				double step = sut->rangeUp[dim] - sut->rangeLow[dim];
				step = step / 10 * (1 - (double)it / iteration / 2.0);  //步长随着迭代数线性下降

				for (unsigned int i = 0; i != gradient_ascent_times; i++)
				{
					double oldValue = bestPosition[dim];
					bestPosition[dim] += delta;  //x+delta
					double fxadd;
					sut->cal_single_fitness_2_param(bestPosition, fxadd);
					bestPosition[dim] = oldValue;   //粒子位置恢复原来的值
					double grad = (fxadd - fit_best) / delta;

					bestPosition[dim] += step*grad;
					if (bestPosition[dim] > sut->rangeUp[dim])
					{
						bestPosition[dim] = sut->rangeUp[dim];
						gradascent_outofbounds_times++;
					}
					if (bestPosition[dim] < sut->rangeLow[dim])
					{
						bestPosition[dim] = sut->rangeLow[dim];
						gradascent_outofbounds_times++;
					}

					//求粒子位置更新后的适应度,并与之前适应度比较
					double fitness_gradascent, y1_pen_gradascent, y2_pen_gradascent, y3_pen_gradascent;
					sut->cal_single_fitness_5_param(bestPosition, fitness_gradascent, y1_pen_gradascent, y2_pen_gradascent, y3_pen_gradascent);
					if (fitness_gradascent > fit_best)  //梯度上升后粒子的适应度
					{
						fit_best = fitness_gradascent;
						y1_penalty = y1_pen_gradascent;
						y2_penalty = y2_pen_gradascent;
						y3_penalty = y3_pen_gradascent;

						performELS_times++;
					}
					else
					{
						bestPosition[dim] = oldValue;
					}
				}
			}
		}  //至此当前代粒子群的全局极值bestPosition[]数组、全局极值fit_best也更新完毕

		   //printf("第%d代中最优粒子的penalty，y1、y2、y3：% 10.4f  % 10.4f % 10.4f\n", it, y1_penalty, y2_penalty, y3_penalty);
		if (false)
		{
			cout << "对应的全局极值点为：" << endl;
			for (int i = 0; i < sut->dimension; i++)
			{
				printf("  % 10.4f", bestPosition[i]);
				if (i == 5 || i == 8 || i == 11 || i == 14 || i == 17 || i == 20)
				{
					cout << endl;
				}
			}
			cout << endl << endl;
		}

		double error = cal_average_joints_tracking_error_from_partiposit(4, bestPosition);//当前代最优粒子对应的5关节平均误差
		if (it != 0 && it != iteration)
		{
			file_each_itera_fitbest << fit_best << ",";
			file_each_itera_joints_error << error << ",";
		}
		if (it == iteration)
		{
			file_each_itera_fitbest << fit_best << "\n";
			file_each_itera_joints_error << error << "\n";
			file_last_itera_fitbest_and_error << -fit_best << ",";
			printf("第%d代粒子群的最优适应值fit_best、y1、y2、y3分别是：% 8.4lf,  % 8.4lf,  % 8.4lf,  % 8.4lf\n", it, fit_best, y1_penalty, y2_penalty, y3_penalty);
			break;
		}

		/*以下，开始新一次迭代*/
		inertia = 1.0 / (1.0 + 1.5 * exp(-2.6 * f_value));

		if (state == 1) /*探索*/
		{
			factor1 = factor1 + (0.05 + (double)(rand() % 51) / 1000.0);  /*[0.05,0.1]*/
			factor2 = factor2 - (0.05 + (double)(rand() % 51) / 1000.0);
		}
		else if (state == 2) /*开发*/
		{
			factor1 = factor1 + 0.5 * (0.05 + (double)(rand() % 51) / 1000.0);    /*[0.025,0.05]*/
			factor2 = factor2 - 0.5 * (0.05 + (double)(rand() % 51) / 1000.0);
		}
		else if (state == 3) /*收敛*/
		{
			factor1 = factor1 + 0.5 * (0.05 + (double)(rand() % 51) / 1000.0);
			factor2 = factor2 + 0.5 * (0.05 + (double)(rand() % 51) / 1000.0);
		}
		else if (state == 4)  /*跳出*/
		{
			factor1 = factor1 - (0.05 + (double)(rand() % 51) / 1000.0);
			factor2 = factor2 + (0.05 + (double)(rand() % 51) / 1000.0);
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


double APSO::fCalculate(const vector<CParticle*>& swarm, const double* gbest, double gbestfit, int mode)
{
	/*
	mode=1:全局位移L2范数+角度L1范数
	mode=2：L2范数
	mode=3：适应值之差的绝对值
	*/
	double min_dis = DBL_MAX;  /*平均距离的最小值*/
	double max_dis = -DBL_MAX;  /*平均距离的最大值*/
	double g_dis = 0;          /*全局最优粒子到其他所有粒子的平均距离*/

	int row = (int)swarm.size();
	int col = (int)swarm.size();
	double ** a = new double *[row];
	for (int i = 0; i < row; i++)
		a[i] = new double[col];

	if (mode == 1)
	{
		for (int i = 0; i < row; i++)   /*计算a[][]矩阵的上三角元素*/
		{
			for (int j = i + 1; j < col; j++)
			{
				double tmp = 0;
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
				double tmp = 0;
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
		double meanDist = 0;
		for (int j = 0; j < i; j++)
			meanDist += a[j][i];
		for (int j = i + 1; j < col; j++)
			meanDist += a[i][j];
		meanDist = meanDist / (double)(col - 1);

		if (meanDist < min_dis) { min_dis = meanDist; }
		if (meanDist > max_dis) { max_dis = meanDist; }
	}

	/*计算全局最优粒子到其它粒子的平均距离*/
	if (mode == 1)
	{
		for (auto i : swarm)
		{
			double tmp = 0;
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
		g_dis = g_dis / (double)(swarm.size());
	}
	else if (mode == 2)
	{
		for (auto i : swarm)
		{
			double tmp = 0;
			for (int k = 0; k < sut->dimension; k++)
			{
				tmp += (i->position[k] - gbest[k])*(i->position[k] - gbest[k]);
			}
			g_dis += sqrt(tmp);
		}
		g_dis = g_dis / (double)(swarm.size());
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
bool APSO::checkValidity(const double* pos)
{
	for (int i = 0; i < sut->dimension; i++)
	{
		bool inRange = (pos[i] >= sut->rangeLow[i]) && (pos[i] <= sut->rangeUp[i]);
		if (inRange == false)
			return false;
	}
	return true;
}

int APSO::fuzzyDecision(double f, int previous)
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
		if (f < (double)7 / (double)30)
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
		if (f < (double)23 / (double)30)
			return (previous == 4 || previous == 3) ? 4 : 1;
		else
			return  (previous == 1) ? 1 : 4;
	}

	if (f >= 0.8)
		return 4;

	return -1;
}

/*生成高斯随机数，服从N(m,sigma*sigma)，均值为m，标准差为sigma*/
double APSO::gaussianRand(double m, double sigma)
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

void APSO::swarmInit(vector<CParticle*>& particleSwarm)
{
	if (21 != sut->dimension)
	{
		cout << "用于粒子群初始化的标准差数组维度有误" << endl;
		exit(EXIT_FAILURE);
	}

	if (true)
	{
		/*
		posit_initializer[]由ground truth高斯扰动来模拟数据手套数据，高斯扰动的标准差在"optimization.cpp"文件中定义
		double absolute_error[21] = { 0,0,0,9,1,1,   0,0,0,   1,1,4,   1,2,7,   1,2,4,   1,1,6 };
		*/
		double sigma_initializer[21] = { 12,12,12,9,1,1,  1,1,1,  1,1,4,  1,2,7,  1,2,4,  1,1,6 };//对posit_initializer[]高斯扰动的标准差

																								  /*
																								  数据集数据转26个DoF，相邻两帧对应维度之差绝对值的平均值(上边行)、最大值(下边行)：
																								  3.13   3.11   2.02   4.51   4.18   4.93 ;     4.07   3.20   7.61   6.49 ;     5.35   3.72   7.73   7.10 ;     5.03   2.00   6.32   6.51 ;     6.01   1.16   7.12   6.58 ;     5.52   2.44   7.43   6.72 ;
																								  26.74  17.13   9.59  35.53  48.57  47.02 ;    43.18  20.00  73.55  49.90 ;    40.37  22.01  62.62  51.62 ;    54.38  16.19  49.72  41.85 ;    45.61   9.15  52.39  43.28 ;    74.78  24.04  71.18  51.97 ;
																								  */
		double sigma_preframe[21] = { 6, 6, 4, 5,5,5,  7,8,6,  5,7,5,  5,4,5,  5,3,5,  5,4,5 };//对posit_preframe[]高斯扰动的标准差

		double* recommend = new double[sut->dimension];

		particleSwarm[0]->particleInit(posit_initializer);
		for (int i = 0; i < population; i++)
		{
			for (int v = 0; v != 4; v++)
			{
				recommend[v] = gaussianRand(posit_preframe[v], sigma_preframe[v]); /*高斯分布使得x可能不在[rangeLow[i],rangeUp[i]]区间内*/
			}
			for (int v = 4; v != sut->dimension; v++)
			{
				recommend[v] = gaussianRand(posit_initializer[v], sigma_initializer[v]); /*高斯分布使得x可能不在[rangeLow[i],rangeUp[i]]区间内*/
			}
			particleSwarm[i]->particleInit(recommend);
		}

		delete[]recommend;
	}

	if (false)
	{
		const double proportion_initializer = 0.5;
		const double proportion_preframe = 0.5;
		if (21 != sut->dimension)
		{
			cout << "用于粒子群初始化的标准差数组维度有误" << endl;
			exit(EXIT_FAILURE);
		}

		double* recommend = new double[sut->dimension];

		/*
		posit_initializer[]由ground truth高斯扰动来模拟数据手套数据，高斯扰动的标准差在"optimization.cpp"文件中定义
		double absolute_error[21] = { 0,0,0,9,1,1,   0,0,0,   1,1,4,   1,2,7,   1,2,4,   1,1,6 };
		*/
		double sigma_initializer[21] = { 12,12,12,9,1,1,  1,1,1,  1,1,4,  1,2,7,  1,2,4,  1,1,6 };//对posit_initializer[]高斯扰动的标准差

																								  /*
																								  数据集数据转26个DoF，相邻两帧对应维度之差绝对值的平均值(上边行)、最大值(下边行)：
																								  3.13   3.11   2.02   4.51   4.18   4.93 ;     4.07   3.20   7.61   6.49 ;     5.35   3.72   7.73   7.10 ;     5.03   2.00   6.32   6.51 ;     6.01   1.16   7.12   6.58 ;     5.52   2.44   7.43   6.72 ;
																								  26.74  17.13   9.59  35.53  48.57  47.02 ;    43.18  20.00  73.55  49.90 ;    40.37  22.01  62.62  51.62 ;    54.38  16.19  49.72  41.85 ;    45.61   9.15  52.39  43.28 ;    74.78  24.04  71.18  51.97 ;
																								  */
		double sigma_preframe[21] = { 6, 6, 4, 5,5,5,  7,8,6,  5,7,5,  5,4,5,  5,3,5,  5,4,5 };//对posit_preframe[]高斯扰动的标准差

																							   //对初始化器产生的粒子位置进行扰动	
		particleSwarm[0]->particleInit(posit_initializer);
		for (int i = 1; i < int(population*proportion_initializer); i++)
		{
			for (int v = 0; v != sut->dimension; v++)
			{
				recommend[v] = gaussianRand(posit_initializer[v], sigma_initializer[v]); /*高斯分布使得x可能不在[rangeLow[i],rangeUp[i]]区间内*/
			}
			particleSwarm[i]->particleInit(recommend);
		}

		//对前帧的跟踪结果扰动
		if (population*proportion_initializer < population)
		{
			particleSwarm[int(population*proportion_initializer)]->particleInit(posit_preframe);
			for (int i = int(population*proportion_initializer) + 1; i < population; i++)
			{
				for (int v = 0; v != sut->dimension; v++)
					recommend[v] = gaussianRand(posit_preframe[v], sigma_preframe[v]); /*高斯分布使得x可能不在[rangeLow[i],rangeUp[i]]区间内*/
				particleSwarm[i]->particleInit(recommend);
			}
		}

		delete[]recommend;
	}
}
