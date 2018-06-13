#include"APSO.h"   //ʹ��APSO::


void  APSO::getBestPosByEvolve()
{
	double inertia = 0.9;    /*����Ȩ��*/
	double factor1 = 2;      /*��֪����*/
	double factor2 = 2;      /*�������*/
	double factor_max = 2.5;
	double factor_min = 1.5;
	double factorSum_max = 4;

	double fit_best = -DBL_MAX;   /*DBL_MIN����double����С��ֵ��-DBL_MAX����*/

								  //��������Ⱥ����������Ⱥ�е����ӳ�ʼ��
	vector<CParticle*> swarm(population);
	for (int i = 0; i != population; i++)
	{
		swarm[i] = new CParticle(sut->dimension, sut->rangeLow, sut->rangeUp);   //ά�ȡ������ռ�߽�
	}
	swarmInit(swarm);

	if (false)
	{
		cout << "���ӳ�ʼ�����(����ʾ10�����ӵ�position)��" << endl;
		for (int n = 0; n != 10; n++)
		{
			cout << "����" << n << ":" << endl;
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

	int it = 0;  /*��������*/
	double f_value = 0;   /*״̬����*/
	int state = 1;        /*���ӵ��ĸ��׶Σ���ʼʱ״̬����Ϊ0���������׶�*/
	const double sigma_max = 1.0;  /*����gbestʱ�õ��Ĳ���sigma_max��sigma_min*/
	const double sigma_min = 0.1;

	double y1_penalty, y2_penalty, y3_penalty;  //Ŀ�꺯���ĵ�һ����������ȡֵ
	unique_ptr<double[]> y1_pen = make_unique<double[]>(population);//�ȼ���new double[population]
	unique_ptr<double[]> y2_pen = make_unique<double[]>(population);
	unique_ptr<double[]> y3_pen = make_unique<double[]>(population);
	while (true)
	{
		for (int i = 0; i != population; ++i)
		{
			function<void()> task = bind(&SUT::cal_single_fitness_5_param, sut, swarm[i]->position, ref(swarm[i]->fitness), ref(y1_pen[i]), ref(y2_pen[i]), ref(y3_pen[i])); //function����ķǾ�̬��Ա����
			threadPool.run(task);
		}
		threadPool.ensureTaskCompleted(population);


		//����ÿ�����ӵ�pbest��fitness_pbest��fitness������ȫ�ּ�ֵbestPosition[]���顢ȫ�ּ�ֵfit_best
		for (int i = 0; i != population; i++)
		{
			double fit = swarm[i]->fitness;
			if (fit >= sut->expected_fitness)
			{
				cout << "�ﵽ�����Ӧ�ȣ�ֹͣ����" << endl;
				memcpy(bestPosition, swarm[i]->position, sut->dimension * sizeof(double));
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
				memcpy(bestPosition, swarm[i]->position, sut->dimension * sizeof(double));
				y1_penalty = y1_pen[i];
				y2_penalty = y2_pen[i];
				y3_penalty = y3_pen[i];
			}
		}


		/*Elitist Learning Strategy*/
		f_value = fCalculate(swarm, bestPosition, fit_best, 1);  //mode=1:ȫ��λ��L2����+�Ƕ�L1����; mode=2��L2����; mode=3����Ӧֵ֮��ľ���ֵ
		state = fuzzyDecision(f_value, state);

		if (state == 4) { inJumpout_times++; }
		/*����������׶�stage=3��ִ�ж�������Ա�����������*/
		if (state == 3)
		{
			inConvergenc_times++;

			if (false)
			{
				double *gbest_tmp = new double[sut->dimension];
				memcpy(gbest_tmp, bestPosition, sut->dimension * sizeof(double));

				/*ԭʼAPSO֮����gbest*/
				int dim = (int)(((double)(rand() % 1000) / 1000.0) * sut->dimension);  /*��ѡһ��ά��*/
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

				//���ֲ��������������ɶȷֳ�ȫ��λ�ơ�ȫ�ֽǶȡ��ؽڽǶ�3�࣬�ֱ����ѡȡ3�����ɶ��е�ĳ��ά�Ƚ��и���
				int dim[3];
				dim[0] = (int)(((double)(rand() % 1000) / 1000.0) * 3);        /*ȫ��λ��[0][1][2]����ѡһ��ά��*/
				dim[1] = 3 + (int)(((double)(rand() % 1000) / 1000.0) * 3);      /*ȫ�ֽǶ�[3][4][5]����ѡһ��ά��*/
				dim[2] = 6 + (int)(((double)(rand() % 1000) / 1000.0) * 15);    /*�ؽڽǶ�[6][7]����[20]����ѡһ��ά��*/
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
			if (true)/*��ȫ���������ӽ���n���ݶ�����*/
			{
				const double delta = 1e-5;
				const int gradient_ascent_times = 5;  //�ݶ���������
				int dim = rand() % sut->dimension;
				double step = sut->rangeUp[dim] - sut->rangeLow[dim];
				step = step / 10 * (1 - (double)it / iteration / 2.0);  //�������ŵ����������½�

				for (unsigned int i = 0; i != gradient_ascent_times; i++)
				{
					double oldValue = bestPosition[dim];
					bestPosition[dim] += delta;  //x+delta
					double fxadd;
					sut->cal_single_fitness_2_param(bestPosition, fxadd);
					bestPosition[dim] = oldValue;   //����λ�ûָ�ԭ����ֵ
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

					//������λ�ø��º����Ӧ��,����֮ǰ��Ӧ�ȱȽ�
					double fitness_gradascent, y1_pen_gradascent, y2_pen_gradascent, y3_pen_gradascent;
					sut->cal_single_fitness_5_param(bestPosition, fitness_gradascent, y1_pen_gradascent, y2_pen_gradascent, y3_pen_gradascent);
					if (fitness_gradascent > fit_best)  //�ݶ����������ӵ���Ӧ��
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
		}  //���˵�ǰ������Ⱥ��ȫ�ּ�ֵbestPosition[]���顢ȫ�ּ�ֵfit_bestҲ�������

		   //printf("��%d�����������ӵ�penalty��y1��y2��y3��% 10.4f  % 10.4f % 10.4f\n", it, y1_penalty, y2_penalty, y3_penalty);
		if (false)
		{
			cout << "��Ӧ��ȫ�ּ�ֵ��Ϊ��" << endl;
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

		double error = cal_average_joints_tracking_error_from_partiposit(4, bestPosition);//��ǰ���������Ӷ�Ӧ��5�ؽ�ƽ�����
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
			printf("��%d������Ⱥ��������Ӧֵfit_best��y1��y2��y3�ֱ��ǣ�% 8.4lf,  % 8.4lf,  % 8.4lf,  % 8.4lf\n", it, fit_best, y1_penalty, y2_penalty, y3_penalty);
			break;
		}

		/*���£���ʼ��һ�ε���*/
		inertia = 1.0 / (1.0 + 1.5 * exp(-2.6 * f_value));

		if (state == 1) /*̽��*/
		{
			factor1 = factor1 + (0.05 + (double)(rand() % 51) / 1000.0);  /*[0.05,0.1]*/
			factor2 = factor2 - (0.05 + (double)(rand() % 51) / 1000.0);
		}
		else if (state == 2) /*����*/
		{
			factor1 = factor1 + 0.5 * (0.05 + (double)(rand() % 51) / 1000.0);    /*[0.025,0.05]*/
			factor2 = factor2 - 0.5 * (0.05 + (double)(rand() % 51) / 1000.0);
		}
		else if (state == 3) /*����*/
		{
			factor1 = factor1 + 0.5 * (0.05 + (double)(rand() % 51) / 1000.0);
			factor2 = factor2 + 0.5 * (0.05 + (double)(rand() % 51) / 1000.0);
		}
		else if (state == 4)  /*����*/
		{
			factor1 = factor1 - (0.05 + (double)(rand() % 51) / 1000.0);
			factor2 = factor2 + (0.05 + (double)(rand() % 51) / 1000.0);
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


double APSO::fCalculate(const vector<CParticle*>& swarm, const double* gbest, double gbestfit, int mode)
{
	/*
	mode=1:ȫ��λ��L2����+�Ƕ�L1����
	mode=2��L2����
	mode=3����Ӧֵ֮��ľ���ֵ
	*/
	double min_dis = DBL_MAX;  /*ƽ���������Сֵ*/
	double max_dis = -DBL_MAX;  /*ƽ����������ֵ*/
	double g_dis = 0;          /*ȫ���������ӵ������������ӵ�ƽ������*/

	int row = (int)swarm.size();
	int col = (int)swarm.size();
	double ** a = new double *[row];
	for (int i = 0; i < row; i++)
		a[i] = new double[col];

	if (mode == 1)
	{
		for (int i = 0; i < row; i++)   /*����a[][]�����������Ԫ��*/
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
		for (int i = 0; i < row; i++)   /*����a[][]�����������Ԫ��*/
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
		double meanDist = 0;
		for (int j = 0; j < i; j++)
			meanDist += a[j][i];
		for (int j = i + 1; j < col; j++)
			meanDist += a[i][j];
		meanDist = meanDist / (double)(col - 1);

		if (meanDist < min_dis) { min_dis = meanDist; }
		if (meanDist > max_dis) { max_dis = meanDist; }
	}

	/*����ȫ���������ӵ��������ӵ�ƽ������*/
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
	�����׶� S1 = exploration
	���ý׶� S2 = exploitation
	�����׶� S3 = convergence
	�����׶� S4 = jumping-out
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

/*���ɸ�˹�����������N(m,sigma*sigma)����ֵΪm����׼��Ϊsigma*/
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
	return X * sigma + m;  //X����N(0,1)��  ת��ΪN(m,sigma*sigma)
}

void APSO::swarmInit(vector<CParticle*>& particleSwarm)
{
	if (21 != sut->dimension)
	{
		cout << "��������Ⱥ��ʼ���ı�׼������ά������" << endl;
		exit(EXIT_FAILURE);
	}

	if (true)
	{
		/*
		posit_initializer[]��ground truth��˹�Ŷ���ģ�������������ݣ���˹�Ŷ��ı�׼����"optimization.cpp"�ļ��ж���
		double absolute_error[21] = { 0,0,0,9,1,1,   0,0,0,   1,1,4,   1,2,7,   1,2,4,   1,1,6 };
		*/
		double sigma_initializer[21] = { 12,12,12,9,1,1,  1,1,1,  1,1,4,  1,2,7,  1,2,4,  1,1,6 };//��posit_initializer[]��˹�Ŷ��ı�׼��

																								  /*
																								  ���ݼ�����ת26��DoF��������֡��Ӧά��֮�����ֵ��ƽ��ֵ(�ϱ���)�����ֵ(�±���)��
																								  3.13   3.11   2.02   4.51   4.18   4.93 ;     4.07   3.20   7.61   6.49 ;     5.35   3.72   7.73   7.10 ;     5.03   2.00   6.32   6.51 ;     6.01   1.16   7.12   6.58 ;     5.52   2.44   7.43   6.72 ;
																								  26.74  17.13   9.59  35.53  48.57  47.02 ;    43.18  20.00  73.55  49.90 ;    40.37  22.01  62.62  51.62 ;    54.38  16.19  49.72  41.85 ;    45.61   9.15  52.39  43.28 ;    74.78  24.04  71.18  51.97 ;
																								  */
		double sigma_preframe[21] = { 6, 6, 4, 5,5,5,  7,8,6,  5,7,5,  5,4,5,  5,3,5,  5,4,5 };//��posit_preframe[]��˹�Ŷ��ı�׼��

		double* recommend = new double[sut->dimension];

		particleSwarm[0]->particleInit(posit_initializer);
		for (int i = 0; i < population; i++)
		{
			for (int v = 0; v != 4; v++)
			{
				recommend[v] = gaussianRand(posit_preframe[v], sigma_preframe[v]); /*��˹�ֲ�ʹ��x���ܲ���[rangeLow[i],rangeUp[i]]������*/
			}
			for (int v = 4; v != sut->dimension; v++)
			{
				recommend[v] = gaussianRand(posit_initializer[v], sigma_initializer[v]); /*��˹�ֲ�ʹ��x���ܲ���[rangeLow[i],rangeUp[i]]������*/
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
			cout << "��������Ⱥ��ʼ���ı�׼������ά������" << endl;
			exit(EXIT_FAILURE);
		}

		double* recommend = new double[sut->dimension];

		/*
		posit_initializer[]��ground truth��˹�Ŷ���ģ�������������ݣ���˹�Ŷ��ı�׼����"optimization.cpp"�ļ��ж���
		double absolute_error[21] = { 0,0,0,9,1,1,   0,0,0,   1,1,4,   1,2,7,   1,2,4,   1,1,6 };
		*/
		double sigma_initializer[21] = { 12,12,12,9,1,1,  1,1,1,  1,1,4,  1,2,7,  1,2,4,  1,1,6 };//��posit_initializer[]��˹�Ŷ��ı�׼��

																								  /*
																								  ���ݼ�����ת26��DoF��������֡��Ӧά��֮�����ֵ��ƽ��ֵ(�ϱ���)�����ֵ(�±���)��
																								  3.13   3.11   2.02   4.51   4.18   4.93 ;     4.07   3.20   7.61   6.49 ;     5.35   3.72   7.73   7.10 ;     5.03   2.00   6.32   6.51 ;     6.01   1.16   7.12   6.58 ;     5.52   2.44   7.43   6.72 ;
																								  26.74  17.13   9.59  35.53  48.57  47.02 ;    43.18  20.00  73.55  49.90 ;    40.37  22.01  62.62  51.62 ;    54.38  16.19  49.72  41.85 ;    45.61   9.15  52.39  43.28 ;    74.78  24.04  71.18  51.97 ;
																								  */
		double sigma_preframe[21] = { 6, 6, 4, 5,5,5,  7,8,6,  5,7,5,  5,4,5,  5,3,5,  5,4,5 };//��posit_preframe[]��˹�Ŷ��ı�׼��

																							   //�Գ�ʼ��������������λ�ý����Ŷ�	
		particleSwarm[0]->particleInit(posit_initializer);
		for (int i = 1; i < int(population*proportion_initializer); i++)
		{
			for (int v = 0; v != sut->dimension; v++)
			{
				recommend[v] = gaussianRand(posit_initializer[v], sigma_initializer[v]); /*��˹�ֲ�ʹ��x���ܲ���[rangeLow[i],rangeUp[i]]������*/
			}
			particleSwarm[i]->particleInit(recommend);
		}

		//��ǰ֡�ĸ��ٽ���Ŷ�
		if (population*proportion_initializer < population)
		{
			particleSwarm[int(population*proportion_initializer)]->particleInit(posit_preframe);
			for (int i = int(population*proportion_initializer) + 1; i < population; i++)
			{
				for (int v = 0; v != sut->dimension; v++)
					recommend[v] = gaussianRand(posit_preframe[v], sigma_preframe[v]); /*��˹�ֲ�ʹ��x���ܲ���[rangeLow[i],rangeUp[i]]������*/
				particleSwarm[i]->particleInit(recommend);
			}
		}

		delete[]recommend;
	}
}
