#include"global.h"

float  get_objective_func(float* particlePos, float& fitness)
{
	//计算两幅图对应像素的插值             --------------------------------------------
	model->GloveParamsConTrollHand(particlePos);
	model->forward_kinematic();
	model->compute_mesh();

	projection->set_color_index(model);
	projection->project_3d_to_2d_(model, generated_mat);
	
	float E_golden = 0.0;  
	float threshold = 100.0;      //门限设置位100mm
	for (int i = 0; i < generated_mat.rows; i++)
	{
		for (int j = 0; j < generated_mat.cols; j++)
		{
			float difference = abs(generated_mat.at<ushort>(i, j) - Input_depthMat.at<ushort>(i, j));
			E_golden += difference < threshold ? pow(difference, 2) : pow(threshold, 2);
		}
	}
	E_golden = sqrt(E_golden);

	fitness = E_golden;
	return E_golden;
}


void reset_upper_lower_Bound(float *original_upper, float *original_lower,
	const float *init_params,
	float *output_upper, float *output_lower)
{
	//pinkey
	output_upper[0] = (init_params[0] + 20)<original_upper[0]? (init_params[0] + 20): original_upper[0]; output_lower[0] = (init_params[0] -20)>original_lower[0]? (init_params[0] - 20): original_lower[0];    //low  //弯曲
	output_upper[1] = (init_params[1] + 20)<original_upper[1]? (init_params[1] + 20): original_upper[1]; output_lower[1] = (init_params[1] -20)>original_lower[1]? (init_params[1] - 20): original_lower[1];           //左右
	output_upper[2] = (init_params[2] + 20)<original_upper[2] ? (init_params[2] + 20) : original_upper[2]; output_lower[2] = (init_params[2] - 20)>original_lower[2] ? (init_params[2] - 20) : original_lower[2];     //middle
	output_upper[20] = (init_params[20] + 20)<original_upper[20] ? (init_params[20] + 20) : original_upper[20]; output_lower[20] = (init_params[20] - 20)>original_lower[20] ? (init_params[20] - 20) : original_lower[20];   //top

	 //ring
	output_upper[3] = (init_params[3] + 20) < original_upper[3] ? (init_params[3] + 20) : original_upper[3]; output_lower[3] = (init_params[3] - 20) > original_lower[3] ? (init_params[3] - 20) : original_lower[3];   //low     //弯曲
	output_upper[4] = (init_params[4] + 20) < original_upper[4] ? (init_params[4] + 20) : original_upper[4]; output_lower[4] = (init_params[4] - 20) > original_lower[4] ? (init_params[4] - 20) : original_lower[4];             //左右
	output_upper[5] = (init_params[5] + 20) < original_upper[5] ? (init_params[5] + 20) : original_upper[5]; output_lower[5] = (init_params[5] - 20) > original_lower[5] ? (init_params[5] - 20) : original_lower[5];     //middle
	output_upper[21] = (init_params[21] + 20) < original_upper[21] ? (init_params[21] + 20) : original_upper[21]; output_lower[21] = (init_params[21] - 20) > original_lower[21] ? (init_params[21] - 20) : original_lower[21];   //top

	 //middle
	output_upper[6] = (init_params[6] + 20) < original_upper[6] ? (init_params[6] + 20) : original_upper[6]; output_lower[6] = (init_params[6] - 20) > original_lower[6] ? (init_params[6] - 20) : original_lower[6];    //low      //弯曲
	output_upper[7] = 0.0; output_lower[7] = 0.0;               //左右
	output_upper[8] = (init_params[8] + 20) < original_upper[8] ? (init_params[8] + 20) : original_upper[8]; output_lower[8] = (init_params[8] - 20) > original_lower[8] ? (init_params[8] - 20) : original_lower[8];     //middle
	output_upper[22] = (init_params[22] + 20) < original_upper[22] ? (init_params[22] + 20) : original_upper[22]; output_lower[22] = (init_params[22] - 20) > original_lower[22] ? (init_params[22] - 20) : original_lower[22];    //top

	//index
	output_upper[9] = (init_params[9] + 20) < original_upper[9] ? (init_params[9] + 20) : original_upper[9];  output_lower[9] = (init_params[9] - 20) > original_lower[9] ? (init_params[9] - 20) : original_lower[9];     //low      //弯曲
	output_upper[10] = (init_params[10] + 20) < original_upper[10] ? (init_params[10] + 20) : original_upper[10]; output_lower[10] = (init_params[10] - 20) > original_lower[10] ? (init_params[10] - 20) : original_lower[10];              //左右
	output_upper[11] = (init_params[11] + 20) < original_upper[11] ? (init_params[11] + 20) : original_upper[11]; output_lower[11] = (init_params[11] - 20) > original_lower[11] ? (init_params[11] - 20) : original_lower[11];    //middle
	output_upper[23] = (init_params[23] + 20) < original_upper[23] ? (init_params[23] + 20) : original_upper[23]; output_lower[23] = (init_params[23] - 20) > original_lower[23] ? (init_params[23] - 20) : original_lower[23];    //top

	//thumb
	output_upper[12] = (init_params[12] + 20) < original_upper[12] ? (init_params[12] + 20) : original_upper[12]; output_lower[12] = (init_params[12] - 20) > original_lower[12] ? (init_params[12] - 20) : original_lower[12];            //low x
	output_upper[13] = (init_params[13] + 20) < original_upper[13] ? (init_params[13] + 20) : original_upper[13]; output_lower[13] = (init_params[13] - 20) > original_lower[13] ? (init_params[13] - 20) : original_lower[13];            //low y
	output_upper[18] = (init_params[18] + 20) < original_upper[18] ? (init_params[18] + 20) : original_upper[18]; output_lower[18] = (init_params[18] - 20) > original_lower[18] ? (init_params[18] - 20) : original_lower[18];            //low z

	output_upper[14] = (init_params[14] + 20) < original_upper[14] ? (init_params[14] + 20) : original_upper[14]; output_lower[14] = (init_params[14] - 20) > original_lower[14] ? (init_params[14] - 20) : original_lower[14];           //top
	output_upper[19] = (init_params[19] + 20) < original_upper[19] ? (init_params[19] + 20) : original_upper[19]; output_lower[19] = (init_params[19] - 20) > original_lower[19] ? (init_params[19] - 20) : original_lower[19];          //middle



	//global rotation
	output_upper[15] = (init_params[15] + 40) < original_upper[15] ? (init_params[15] + 40) : original_upper[15]; output_lower[15] = (init_params[15] - 40) > original_lower[15] ? (init_params[15] - 40) : original_lower[15];
	output_upper[16] = (init_params[16] + 40) < original_upper[16] ? (init_params[16] + 40) : original_upper[16]; output_lower[16] = (init_params[16] - 40) > original_lower[16] ? (init_params[16] - 40) : original_lower[16];
	output_upper[17] = (init_params[17] + 40) < original_upper[17] ? (init_params[17] + 40) : original_upper[17]; output_lower[17] = (init_params[17] - 40) > original_lower[17] ? (init_params[17] - 40) : original_lower[17];

	//global position
	output_upper[24] = (init_params[24] + 50) < original_upper[24] ? (init_params[24] + 50) : original_upper[24]; output_lower[24] = (init_params[24] - 50) > original_lower[24] ? (init_params[24] - 50) : original_lower[24];
	output_upper[25] = (init_params[25] + 50) < original_upper[25] ? (init_params[25] + 50) : original_upper[25]; output_lower[25] = (init_params[25] - 50) > original_lower[25] ? (init_params[25] - 50) : original_lower[25];
	output_upper[26] = (init_params[26] + 50) < original_upper[26] ? (init_params[26] + 50) : original_upper[26]; output_lower[26] = (init_params[26] - 50) > original_lower[26] ? (init_params[26] - 50) : original_lower[26];
}
void poseEstimate(const Mat& depthSeg, const float *initParams, float *upper, float *lower, float* output_dof)
{

	depthSeg.copyTo(Input_depthMat);
	//然后初始化SUT和APSO

	//这里的upper和lower需要结合initialparams重新规划。------------------------------------
	float *upper_bound = new float[ParticleDim];
	float *lower_bound = new float[ParticleDim];
	reset_upper_lower_Bound(upper, lower, initParams, upper_bound, lower_bound);

	SUT sut(ParticleDim, upper_bound, lower_bound, &get_objective_func);    /*目标函数的粒子维度，各维度取值下界、上界*/
	APSO pso(&sut, 80, 50, initParams);/*sut、粒子总数、迭代数、初始化器产生的初始粒子位置、前一帧跟踪结果*/

	//然后调用pso中的优化方法

	pso.getBestPosByEvolve();

	memcpy(output_dof, pso.bestPosition, sizeof(float)*ParticleDim);
}

