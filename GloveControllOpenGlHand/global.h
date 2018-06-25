#pragma once
#include<opencv.hpp>
#include"CParticle.h"
#include"extern.h"
#include "Projection.h"
#include "APSO.h"
#include<stdlib.h>  /*stdlib.hÖÐ°üº¬rand()º¯Êý*/

using namespace std;
using namespace cv;

static const int ParticleDim = 27;
static cv::Mat generated_mat = cv::Mat::zeros(424, 512, CV_16UC1);
static Mat Input_depthMat;
void poseEstimate(const Mat& depthSeg, const float * initParams, float *upper,float *lower,float* output_dof);
void reset_upper_lower_Bound(float *original_upper, float *original_lower, 
	const float *init_params, 
	float *output_upper, float *output_lower);