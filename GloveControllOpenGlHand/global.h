#pragma once
#include "threadPool.h"
#include<opencv.hpp>
#include"CParticle.h"
#include "APSO.h"
#include<stdlib.h>  /*stdlib.hÖÐ°üº¬rand()º¯Êý*/

using namespace std;
using namespace cv;
void poseEstimate(const Mat& depthSeg, double* output_dof);