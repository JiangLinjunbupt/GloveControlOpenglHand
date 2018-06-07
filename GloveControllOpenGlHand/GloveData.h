#pragma once
#include <vcclr.h>  
#include "Model.h"
#using "../debug/GloveTool.dll"
#using "../debug/GloveLibPC.dll"
using namespace GloveLib;
using namespace GloveTool;
using namespace System;

class GloveData
{
public:
	float *HandinfParams;
	GloveData()
	{
		gm = gcnew GloveModel();
		gm->GetSingleton();
		gm->Mychuankou = "COM3";
		gm->Conected();
		gm->SetSocket();
		HandinfParams = new float[19];
	}
	~GloveData() { 
		delete HandinfParams;
		gm->CloseSocket = true;
		gm->CloseGloveModel();
	}
	void GetGloveData()
	{
		gm->GetData();
		gm->SkeletonJsonToHandinf(gm->fram);
		for (int i = 0;i < 5;i++)
		{
			HandinfParams[i*3 + 0] = gm->handinformation->fingers[i]->Mcp_x;
			HandinfParams[i*3 + 1] = gm->handinformation->fingers[i]->Mcp_z;
			HandinfParams[i*3 + 2] = gm->handinformation->fingers[i]->Pip;
		}
		HandinfParams[15] = gm->handinformation->global_roll_x;
		HandinfParams[16] = gm->handinformation->global_pitch_y;
		HandinfParams[17] = gm->handinformation->global_yaw_z;
		HandinfParams[18] = gm->handinformation->fingers[4]->Dip;
	}

	void GloveControlHand()
	{
		this->GetGloveData();

		Pose p_globle(this->HandinfParams[15], this->HandinfParams[16], this->HandinfParams[17]);
		model->set_hand_rotation(p_globle);

		//Pose p_thumb_lower(0, this->HandinfParams[12] + 40, this->HandinfParams[13] - 10);
		Pose p_thumb_lower(this->HandinfParams[12] + 40, this->HandinfParams[18], this->HandinfParams[13] - 10);
		Pose p_thumb_middle(0, 0.66*this->HandinfParams[14], 0);
		Pose p_thumb_top(0, this->HandinfParams[14], 0);

		model->set_thumbLower_rotation(p_thumb_lower);
		//model->set_one_rotation(p_thumb_lower, 18);
		model->set_one_rotation(p_thumb_middle, 19);
		model->set_one_rotation(p_thumb_top, 20);


		if (this->HandinfParams[1] < -10)
		{
			this->HandinfParams[1] = this->HandinfParams[1] + 15;
		}
		Pose p_pinkey_lower(0, this->HandinfParams[0], this->HandinfParams[1]);
		Pose p_pinkey_middle(0, this->HandinfParams[2], 0);
		Pose p_pinkey_top(0, 0.66*this->HandinfParams[2], 0);
		model->set_one_rotation(p_pinkey_lower, 10);
		model->set_one_rotation(p_pinkey_middle, 11);
		model->set_one_rotation(p_pinkey_middle, 12);


		if (this->HandinfParams[4] < 0)
		{
			this->HandinfParams[4] = this->HandinfParams[4] + 10;
		}
		Pose p_ring_lower(0, this->HandinfParams[3], this->HandinfParams[4]);
		Pose p_ring_middle(0, this->HandinfParams[5], 0);
		Pose p_ring_top(0, 0.66*this->HandinfParams[5], 0);
		model->set_one_rotation(p_ring_lower, 14);
		model->set_one_rotation(p_ring_middle, 15);
		model->set_one_rotation(p_ring_top, 16);


		Pose p_middle_lower(0, this->HandinfParams[6], this->HandinfParams[7]);
		Pose p_middle_middle(0, this->HandinfParams[8], 0);
		Pose p_middle_top(0, 0.66*this->HandinfParams[8], 0);
		model->set_one_rotation(p_middle_lower, 6);
		model->set_one_rotation(p_middle_middle, 7);
		model->set_one_rotation(p_middle_top, 8);


		if (this->HandinfParams[10] > 0)
		{
			this->HandinfParams[10] = this->HandinfParams[10] - 15;
		}
		Pose p_index_lower(0, this->HandinfParams[9], this->HandinfParams[10]);
		Pose p_index_middle(0, this->HandinfParams[11], 0);
		Pose p_index_top(0, 0.66*this->HandinfParams[11], 0);
		model->set_one_rotation(p_index_lower, 2);
		model->set_one_rotation(p_index_middle, 3);
		model->set_one_rotation(p_index_top, 4);


		model->forward_kinematic();
		model->compute_mesh();
	}
private:
	gcroot<GloveModel^> gm;
};

static GloveData glovedata;