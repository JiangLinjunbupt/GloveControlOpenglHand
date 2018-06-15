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
		HandinfParams = new float[27];
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
		HandinfParams[18] = gm->handinformation->fingers[4]->Dip;    //大拇指的Mcp_y


		HandinfParams[19] = 0.66*HandinfParams[14];  //大拇指弯曲
		HandinfParams[20] = 0.66*HandinfParams[2];
		HandinfParams[21] = 0.66*HandinfParams[5];
		HandinfParams[22] = 0.66*HandinfParams[8];
		HandinfParams[23] = 0.66*HandinfParams[11];


		//调整参数范围，使其看起来不那么奇怪。
		if (HandinfParams[1] < -10)
		{
			HandinfParams[1] = HandinfParams[1] + 15;
		}

		if (HandinfParams[4] < 0)
		{
			HandinfParams[4] = HandinfParams[4] + 10;
		}

		if (HandinfParams[10] > 0)
		{
			HandinfParams[10] = HandinfParams[10] - 15;
		}

		HandinfParams[12] = HandinfParams[12] + 40;
		HandinfParams[13] = HandinfParams[13] - 10;

		HandinfParams[7] = 0.0;


		for (int i = 0; i < 24; i++)
		{
			if (i == 15 || i == 16 || i == 17)
			{
				if (HandinfParams[i] < 0)
				{
					HandinfParams[i] = HandinfParams[i] + 360.0;
				}
			}
			else
			{
				if (HandinfParams[i] > 200)
				{
					HandinfParams[i] = HandinfParams[i] - 360;
				}
				else if(HandinfParams[i]<-180)
				{
					HandinfParams[i] = HandinfParams[i] + 360;
				}
				else
				{
					;
				}
			}
		}
	}

private:
	gcroot<GloveModel^> gm;
};

static GloveData glovedata;