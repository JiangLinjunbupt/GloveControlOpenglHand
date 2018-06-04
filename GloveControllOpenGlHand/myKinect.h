#pragma once
#include <Kinect.h>
#include<string>
#include <opencv2\opencv.hpp>
using namespace cv;
using namespace std;
// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}


class myKinect
{
	//kinect 2.0 ����ȿռ�ĸ�*���� 424 * 512���ڹ�������˵��
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;
private:
	IKinectSensor       * mySensor;

	IColorFrameReader   * mycolorReader;
	IDepthFrameReader   * mydepthReader;
	IBodyFrameReader	* myBodyReader;
	ICoordinateMapper   * myMapper;
	IBodyFrameSource	* myBodySource;

public:
	myKinect();
	~myKinect();

	HRESULT  InitializeDefaultSensor();//���ڳ�ʼ��kinect
	void Collectdata();

	cv::Mat HandsegmentMat;
};


static myKinect mykinect;