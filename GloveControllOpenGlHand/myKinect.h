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
	//kinect 2.0 的深度空间的高*宽是 424 * 512，在官网上有说明
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

	HRESULT  InitializeDefaultSensor();//用于初始化kinect
	void Collectdata();

	cv::Mat HandsegmentMat;
};


static myKinect mykinect;