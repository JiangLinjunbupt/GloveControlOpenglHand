#include "myKinect.h"
#include <math.h>
#include <iostream>
#include <stdio.h>

myKinect::myKinect():
	mySensor(NULL),
	mydepthReader(NULL),
	mycolorReader(NULL),
	myBodyReader(NULL),
	myBodySource(NULL),
	myMapper(NULL) {
	this->HandsegmentMat = Mat::zeros(424,512,CV_16UC1);
}


myKinect::~myKinect()
{
	SafeRelease(myBodySource);
	SafeRelease(myBodyReader);
	SafeRelease(myMapper);
	SafeRelease(mycolorReader);
	SafeRelease(mydepthReader);

	if (mySensor)
	{
		mySensor->Close();
	}

	SafeRelease(mySensor);
}

HRESULT myKinect::InitializeDefaultSensor()
{
	HRESULT hr;
	//����kinect
	hr = GetDefaultKinectSensor(&mySensor);
	if (FAILED(hr)) {
		return hr;
	}
	if (mySensor)
	{
		// Initialize the Kinect and get coordinate mapper and the body reader
		IColorFrameSource   * mycolorSource = nullptr;
		IDepthFrameSource   * mydepthSource = nullptr;   //ȡ���������

		hr = mySensor->Open();    //��kinect 

		//coordinatemapper
		if (SUCCEEDED(hr))
		{
			hr = mySensor->get_CoordinateMapper(&myMapper);
		}
		//color
		if (SUCCEEDED(hr))
		{
			hr = mySensor->get_ColorFrameSource(&mycolorSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = mycolorSource->OpenReader(&mycolorReader);
		}

		//depth
		if (SUCCEEDED(hr)) {
			hr = mySensor->get_DepthFrameSource(&mydepthSource);
		}

		if (SUCCEEDED(hr)) {
			hr = mydepthSource->OpenReader(&mydepthReader);
		}

		//body
		if (SUCCEEDED(hr)) {
			hr = mySensor->get_BodyFrameSource(&myBodySource);
		}

		if (SUCCEEDED(hr)) {
			hr = myBodySource->OpenReader(&myBodyReader);
		}

		SafeRelease(mycolorSource);
		SafeRelease(mydepthSource);
	}

	if (!mySensor || FAILED(hr))
	{
		std::cerr << "Kinect initialization failed!" << std::endl;
		return E_FAIL;
	}

	return hr;

}

void myKinect::Collectdata()
{
	Mat original_depth_16U(cDepthHeight, cDepthWidth, CV_16UC1);    //����ͼ�����
	UINT16 *depthData = new UINT16[424 * 512];

	Mat copy_original_depth_16U(cDepthHeight, cDepthWidth, CV_16UC1);    //����ͼ�����
	Mat m_middepth8u(424, 512, CV_8UC1);
	
	ColorSpacePoint *m_pcolorcoordinate = new ColorSpacePoint[512 * 424];
	CameraSpacePoint *m_pcameracoordinate = new CameraSpacePoint[512 * 424];

	Mat image_color(1080, 1920, CV_8UC4);
	Mat hsv;
	Mat bw(1080, 1920, CV_8UC1);
	Mat bw2(1080, 1920, CV_8UC1);
	Mat bw3(1080, 1920, CV_8UC1);

	//�����ʧ��kinect���򲻼�������
	if (!mydepthReader)
	{
		return;
	}


	IBodyFrame		* myBodyFrame = nullptr;
	IDepthFrame     * mydepthFrame = nullptr;
	IColorFrame     * mycolorFrame = nullptr;

	if (mycolorReader->AcquireLatestFrame(&mycolorFrame) == S_OK) {
		mycolorFrame->CopyConvertedFrameDataToArray(1080 * 1920 * 4, (BYTE *)image_color.data, ColorImageFormat_Bgra);
		//imshow("color", image_color);
		cvtColor(image_color, hsv, CV_BGR2HSV);

		//inRange(hsv, Scalar(0, 10, 60), Scalar(20, 150, 255), bw);  //��ɫ
		//inRange(hsv, Scalar(0, 0, 0), Scalar(180, 255, 46), bw2); //��ɫ
		//inRange(hsv, Scalar(155, 43, 35), Scalar(180, 255, 255), bw2);
		//inRange(hsv, Scalar(0, 43, 35), Scalar(11, 255, 255), bw3);
		//bw = bw3 + bw2;        //��ɫ

		inRange(hsv, Scalar(0, 0, 0), Scalar(255, 255, 255), bw);  //������ɫ
	}
	if (mydepthReader->AcquireLatestFrame(&mydepthFrame) == S_OK) {
		mydepthFrame->CopyFrameDataToArray(cDepthHeight * cDepthWidth, (UINT16 *)original_depth_16U.data); //�Ȱ����ݴ���16λ��ͼ�������																		
		mydepthFrame->CopyFrameDataToArray(cDepthHeight * cDepthWidth, depthData);
		original_depth_16U.copyTo(copy_original_depth_16U);
		// ���ȴ������Ϊ 0 �ĵ㣬����ʵ�����޷������ĵ㣬
		//���Խ����Ϊ 0 �ĵ����������
		for (int i = 0; i < 424; i++)
		{
			for (int j = 0; j < 512; j++)
			{
				unsigned short & temp = copy_original_depth_16U.at<unsigned short>(i, j);
				if (temp == 0)
				{
					temp = 65535;//16λ���ֵΪ65535
				}
			}
		}

		////��˹ƽ���˲�
		GaussianBlur(copy_original_depth_16U, copy_original_depth_16U, Size(9, 9), 0.85, 0.85);

		double minValue, maxValue;
		minMaxIdx(copy_original_depth_16U, &minValue, &maxValue);

		for (int i = 0; i < 424; i++)
		{
			for (int j = 0; j < 512; j++)
			{
				if (original_depth_16U.at<unsigned short>(i, j) > minValue + 120)
					m_middepth8u.at<unsigned char>(i, j) = 0;
				else
				{
					m_middepth8u.at<unsigned char>(i, j) = original_depth_16U.at<unsigned short>(i, j) % 255;
				}
			}
		}
		//��ȡ���ͼ����ɫ�ռ䣬������ռ��ӳ�䣻
		while (myMapper->MapDepthFrameToColorSpace(424 * 512, depthData, 424 * 512, m_pcolorcoordinate) != S_OK) { ; }
		while (myMapper->MapDepthFrameToCameraSpace(424 * 512, depthData, 424 * 512, m_pcameracoordinate) != S_OK) { ; }
		while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != S_OK) { ; }			//��ȡBody����


		for (int i = 0; i < 424; i++)
		{
			for (int j = 0; j < 512; j++)
			{
				int index_depth = i * 512 + j;
				ColorSpacePoint pp = m_pcolorcoordinate[index_depth];
				if (pp.X != -std::numeric_limits<float>::infinity() && pp.Y != -std::numeric_limits<float>::infinity())
				{
					int colorX = static_cast<int>(pp.X + 0.5f);
					int colorY = static_cast<int>(pp.Y + 0.5f);
					if ((colorX >= 0 && colorX < 1920) && (colorY >= 0 && colorY < 1080))
					{
						if (bw.at<unsigned char>(colorY, colorX) == 0)
						{
							m_middepth8u.at<unsigned char>(i, j) = 0;
							//save_original_depth_16U.at<ushort>(i, j) = 65535;
						}
						else
						{
							//m_middepth8u.at<unsigned char>(i, j) = 0;
						}
					}
				}
			}
		}


		//int	bodyBufferSize = 0;
		//myBodySource->get_BodyCount(&bodyBufferSize);
		//IBody	** bodyArray = new IBody *[bodyBufferSize];
		//for (int i = 0; i < bodyBufferSize; i++)
		//	bodyArray[i] = nullptr;
		//myBodyFrame->GetAndRefreshBodyData(bodyBufferSize, bodyArray);

		//int nearestBodyindex = -1;
		//float MinHead_Z = 100;
		//CameraSpacePoint nearestRightHandPosition;
		//for (int i = 0; i < bodyBufferSize; i++)					//����6����
		//{
		//	BOOLEAN		result = false;
		//	if (bodyArray[i]->get_IsTracked(&result) == S_OK && result)
		//	{
		//		_Joint	jointArray[JointType_Count];				//���ؽڵ��������ʽ��ʼ����
		//		bodyArray[i]->GetJoints(JointType_Count, jointArray);

		//		if (jointArray[JointType_Head].TrackingState == TrackingState_Tracked&&jointArray[JointType_HandRight].TrackingState == TrackingState_Tracked)
		//		{
		//			if (jointArray[JointType_Head].Position.Z < MinHead_Z)
		//			{
		//				MinHead_Z = jointArray[JointType_Head].Position.Z;
		//				nearestBodyindex = i;
		//				nearestRightHandPosition = jointArray[JointType_HandRight].Position;

		//			}
		//		}
		//	}


		//}


		//for (int i = 0; i < 424; i++)
		//{
		//	for (int j = 0; j < 512; j++)
		//	{
		//		if (m_middepth8u.at<uchar>(i, j) != 0)
		//		{
		//			int index_depth = i * 512 + j;
		//			CameraSpacePoint p = m_pcameracoordinate[index_depth];
		//			float distance = sqrt(pow((p.X - nearestRightHandPosition.X), 2) + pow(p.Y - nearestRightHandPosition.Y, 2) + pow(p.Z - nearestRightHandPosition.Z, 2));

		//			if (distance <0.12)
		//			{
		//				;
		//			}
		//			else
		//			{
		//				m_middepth8u.at<uchar>(i, j) = 0;
		//			}

		//		}

		//	}
		//}

		//����ɫ�ָ������Ӧ����һ��������⣬��ȡ���������

		//��ֵ��HandsegmentMat ��Ϊ���������ݣ�
		this->HandsegmentMat.setTo(0);
		for (int i = 0; i < 424; i++)
		{
			for (int j = 0; j < 512; j++)
			{
				if (m_middepth8u.at<uchar>(i, j) != 0)
				{
					HandsegmentMat.at<ushort>(i, j) = original_depth_16U.at<ushort>(i, j);
				}
			}
		}

		medianBlur(HandsegmentMat, HandsegmentMat, 5);

		imshow("m_middepth8u", m_middepth8u);
		//delete[] bodyArray;
	}
	SafeRelease(myBodyFrame);
	SafeRelease(mydepthFrame);
	SafeRelease(mycolorFrame);
}
