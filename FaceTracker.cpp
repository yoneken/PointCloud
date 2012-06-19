#include "FaceTracker.h"
#include <Windows.h>
#include <stdio.h>

#pragma comment(lib, "Kinect10.lib")
#include <ole2.h>
#include <NuiApi.h>

#pragma comment(lib, "FaceTrackLib.lib")
#include <FaceTrackLib.h>

#define STDERR(str) OutputDebugString(L##str)
//#define STDERR(str) 
#define MIN(a,b) ((a)<(b))?(a):(b)

IFTFaceTracker *pFaceTracker;
IFTResult *pFTResult;
IFTImage *iftColorImage;
IFTImage *iftDepthImage;
bool lastTrackSucceeded;
float faceScale;
float faceR[3];
float faceT[3];

extern Vector4 skels[NUI_SKELETON_COUNT][NUI_SKELETON_POSITION_COUNT];

void initFaceTracker(void)
{
	HRESULT hr;
	pFaceTracker = FTCreateFaceTracker(NULL);	// We don't use any options.
	if(!pFaceTracker){
		STDERR("Could not create the face tracker.\r\n");
		return;
	}
	
	FT_CAMERA_CONFIG videoConfig;
	videoConfig.Width = 640;
	videoConfig.Height = 480;
	videoConfig.FocalLength = NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS;			// 640x480
	//videoConfig.FocalLength = NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS * 2.f;	// 1280x960

	FT_CAMERA_CONFIG depthConfig;
	depthConfig.Width = 320;
	depthConfig.Height = 240;
	//depthConfig.FocalLength = NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS / 4.f;	//  80x 60
	depthConfig.FocalLength = NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS;			// 320x240
	//depthConfig.FocalLength = NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS * 2.f;	// 640x480

	hr = pFaceTracker->Initialize(&videoConfig, &depthConfig, NULL, NULL);
	if(!pFaceTracker){
		STDERR("Could not initialize the face tracker.\r\n");
		return;
	}

	hr = pFaceTracker->CreateFTResult(&pFTResult);
	if (FAILED(hr) || !pFTResult)
	{
		STDERR("Could not initialize the face tracker result.\r\n");
		return;
	}

	iftColorImage = FTCreateImage();
	if (!iftColorImage || FAILED(hr = iftColorImage->Allocate(videoConfig.Width, videoConfig.Height, FTIMAGEFORMAT_UINT8_B8G8R8X8)))
	{
		STDERR("Could not create the color image.\r\n");
		return;
	}
	iftDepthImage = FTCreateImage();
	if (!iftDepthImage || FAILED(hr = iftDepthImage->Allocate(320, 240, FTIMAGEFORMAT_UINT16_D13P3)))
	{
		STDERR("Could not create the depth image.\r\n");
		return;
	}

	lastTrackSucceeded = false;
	faceScale = 0;
}

void clearFaceTracker(void)
{
	pFaceTracker->Release();
    pFaceTracker = NULL;

    if(iftColorImage)
    {
        iftColorImage->Release();
        iftColorImage = NULL;
    }

    if(pFTResult)
    {
        pFTResult->Release();
        pFTResult = NULL;
    }
}

void storeFace(int playerID)
{
	HRESULT hrFT = E_FAIL;
	FT_SENSOR_DATA sensorData(iftColorImage, iftDepthImage);
	FT_VECTOR3D hint[2];

	hint[0].x = skels[playerID][NUI_SKELETON_POSITION_HEAD].x;
	hint[0].y = skels[playerID][NUI_SKELETON_POSITION_HEAD].y;
	hint[0].z = skels[playerID][NUI_SKELETON_POSITION_HEAD].z;
	hint[1].x = skels[playerID][NUI_SKELETON_POSITION_SHOULDER_CENTER].x;
	hint[1].y = skels[playerID][NUI_SKELETON_POSITION_SHOULDER_CENTER].y;
	hint[1].z = skels[playerID][NUI_SKELETON_POSITION_SHOULDER_CENTER].z;

	if (lastTrackSucceeded)
	{
		hrFT = pFaceTracker->ContinueTracking(&sensorData, hint, pFTResult);
	}
	else
	{
		hrFT = pFaceTracker->StartTracking(&sensorData, NULL, hint, pFTResult);
	}

	lastTrackSucceeded = SUCCEEDED(hrFT) && SUCCEEDED(pFTResult->GetStatus());
	if (lastTrackSucceeded)
	{
		pFTResult->Get3DPose(&faceScale, faceR, faceT);
		printf("%3.2f, %3.2f, %3.2f, %3.2f\r\n", faceScale, faceR[0], faceR[1], faceR[2]);
	}
	else
	{
		pFTResult->Reset();
	}
}

void setColorImage(unsigned char *data, int size)
{
	memcpy(iftColorImage->GetBuffer(), data, MIN(iftColorImage->GetBufferSize(), size));
}

void setDepthImage(unsigned char *data, int size)
{
	memcpy(iftDepthImage->GetBuffer(), data, MIN(iftDepthImage->GetBufferSize(), size));
}
