/*
 *  PointCloud.cpp
 *
 *  Created: 2012/06/19
 *  Author: yoneken
 */


#define STDERR(str) OutputDebugString(L##str)
//#define STDERR(str) 

#if defined(WIN32)
#include <GL/glut.h>    			// Header File For The GLUT Library
#include <GL/gl.h>
#include <GL/glu.h>
#else /* OS : !win */
#include <GLUT/glut.h>
#endif /* OS */

#pragma comment(lib, "Kinect10.lib")
#include <ole2.h>
#include <NuiApi.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "FaceTracker.h"

/* ASCII code for the escape key. */
#define KEY_ESCAPE 27
#define DEFAULT_WIDTH 1280
#define DEFAULT_HEIGHT 960

int window;									// The number of our GLUT window
int time,timeprev=0;						// For calculating elapsed time

INuiSensor *pNuiSensor;
HANDLE hNextColorFrameEvent;
HANDLE hNextDepthFrameEvent;
HANDLE hNextSkeletonEvent;
HANDLE pVideoStreamHandle;
HANDLE pDepthStreamHandle;

typedef enum _TEXTURE_INDEX{
	IMAGE_TEXTURE = 0,
	DEPTH_TEXTURE,
	EDITED_TEXTURE,
	TEXTURE_NUM,
} TEXTURE_INDEX;
GLuint bg_texture[TEXTURE_NUM];

Vector4 skels[NUI_SKELETON_COUNT][NUI_SKELETON_POSITION_COUNT];
int trackedDataIndex = 0;
bool tracked = false;
unsigned short depth[240][320];

const int win_width = 640;
const int win_height = 480;
int tex_width = 320;
int tex_height = 240;
short CloudMap[win_height][win_width][3];
short TexMap[win_height][win_width][3];
unsigned int indices[win_height][win_width];

// mouse
int mx=-1,my=-1; // Prevous mouse coordinates
int rotangles[2] = {0}; // Panning angles
float zoom = 1; // zoom factor
short base[3] = {0};

void glDrawArrowd(double x0, double y0, double z0,
 double x1, double y1, double z1)
{
	GLUquadricObj *arrows[2];
	double x2, y2, z2, len, ang;

	x2 = x1-x0; y2 = y1-y0; z2 = z1-z0;
	len = sqrt(x2*x2 + y2*y2 + z2*z2);
	if(len != 0.0){
		ang = acos(z2*len/(sqrt(x2*x2+y2*y2+z2*z2)*len))/3.1415926*180.0;

		glPushMatrix();
			glTranslated( x0, y0, z0);
			glRotated( ang, -y2*len, x2*len, 0.0);
			arrows[0] = gluNewQuadric();
			gluQuadricDrawStyle(arrows[0], GLU_FILL);
			gluCylinder(arrows[0], len/80, len/80, len*0.9, 8, 8);
			glPushMatrix();
				glTranslated( 0.0, 0.0, len*0.9);
				arrows[1] = gluNewQuadric();
				gluQuadricDrawStyle(arrows[1], GLU_FILL);
				gluCylinder(arrows[1], len/30, 0.0f, len/10, 8, 8);
			glPopMatrix();
		glPopMatrix();
	}
}

void makeCloudMap(void)
{
	GLfloat win_tex_w = (float)win_width/tex_width;
	GLfloat win_tex_h = (float)win_height/tex_height;
	long cx=0,cy=0;

	for( int y = 0; y < 240; ++y ){
		for( int x = 0; x < 320; ++x){
			Vector4 tmp = NuiTransformDepthImageToSkeleton(x, y, depth[y][x]);
			NuiImageGetColorPixelCoordinatesFromDepthPixel(NUI_IMAGE_RESOLUTION_320x240, NULL, x, y, depth[y][x], &cx, &cy);
			CloudMap[y][x][0] = (short)(tmp.z*1000);
			TexMap[y][x][0] = (short)cx*win_tex_w;
			CloudMap[y][x][1] = -(short)(tmp.x*1000);
			TexMap[y][x][1] = (short)cy*win_tex_h;
			CloudMap[y][x][2] = (short)(tmp.y*1000);
			TexMap[y][x][2] = (short)(tmp.z*1000);
			indices[y][x] = y*win_width+x;
		}
	}
}

/*
 * @brief A general Nui initialization function.  Sets all of the initial parameters.
 */
void initNui(void)	        // We call this right after Nui functions called.
{
	HRESULT hr;

	hr = NuiCreateSensorByIndex(0, &pNuiSensor);
	if(FAILED(hr)) STDERR("Cannot connect with kinect0.\r\n");

	hr = pNuiSensor->NuiInitialize(

		//NUI_INITIALIZE_FLAG_USES_DEPTH |
		NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | 
		NUI_INITIALIZE_FLAG_USES_COLOR | 
		NUI_INITIALIZE_FLAG_USES_SKELETON
		);
	if ( E_NUI_SKELETAL_ENGINE_BUSY == hr ){
		hr = pNuiSensor->NuiInitialize(
			NUI_INITIALIZE_FLAG_USES_DEPTH |
			NUI_INITIALIZE_FLAG_USES_COLOR
			);
	}
	if(FAILED(hr)){
		STDERR("Cannot initialize kinect.\r\n");
	}

	hNextColorFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
	hNextDepthFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
	hNextSkeletonEvent = CreateEvent( NULL, TRUE, FALSE, NULL );

	if(HasSkeletalEngine(pNuiSensor)){
		hr = pNuiSensor->NuiSkeletonTrackingEnable( hNextSkeletonEvent, 
			//NUI_SKELETON_TRACKING_FLAG_TITLE_SETS_TRACKED_SKELETONS |
			//NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT 
			0
			);
		if(FAILED(hr)) STDERR("Cannot track skeletons\r\n");
	}

	hr = pNuiSensor->NuiImageStreamOpen(
		NUI_IMAGE_TYPE_COLOR,
		NUI_IMAGE_RESOLUTION_640x480,
		0,
		2,
		hNextColorFrameEvent,
		&pVideoStreamHandle );
	if(FAILED(hr)){
		STDERR("Cannot open image stream\r\n");
	}

	hr = pNuiSensor->NuiImageStreamOpen(
		HasSkeletalEngine(pNuiSensor) ? NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX : NUI_IMAGE_TYPE_DEPTH,
		NUI_IMAGE_RESOLUTION_320x240,
		0,
		2,
		hNextDepthFrameEvent,
		&pDepthStreamHandle );
	if(FAILED(hr)){
		STDERR("Cannot open depth and player stream\r\n");
	}
/*
	hr = pNuiSensor->NuiImageStreamOpen(
		NUI_IMAGE_TYPE_DEPTH,
		NUI_IMAGE_RESOLUTION_640x480,
		0,
		2,
		hNextDepthFrameEvent,
		&pDepthStreamHandle );
	if(FAILED(hr)){
		STDERR("Cannot open depth stream\r\n");
	}
*/
#if defined(USE_FACETRACKER)
	initFaceTracker();
#endif
}

void storeNuiImage(void)
{
	NUI_IMAGE_FRAME imageFrame;

	if(WAIT_OBJECT_0 != WaitForSingleObject(hNextColorFrameEvent, 0)) return;

	HRESULT hr =  pNuiSensor->NuiImageStreamGetNextFrame(
		pVideoStreamHandle,
		0,
		&imageFrame );
	if( FAILED( hr ) ){
		return;
	}
	if(imageFrame.eImageType != NUI_IMAGE_TYPE_COLOR)
		STDERR("Image type is not match with the color\r\n");

	INuiFrameTexture *pTexture = imageFrame.pFrameTexture;
	NUI_LOCKED_RECT LockedRect;
	pTexture->LockRect( 0, &LockedRect, NULL, 0 );
	if( LockedRect.Pitch != 0 ){
		byte * pBuffer = (byte *)LockedRect.pBits;
#if defined(USE_FACETRACKER)
		setColorImage(LockedRect.pBits, LockedRect.size);
#endif
		NUI_SURFACE_DESC pDesc;
		pTexture->GetLevelDesc(0, &pDesc);
		//printf("w: %d, h: %d, byte/pixel: %d\r\n", pDesc.Width, pDesc.Height, LockedRect.Pitch/pDesc.Width);
		typedef struct t_RGBA{
			byte r;
			byte g;
			byte b;
			byte a;
		};
		t_RGBA *p = (t_RGBA *)pBuffer;
		for(int i=0;i<pTexture->BufferLen()/4;i++){
			byte b = p->b;
			p->b = p->r;
			p->r = b;
			p->a = (byte)255;
			p++;
		}

		glBindTexture(GL_TEXTURE_2D, bg_texture[IMAGE_TEXTURE]);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
			pDesc.Width,  pDesc.Height,
			0, GL_RGBA, GL_UNSIGNED_BYTE, pBuffer);
		pTexture->UnlockRect(0);
	}else{
		STDERR("Buffer length of received texture is bogus\r\n");
	}

	pNuiSensor->NuiImageStreamReleaseFrame( pVideoStreamHandle, &imageFrame );
}

void storeNuiDepth(void)
{
	NUI_IMAGE_FRAME depthFrame;

	if(WAIT_OBJECT_0 != WaitForSingleObject(hNextDepthFrameEvent, 0)) return;

	HRESULT hr = pNuiSensor->NuiImageStreamGetNextFrame(
		pDepthStreamHandle,
		0,
		&depthFrame );
	if( FAILED( hr ) ){
		return;
	}
	if(depthFrame.eImageType != NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX)
		STDERR("Depth type is not match with the depth and players\r\n");

	INuiFrameTexture *pTexture = depthFrame.pFrameTexture;
	NUI_LOCKED_RECT LockedRect;
	pTexture->LockRect( 0, &LockedRect, NULL, 0 );
	if( LockedRect.Pitch != 0 ){
		unsigned short *pBuffer = (unsigned short *)LockedRect.pBits;
		memcpy(depth, LockedRect.pBits, pTexture->BufferLen());
#if defined(USE_FACETRACKER)
		setDepthImage(LockedRect.pBits, LockedRect.size);
#endif

		NUI_SURFACE_DESC pDesc;
		pTexture->GetLevelDesc(0, &pDesc);
		//printf("w: %d, h: %d, byte/pixel: %d\r\n", pDesc.Width, pDesc.Height, LockedRect.Pitch/pDesc.Width);

		unsigned short *p = (unsigned short *)pBuffer;
		for(int i=0;i<pTexture->BufferLen()/2;i++){
			//*p = (unsigned short)((*p & 0xff00)>>8) | ((*p & 0x00ff)<<8);	// for test
			//*p = (unsigned short)((*p & 0xfff8)>>3);
			*p = (unsigned short)(NuiDepthPixelToDepth(*pBuffer));
			p++;
		}
		glBindTexture(GL_TEXTURE_2D, bg_texture[DEPTH_TEXTURE]);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 2);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE,
			pDesc.Width,  pDesc.Height,
			0, GL_LUMINANCE, GL_UNSIGNED_SHORT, pBuffer);
		pTexture->UnlockRect(0);
	}
	else{
		STDERR("Buffer length of received texture is bogus\r\n");
	}
	pNuiSensor->NuiImageStreamReleaseFrame( pDepthStreamHandle, &depthFrame );
}

void storeNuiSkeleton(void)
{
	if(WAIT_OBJECT_0 != WaitForSingleObject(hNextSkeletonEvent, 0)) return;

	NUI_SKELETON_FRAME SkeletonFrame = {0};
	HRESULT hr = pNuiSensor->NuiSkeletonGetNextFrame( 0, &SkeletonFrame );

	bool bFoundSkeleton = false;
	for( int i = 0 ; i < NUI_SKELETON_COUNT ; i++ ){
		if( SkeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED ){
			bFoundSkeleton = true;
			trackedDataIndex = i;
		}
	}

	tracked = bFoundSkeleton;
	// no skeletons!
	if( !bFoundSkeleton )
		return;

	// smooth out the skeleton data
	pNuiSensor->NuiTransformSmooth(&SkeletonFrame,NULL);

	// store each skeleton color according to the slot within they are found.
	for( int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )
	{
		if( (SkeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED)){
			memcpy(skels[i], SkeletonFrame.SkeletonData[i].SkeletonPositions, sizeof(Vector4)*NUI_SKELETON_POSITION_COUNT);
		}
	}
}

void drawTexture(TEXTURE_INDEX index)
{
	const short vertices[] = {
		0,	  0,
		DEFAULT_WIDTH,	  0,
		0,	DEFAULT_HEIGHT,
		DEFAULT_WIDTH,	DEFAULT_HEIGHT,
	};
	const GLfloat texCoords[] = {
		1.0f,	1.0f,
		0.0f,	1.0f,
		1.0f,	0.0f,
		0.0f,	0.0f,
	};
	glEnable(GL_TEXTURE_2D);
	glShadeModel(GL_FLAT);
	glDisableClientState(GL_COLOR_ARRAY);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(2, GL_SHORT, 0, vertices);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glTexCoordPointer(2, GL_FLOAT, 0, texCoords);

	glBindTexture(GL_TEXTURE_2D, bg_texture[index]);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glBindTexture(GL_TEXTURE_2D, 0);

	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisable(GL_TEXTURE_2D);
}

void drawNuiSkeleton(int playerID)
{
	int scaleX = DEFAULT_WIDTH;
	int scaleY = DEFAULT_HEIGHT;
	long x=0,y=0;
	unsigned short depth=0;
	float fx=0,fy=0;
	long cx=0,cy=0;
	int display_pos[NUI_SKELETON_POSITION_COUNT][2];

	for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; i++)
	{
		// Overlay on depth image
		//NuiTransformSkeletonToDepthImage( skels[playerID][i], &fx, &fy);
		//display_pos[i][0] = (int) ( fx / 320.0 * DEFAULT_WIDTH);
		//display_pos[i][1] = (int) ( fy / 240.0 * DEFAULT_HEIGHT);

		// Overlay on color image
		NuiTransformSkeletonToDepthImage( skels[playerID][i], &x, &y, &depth);
		NuiImageGetColorPixelCoordinatesFromDepthPixel(NUI_IMAGE_RESOLUTION_640x480, NULL, x, y, depth, &cx, &cy);
		display_pos[i][0] = (int) cx;
		display_pos[i][1] = (int) cy;
	}

	glColor3ub(255, 255, 0);
	glLineWidth(6);
	glBegin(GL_LINE_STRIP);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_HIP_CENTER][0], scaleY - display_pos[NUI_SKELETON_POSITION_HIP_CENTER][1]);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_SPINE][0], scaleY - display_pos[NUI_SKELETON_POSITION_SPINE][1]);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_SHOULDER_CENTER][0], scaleY - display_pos[NUI_SKELETON_POSITION_SHOULDER_CENTER][1]);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_HEAD][0], scaleY - display_pos[NUI_SKELETON_POSITION_HEAD][1]);
	glEnd();
	glBegin(GL_LINE_STRIP);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_SHOULDER_CENTER][0], scaleY - display_pos[NUI_SKELETON_POSITION_SHOULDER_CENTER][1]);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_SHOULDER_LEFT][0], scaleY - display_pos[NUI_SKELETON_POSITION_SHOULDER_LEFT][1]);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_ELBOW_LEFT][0], scaleY - display_pos[NUI_SKELETON_POSITION_ELBOW_LEFT][1]);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_WRIST_LEFT][0], scaleY - display_pos[NUI_SKELETON_POSITION_WRIST_LEFT][1]);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_HAND_LEFT][0], scaleY - display_pos[NUI_SKELETON_POSITION_HAND_LEFT][1]);
	glEnd();
	glBegin(GL_LINE_STRIP);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_SHOULDER_CENTER][0], scaleY - display_pos[NUI_SKELETON_POSITION_SHOULDER_CENTER][1]);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_SHOULDER_RIGHT][0], scaleY - display_pos[NUI_SKELETON_POSITION_SHOULDER_RIGHT][1]);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_ELBOW_RIGHT][0], scaleY - display_pos[NUI_SKELETON_POSITION_ELBOW_RIGHT][1]);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_WRIST_RIGHT][0], scaleY - display_pos[NUI_SKELETON_POSITION_WRIST_RIGHT][1]);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_HAND_RIGHT][0], scaleY - display_pos[NUI_SKELETON_POSITION_HAND_RIGHT][1]);
	glEnd();
	glBegin(GL_LINE_STRIP);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_HIP_CENTER][0], scaleY - display_pos[NUI_SKELETON_POSITION_HIP_CENTER][1]);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_HIP_LEFT][0], scaleY - display_pos[NUI_SKELETON_POSITION_HIP_LEFT][1]);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_KNEE_LEFT][0], scaleY - display_pos[NUI_SKELETON_POSITION_KNEE_LEFT][1]);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_ANKLE_LEFT][0], scaleY - display_pos[NUI_SKELETON_POSITION_ANKLE_LEFT][1]);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_FOOT_LEFT][0], scaleY - display_pos[NUI_SKELETON_POSITION_FOOT_LEFT][1]);
	glEnd();
	glBegin(GL_LINE_STRIP);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_HIP_CENTER][0], scaleY - display_pos[NUI_SKELETON_POSITION_HIP_CENTER][1]);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_HIP_RIGHT][0], scaleY - display_pos[NUI_SKELETON_POSITION_HIP_RIGHT][1]);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_KNEE_RIGHT][0], scaleY - display_pos[NUI_SKELETON_POSITION_KNEE_RIGHT][1]);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_ANKLE_RIGHT][0], scaleY - display_pos[NUI_SKELETON_POSITION_ANKLE_RIGHT][1]);
		glVertex2i( scaleX - display_pos[NUI_SKELETON_POSITION_FOOT_RIGHT][0], scaleY - display_pos[NUI_SKELETON_POSITION_FOOT_RIGHT][1]);
	glEnd();
	glColor3ub(0, 0, 0);
}

void drawNuiSkeleton3d(int playerID)
{
	glColor3ub(255, 255, 0);
	glLineWidth(6);
	glPushMatrix();
	glBegin(GL_LINE_STRIP);
		glVertex3f(skels[playerID][NUI_SKELETON_POSITION_HIP_CENTER].z, -skels[playerID][NUI_SKELETON_POSITION_HIP_CENTER].x, skels[playerID][NUI_SKELETON_POSITION_HIP_CENTER].y);
		glVertex3f(skels[playerID][NUI_SKELETON_POSITION_SPINE].z, -skels[playerID][NUI_SKELETON_POSITION_SPINE].x, skels[playerID][NUI_SKELETON_POSITION_SPINE].y);
		glVertex3f(skels[playerID][NUI_SKELETON_POSITION_SHOULDER_CENTER].z, -skels[playerID][NUI_SKELETON_POSITION_SHOULDER_CENTER].x, skels[playerID][NUI_SKELETON_POSITION_SHOULDER_CENTER].y);
		//glVertex3f(skels[playerID][NUI_SKELETON_POSITION_HEAD].z, -skels[playerID][NUI_SKELETON_POSITION_HEAD].x, skels[playerID][NUI_SKELETON_POSITION_HEAD].y);
	glEnd();
	glBegin(GL_LINE_STRIP);
		glVertex3f(skels[playerID][NUI_SKELETON_POSITION_SHOULDER_CENTER].z, -skels[playerID][NUI_SKELETON_POSITION_SHOULDER_CENTER].x, skels[playerID][NUI_SKELETON_POSITION_SHOULDER_CENTER].y);
		glVertex3f(skels[playerID][NUI_SKELETON_POSITION_SHOULDER_LEFT].z, -skels[playerID][NUI_SKELETON_POSITION_SHOULDER_LEFT].x, skels[playerID][NUI_SKELETON_POSITION_SHOULDER_LEFT].y);
		glVertex3f(skels[playerID][NUI_SKELETON_POSITION_ELBOW_LEFT].z, -skels[playerID][NUI_SKELETON_POSITION_ELBOW_LEFT].x, skels[playerID][NUI_SKELETON_POSITION_ELBOW_LEFT].y);
		glVertex3f(skels[playerID][NUI_SKELETON_POSITION_WRIST_LEFT].z, -skels[playerID][NUI_SKELETON_POSITION_WRIST_LEFT].x, skels[playerID][NUI_SKELETON_POSITION_WRIST_LEFT].y);
		glVertex3f(skels[playerID][NUI_SKELETON_POSITION_HAND_LEFT].z, -skels[playerID][NUI_SKELETON_POSITION_HAND_LEFT].x, skels[playerID][NUI_SKELETON_POSITION_HAND_LEFT].y);
	glEnd();
	glBegin(GL_LINE_STRIP);
		glVertex3f(skels[playerID][NUI_SKELETON_POSITION_SHOULDER_CENTER].z, -skels[playerID][NUI_SKELETON_POSITION_SHOULDER_CENTER].x, skels[playerID][NUI_SKELETON_POSITION_SHOULDER_CENTER].y);
		glVertex3f(skels[playerID][NUI_SKELETON_POSITION_SHOULDER_RIGHT].z, -skels[playerID][NUI_SKELETON_POSITION_SHOULDER_RIGHT].x, skels[playerID][NUI_SKELETON_POSITION_SHOULDER_RIGHT].y);
		glVertex3f(skels[playerID][NUI_SKELETON_POSITION_ELBOW_RIGHT].z, -skels[playerID][NUI_SKELETON_POSITION_ELBOW_RIGHT].x, skels[playerID][NUI_SKELETON_POSITION_ELBOW_RIGHT].y);
		glVertex3f(skels[playerID][NUI_SKELETON_POSITION_WRIST_RIGHT].z, -skels[playerID][NUI_SKELETON_POSITION_WRIST_RIGHT].x, skels[playerID][NUI_SKELETON_POSITION_WRIST_RIGHT].y);
		glVertex3f(skels[playerID][NUI_SKELETON_POSITION_HAND_RIGHT].z, -skels[playerID][NUI_SKELETON_POSITION_HAND_RIGHT].x, skels[playerID][NUI_SKELETON_POSITION_HAND_RIGHT].y);
	glEnd();
	glBegin(GL_LINE_STRIP);
		glVertex3f(skels[playerID][NUI_SKELETON_POSITION_HIP_CENTER].z, -skels[playerID][NUI_SKELETON_POSITION_HIP_CENTER].x, skels[playerID][NUI_SKELETON_POSITION_HIP_CENTER].y);
		glVertex3f(skels[playerID][NUI_SKELETON_POSITION_HIP_LEFT].z, -skels[playerID][NUI_SKELETON_POSITION_HIP_LEFT].x, skels[playerID][NUI_SKELETON_POSITION_HIP_LEFT].y);
		glVertex3f(skels[playerID][NUI_SKELETON_POSITION_KNEE_LEFT].z, -skels[playerID][NUI_SKELETON_POSITION_KNEE_LEFT].x, skels[playerID][NUI_SKELETON_POSITION_KNEE_LEFT].y);
		glVertex3f(skels[playerID][NUI_SKELETON_POSITION_ANKLE_LEFT].z, -skels[playerID][NUI_SKELETON_POSITION_ANKLE_LEFT].x, skels[playerID][NUI_SKELETON_POSITION_ANKLE_LEFT].y);
		glVertex3f(skels[playerID][NUI_SKELETON_POSITION_FOOT_LEFT].z, -skels[playerID][NUI_SKELETON_POSITION_FOOT_LEFT].x, skels[playerID][NUI_SKELETON_POSITION_FOOT_LEFT].y);
	glEnd();
	glBegin(GL_LINE_STRIP);
		glVertex3f(skels[playerID][NUI_SKELETON_POSITION_HIP_CENTER].z, -skels[playerID][NUI_SKELETON_POSITION_HIP_CENTER].x, skels[playerID][NUI_SKELETON_POSITION_HIP_CENTER].y);
		glVertex3f(skels[playerID][NUI_SKELETON_POSITION_HIP_RIGHT].z, -skels[playerID][NUI_SKELETON_POSITION_HIP_RIGHT].x, skels[playerID][NUI_SKELETON_POSITION_HIP_RIGHT].y);
		glVertex3f(skels[playerID][NUI_SKELETON_POSITION_KNEE_RIGHT].z, -skels[playerID][NUI_SKELETON_POSITION_KNEE_RIGHT].x, skels[playerID][NUI_SKELETON_POSITION_KNEE_RIGHT].y);
		glVertex3f(skels[playerID][NUI_SKELETON_POSITION_ANKLE_RIGHT].z, -skels[playerID][NUI_SKELETON_POSITION_ANKLE_RIGHT].x, skels[playerID][NUI_SKELETON_POSITION_ANKLE_RIGHT].y);
		glVertex3f(skels[playerID][NUI_SKELETON_POSITION_FOOT_RIGHT].z, -skels[playerID][NUI_SKELETON_POSITION_FOOT_RIGHT].x, skels[playerID][NUI_SKELETON_POSITION_FOOT_RIGHT].y);
	glEnd();

	glDrawArrowd(
		skels[playerID][NUI_SKELETON_POSITION_HEAD].z, -skels[playerID][NUI_SKELETON_POSITION_HEAD].x, 
		skels[playerID][NUI_SKELETON_POSITION_HEAD].y, skels[playerID][NUI_SKELETON_POSITION_SHOULDER_CENTER].z,
		-skels[playerID][NUI_SKELETON_POSITION_SHOULDER_CENTER].x, skels[playerID][NUI_SKELETON_POSITION_SHOULDER_CENTER].y);
	glColor3ub(0, 0, 0);
	glPopMatrix();
}

/*
 * @brief A general OpenGL initialization function.  Sets all of the initial parameters.
 */
void initGL(int Width, int Height)
{
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);
	glShadeModel(GL_SMOOTH);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0f,(GLfloat)Width/(GLfloat)Height,0.1f,1000.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	for(int i=0;i<TEXTURE_NUM;i++){
		glGenTextures(1, &bg_texture[i]);
		glBindTexture(GL_TEXTURE_2D, bg_texture[i]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	}
}

/*
 * @brief Any User DeInitialization Goes Here
 */
void close(void)
{
	for(int i=0;i<TEXTURE_NUM;i++){
		glDeleteTextures(1, &bg_texture[i]);
	}

	CloseHandle( hNextColorFrameEvent );
	hNextColorFrameEvent = NULL;

	CloseHandle( hNextDepthFrameEvent );
	hNextDepthFrameEvent = NULL;

	CloseHandle( hNextSkeletonEvent );
	hNextSkeletonEvent = NULL;
	if(HasSkeletalEngine(pNuiSensor)) pNuiSensor->NuiSkeletonTrackingDisable();

#if defined(USE_FACETRACKER)
	clearFaceTracker();
#endif

	pNuiSensor->NuiShutdown();
	pNuiSensor->Release();
	pNuiSensor = NULL;
}

/*
 * @brief The function called when our window is resized (which shouldn't happen, because we're fullscreen)
 */
void reSizeGL(int Width, int Height)
{
	if (Height==0)						// Prevent A Divide By Zero If The Window Is Too Small
		Height=1;

	glViewport(0, 0, Width, Height);	// Reset The Current Viewport And Perspective Transformation

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0f,(GLfloat)Width/(GLfloat)Height,0.1f,1000.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

/*
 * @brief The function called whenever a normal key is pressed.
 */
void NormalKeyPressed(unsigned char keys, int x, int y)
{
	if (keys == KEY_ESCAPE) {
		close();
		exit(0);
	}
}

/*
 * @brief The function called whenever a special key is pressed.
 */
void SpecialKeyPressed(int key, int _x, int _y)
{
	switch (key) {
		case GLUT_KEY_F1:
			break;
		case GLUT_KEY_LEFT:
			base[1] += 30;
			break;
		case GLUT_KEY_RIGHT:
			base[1] -= 30;
			break;
		case GLUT_KEY_UP:
			base[2] += 30;
			break;
		case GLUT_KEY_DOWN:
			base[2] -= 30;
			break;
	}
}

void mouseMoved(int x, int y)
{
    if (mx>=0 && my>=0) {
        rotangles[0] += y-my;
        rotangles[1] += x-mx;
    }
    mx = x;
    my = y;
}

void mousePressed(int button, int state, int x, int y)
{
	enum
	{
		MOUSE_LEFT_BUTTON = 0,
		MOUSE_MIDDLE_BUTTON = 1,
		MOUSE_RIGHT_BUTTON = 2,
		MOUSE_SCROLL_UP = 3,
		MOUSE_SCROLL_DOWN = 4
	};

	switch(button){
		case GLUT_LEFT_BUTTON:
			if(state == GLUT_DOWN) {
				mx = x;
				my = y;
			}else if(state == GLUT_UP) {
				mx = -1;
				my = -1;
				//printf("%d:%d Depth[mm]: %d\r\n", x, y, (depth[y/2][x/2]&0xfff8)>>3);
			}
			break;
		case GLUT_RIGHT_BUTTON:
			if(state == GLUT_DOWN) {
			}else if(state == GLUT_UP) {
			}
			break;
		case MOUSE_SCROLL_UP:
			base[0] += 20;
			break;
		case MOUSE_SCROLL_DOWN:
			base[0] -= 20;
			break;
		default:
			;
	}
}

/*
 * @brief The main drawing function.
 */
void drawGL()
{
	time=glutGet(GLUT_ELAPSED_TIME);
	int milliseconds = time - timeprev;
	timeprev = time;

	float dt = milliseconds / 1000.0f;						// Convert Milliseconds To Seconds

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60, (float)win_width/win_height, 0.3, 10000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glPushMatrix();
		glScalef(zoom,zoom,1);
		//glTranslatef(-win_width/2, -win_height/2, 0.0f);
		glRotatef(rotangles[0], 1,0,0);
		glRotatef(rotangles[1], 0,1,0);

		glMatrixMode(GL_TEXTURE);
		glLoadIdentity();
		glScalef(1.0f/win_width,1.0f/win_height,1);
		glMatrixMode(GL_MODELVIEW);
		gluLookAt(  base[0], base[1], base[2],
					base[0]+1, base[1], base[2],
					0, 0, 1);

		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_SHORT, 0, CloudMap);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glTexCoordPointer(3, GL_SHORT, 0, TexMap);

		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, bg_texture[IMAGE_TEXTURE]);

		glPointSize(2.0f);
		glDrawElements(GL_POINTS, win_width*win_height, GL_UNSIGNED_INT, indices);
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);
		glDisable(GL_TEXTURE_2D);
	glPopMatrix();
	
	glPushMatrix();
		glScalef(zoom,zoom,1);
		glRotatef(rotangles[0], 1,0,0);
		glRotatef(rotangles[1], 0,1,0);

		glMatrixMode(GL_TEXTURE);
		glLoadIdentity();
		glScalef(1.0f/win_width,1.0f/win_height,1);
		glMatrixMode(GL_MODELVIEW);
		gluLookAt(  base[0]/1000.0, base[1]/1000.0, base[2]/1000.0,
					base[0]/1000.0+0.001, base[1]/1000.0, base[2]/1000.0,
					0, 0, 1);
		drawNuiSkeleton3d(trackedDataIndex);
	glPopMatrix();

	//drawTexture(DEPTH_TEXTURE);
	//drawTexture(IMAGE_TEXTURE);
	//drawNuiSkeleton(trackedDataIndex);

	glFlush ();					// Flush The GL Rendering Pipeline
	glutSwapBuffers();			// swap buffers to display, since we're double buffered.
}

void idleGL()
{
	storeNuiDepth();
	storeNuiImage();
	storeNuiSkeleton();
	makeCloudMap();

#if defined(USE_FACETRACKER)
	storeFace(trackedDataIndex);
#endif
	glutPostRedisplay();
}

int main(int argc , char ** argv) {
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
	glutInitWindowSize(DEFAULT_WIDTH, DEFAULT_HEIGHT);
	glutInitWindowPosition(0, 0);
	window = glutCreateWindow("Kinect SDK v1.5 with OpenGL");
	glutReshapeFunc(&reSizeGL);
	glutDisplayFunc(&drawGL);
	glutIdleFunc(&idleGL);
	glutKeyboardFunc(&NormalKeyPressed);
	glutSpecialFunc(&SpecialKeyPressed);
	glutMouseFunc(&mousePressed);
	glutMotionFunc(&mouseMoved);

	initNui();
	initGL(DEFAULT_WIDTH, DEFAULT_HEIGHT);
	
	glutMainLoop();

	return 0;
}
