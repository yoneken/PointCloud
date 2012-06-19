#ifndef __FACE_TRACKER_H__
#define __FACE_TRACKER_H__

#define USE_FACETRACKER

void initFaceTracker(void);
void clearFaceTracker(void);
void storeFace(int playerID);

void setColorImage(unsigned char *, int);
void setDepthImage(unsigned char *, int);

#endif