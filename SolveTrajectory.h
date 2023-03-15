#ifndef __SOLVETRAJECTORY_H__
#define __SOLVETRAJECTORY_H__

#define PI 3.1415926535f
#define GRAVITY 9.78

struct SolveTrajectory
{
    float _x;
    float _y;
    float _z;
    float _v;
    float _k;
    float offset_pitch;
    float offset_yaw;
};


extern void GimbalContrlInit(float x,float y,float z,float pitch,float yaw, float v, float k);
extern float GimbalContrlBulletModel(float x, float v, float angle);
extern float GimbalContrlGetPitch(float x, float y, float v);
extern void GimbalContrlTransform(float *x, float *y, float *z, float *pitch, float *yaw);
#endif /*__SOLVETRAJECTORY_H__*/