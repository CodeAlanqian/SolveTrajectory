#ifndef __SOLVETRAJECTORY_H__
#define __SOLVETRAJECTORY_H__

#define PI 3.1415926535f
#define GRAVITY 9.78

struct SolveTrajectory
{
    float current_v;      //当前弹速
    float _k;             //弹道系数
    float current_pitch;  //当前pitch
    float current_yaw;    //当前yaw
};


extern void GimbalContrlInit(float pitch,float yaw, float v, float k);
extern float GimbalContrlBulletModel(float x, float v, float angle);
extern float GimbalContrlGetPitch(float x, float y, float v);
extern void GimbalContrlTransform(float x_fromROS, float y_fromROS, float z_fromROS, float *pitch, float *yaw);

#endif /*__SOLVETRAJECTORY_H__*/