#ifndef __SOLVETRAJECTORY_H__
#define __SOLVETRAJECTORY_H__
#ifndef PI
#define PI 3.1415926535f
#endif
#define GRAVITY 9.78

struct SolveTrajectory
{
    float current_v;      //当前弹速
    float _k;             //弹道系数
    float current_pitch;  //当前pitch
    float current_yaw;    //当前yaw

    float tar_yaw;        //目标yaw
    float v_yaw;
    float tar_r1;        //目标中心到前后装甲板的距离
    float tar_r2;        //目标中心到左右装甲板的距离
    float z2;            //目标中心到左右装甲板的距离
};

struct tar_pos
{
    float x;
    float y;
    float z;
    float yaw;
};

extern void GimbalControlInit(float pitch, float yaw, float tar_yaw , float v_yaw, float r1, float r2, float z2, float v, float k);
extern float GimbalControlBulletModel(float x, float v, float angle);
extern float GimbalControlGetPitch(float x, float y, float v);

extern void GimbalControlTransform(float xw, float yw, float zw,
                                float vxw, float vyw, float vzw,
                                int timestamp_start, float *pitch, float *yaw,
                                float *aim_x, float *aim_y, float *aim_z);

#endif /*__SOLVETRAJECTORY_H__*/