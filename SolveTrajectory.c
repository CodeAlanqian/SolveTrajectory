// 弹道结算
// 只考虑水平方向的空气阻力

#include <math.h>
#include <stdio.h>

#include "SolveTrajectory.h"

struct SolveTrajectory st;
float t = 0.0f;
/*
@param pitch:rad
@param yaw:rad
@param v:m/s
@param k:弹道系数
*/
void GimbalControlInit(float pitch, float yaw, float v, float k)
{
    st.current_pitch = pitch;
    st.current_yaw = yaw;
    st.current_v = v;
    st._k = k;
    printf("init %f,%f,%f,%f\n", st.current_pitch, st.current_yaw, st.current_v, st._k);
}

/*
@param x:m
@param v:m/s
@param angle:rad
@return y:m
*/
float GimbalControlBulletModel(float x, float v, float angle)
{
    float y;
    t = (float)((exp(st._k * x) - 1) / (st._k * v * cos(angle)));
    y = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
    printf("model %f %f\n", t, y);
    return y;
}

// x:distance , y: height
float GimbalControlGetPitch(float x, float y, float v)
{
    float y_temp, y_actual, dy;
    float angle_pitch;
    y_temp = y;
    // iteration
    int i = 0;
    for (i = 0; i < 20; i++)
    {
        angle_pitch = (float)atan2(y_temp, x); // rad
        y_actual = GimbalControlBulletModel(x, v, angle_pitch);
        dy = y - y_actual;
        y_temp = y_temp + dy;
        printf("iteration num %d: angle_pitch %f,temp target y:%f,err of y:%f\n", i + 1, angle_pitch * 180 / PI, y_temp, dy);
        if (fabsf(dy) < 0.000001)
        {
            break;
        }
    }
    return angle_pitch;
}

/*GimbalControlTransform()
@param xw:ROS坐标系下的x
@param yw:ROS坐标系下的y
@param zw:ROS坐标系下的z
@param vxw:ROS坐标系下的vx
@param vyw:ROS坐标系下的vy
@param vzw:ROS坐标系下的vz
@param timestamp_start:开始时间戳
@param pitch:rad  传出pitch
@param yaw:rad    传出yaw
*/
void GimbalControlTransform(float xw, float yw, float zw,
                            float vxw, float vyw, float vzw,
                            int timestamp_start, float *pitch, float *yaw)
{
    float x, y, z, vx, vy, vz;
    int timestamp_now = timestamp_start + 200; // 假设当前时间戳=开始时间戳+1ms
    // TODO：获取当前时间戳

    // 世界坐标系转换到机器人枪管坐标系
    float cos_yaw = cos(st.current_yaw);
    float sin_yaw = sin(st.current_yaw);
    float cos_pitch = cos(st.current_pitch);
    float sin_pitch = sin(st.current_pitch);

    vx = vxw * cos_yaw * cos_pitch - vyw * sin_yaw * cos_pitch - vzw * sin_pitch;
    vy = vxw * sin_yaw + vyw * cos_yaw;
    // vz = vxw * sin_yaw * cos_pitch - vyw * sin_yaw * sin_pitch + vzw * cos_pitch;
    // XOY平面运动，不考虑z方向速度

    // old
    //  x = xw * cos_yaw *cos_pitch - yw * sin_yaw * cos_pitch - zw * sin_pitch + 0.19133;
    //  y = xw * sin_yaw + yw * cos_yaw;
    //  z = xw * sin_yaw * cos_pitch - yw * sin_yaw * sin_pitch + zw * cos_pitch + 0.21265;

    x = xw * cos_yaw * cos_pitch - yw * sin_yaw * -(zw + 0.21265) * cos_yaw * sin_pitch + 0.19133;
    y = xw * sin_yaw * cos_pitch + yw * cos_yaw - zw * sin_yaw * sin_pitch;
    z = xw * sin_pitch + (zw + 0.21265) * cos_pitch;

    *pitch = -GimbalControlGetPitch(sqrt((x) * (x) + (y) * (y)), z, st.current_v);
    // 线性预测
    // int timeDelay = timestamp_now - timestamp_start + t; // 计算通信及解算时间戳延时+子弹飞行时间
    // int timeDelay = t; //子弹飞行时间

    // x = x + vx * (float)(timeDelay / 1000.0);
    // y = y + vy * (float)(timeDelay / 1000.0);

    *yaw = (float)(atan2(y, x));
}

// 从坐标轴正向看向原点，逆时针方向为正

int main()
{
    float tar_x = 0.3, tar_y = 0.2, tar_z = 0;    // target point  s = sqrt(x^2+y^2)
    float tar_vx = 0.1, tar_vy = 0.1, tar_vz = 0; // target velocity
    float pitch = 0;
    float yaw = 0;
    int timestamp = 1;
    // 机器人初始状态
    GimbalControlInit(0, 0, 25, 0.1);
    /*
    /param pitch:rad  传入当前pitch
    /param yaw:rad    传入当前yaw
    /param v:m/s      传入当前弹速
    /param k:弹道系数
    */

    GimbalControlTransform(tar_x, tar_y, tar_z, tar_vx, tar_vy, tar_vz, timestamp, &pitch, &yaw);
    /*
    /param x_fromROS:ROS坐标系下的x
    /param y_fromROS:ROS坐标系下的y
    /param z_fromROS:ROS坐标系下的z
    /param pitch:rad  传出pitch
    /param yaw:rad    传出yaw
    */

    printf("main %f %f ", pitch * 180 / PI, yaw * 180 / PI);
    printf("main %f %f", pitch, yaw);

    return 0;
}
