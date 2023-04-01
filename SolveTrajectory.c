// 弹道结算
//只考虑水平方向的空气阻力

#include <math.h>
#include <stdio.h>

#include "SolveTrajectory.h"

struct SolveTrajectory st;

void GimbalControlInit(float pitch,float yaw, float v, float k) {
    st.current_pitch = pitch;
    st.current_yaw = yaw;
    st.current_v = v;
    st._k = k;
    printf("init\n");
    printf("%f,%f,%f,%f\n",st.current_pitch,st.current_yaw,st.current_v,st._k);
}


float GimbalControlBulletModel(float x, float v, float angle) { //x:m,v:m/s,angle:rad
    float t, y;
    t = (float)((exp(st._k * x) - 1) / (st._k * v * cos(angle)));
    y = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
    printf("model %f %f\n",t,y);
	return y;
}

//x:distance , y: height
float GimbalControlGetPitch(float x, float y, float v) {
    float y_temp, y_actual, dy;
    float angle;
    y_temp = y;
    //printf("iteration\n");
    
    //iteration
    int i = 0;
    for (i = 0; i < 20; i++) {
    angle = (float) atan2(y_temp, x);
    y_actual = GimbalControlBulletModel(x, v, angle);
    dy = y - y_actual;
    y_temp = y_temp + dy;
    printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,angle*180/PI,y_temp,dy);
    if (fabsf(dy) < 0.000001) {
        break;
    }
    

    }
    return angle;

}

void GimbalControlTransform(float x_fromROS, float y_fromROS, float z_fromROS, float *pitch, float *yaw) {
    float x, y, z;
    //世界坐标系转换到机器人枪管坐标系
    float cos_yaw = cos(st.current_yaw);
    float sin_yaw = sin(st.current_yaw);
    float cos_pitch = cos(st.current_pitch);
    float sin_pitch = sin(st.current_pitch);

    x = x_fromROS * cos_yaw *cos_yaw - y_fromROS * sin_yaw * cos_yaw - z_fromROS * sin_pitch + 0.19133;
    y = x_fromROS * sin_yaw + y_fromROS * cos_yaw;
    z = x_fromROS * sin_yaw * cos_yaw - y_fromROS * sin_yaw * sin_yaw + z_fromROS * cos_pitch + 0.21265;


    *pitch = -GimbalControlGetPitch(sqrt((x)*(x)+(y)*(y)), z, st.current_v);
    *yaw = (float) (atan2(y ,  x));
}

//从坐标轴正向看向原点，逆时针方向为正

int main(){
    float tar_x = 0.3, tar_y = 0.2, tar_z = 0; //target point  s = sqrt(x^2+y^2) 
    float pitch = 0;
    float yaw = 0;

    //机器人初始状态
    GimbalControlInit(0, 0, 25, 0.1);
    /*
    /param pitch:rad  传入当前pitch
    /param yaw:rad    传入当前yaw
    /param v:m/s      传入当前弹速
    /param k:弹道系数
    */
    
    GimbalControlTransform(tar_x, tar_y, tar_z, &pitch, &yaw);
    /*
    /param x_fromROS:ROS坐标系下的x
    /param y_fromROS:ROS坐标系下的y
    /param z_fromROS:ROS坐标系下的z
    /param pitch:rad  传出pitch
    /param yaw:rad    传出yaw
    */



    printf("main %f %f ",pitch*180/PI,yaw*180/PI);
    printf("main %f %f",pitch,yaw);

    return 0;
}
