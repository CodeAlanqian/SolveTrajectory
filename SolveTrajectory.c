// 弹道结算
//只考虑水平方向的空气阻力

#include <math.h>
#include <stdio.h>

#include "SolveTrajectory.h"

struct SolveTrajectory st;

void GimbalContrlInit(float x,float y,float z,float pitch,float yaw, float v, float k) {
    st._x = x;
    st._y = y;
    st._z = z;
    st.offset_pitch = pitch;
    st.offset_yaw = yaw;
    st._v = v;
    st._k = k;
    printf("init\n");
    printf("%f,%f,%f,%f,%f,%f,%f\n",st._x,st._y,st._z,st.offset_pitch,st.offset_yaw,st._v,st._k);
}


float GimbalContrlBulletModel(float x, float v, float angle) { //x:m,v:m/s,angle:rad
    float t, y;
    t = (float)((exp(st._k * x) - 1) / (st._k * v * cos(angle)));
    y = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
    printf("model %f %f\n",t,y);
	return y;
}

//x:distance , y: height
float GimbalContrlGetPitch(float x, float y, float v) {
    float y_temp, y_actual, dy;
    float angle;
    y_temp = y;
    //printf("iteration\n");
    
    //iteration
    int i = 0;
    for (i = 0; i < 20; i++) {
    angle = (float) atan2(y_temp, x);
    y_actual = GimbalContrlBulletModel(x, v, angle);
    dy = y - y_actual;
    y_temp = y_temp + dy;
    printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,angle*180/PI,y_temp,dy);
    if (fabsf(dy) < 0.000001) {
        break;
    }
    
    

    }
    return angle;

}

void GimbalContrlTransform(float *x, float *y, float *z, float *pitch, float *yaw) {
    *pitch = -GimbalContrlGetPitch(sqrt((*x)*(*x)+(*y)*(*y)), *z, st._v);
    *yaw = (float) (atan2(*x-st._x , *y - st._y ));
}

//从坐标轴正向看向原点，逆时针方向为正

int main(){
    float tar_x = 3, tar_y = 2.2, tar_z = 1.0; //target point  s = sqrt(x^2+y^2) 
    float pitch = 0;
    float yaw = 0;

    //机器人初始状态
    GimbalContrlInit(0, 0, 0, 0, 0, 25, 0.05);
    
    GimbalContrlTransform(&tar_x, &tar_y, &tar_z, &pitch, &yaw);
    printf("main %f %f",pitch*180/PI,yaw*180/PI);

    return 0;
}
