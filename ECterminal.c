/*
@author: CodeAlan  华南师大Vanguard战队
*/
// 弹道解算
// 只考虑水平方向的空气阻力

#include <math.h>
#include <stdio.h>

#include "SolveTrajectory.h"

struct SolveTrajectory st;
struct tar_pos tar_position[4];
float t = 0.5f; // 飞行时间

/*
@brief 初始化
@param pitch:rad
@param yaw:rad
@param v:m/s
@param k:弹道系数
*/
void GimbalControlInit(float pitch, float yaw, float tar_yaw , float v_yaw, float r1, float r2, float z2, float v, float k)
{
    st.current_pitch = pitch;
    st.current_yaw = yaw;
    st.current_v = v;
    st._k = k;
    st.tar_yaw = tar_yaw;
    st.v_yaw = v_yaw;
    st.tar_r1 = r1;
    st.tar_r2 = r2;
    st.z2 = z2;
}

/*
@brief 弹道模型
@param x:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return y:m
*/
float GimbalControlBulletModel(float x, float v, float angle)
{
    float y;
    t = (float)((exp(st._k * x) - 1) / (st._k * v * cos(angle)));
    y = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
    return y;
}

/*
@brief pitch轴解算
@param x:m 距离
@param y:m 高度
@param v:m/s
@return angle_pitch:rad
*/
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
        dy = 0.3*(y - y_actual);
        y_temp = y_temp + dy;
        if (fabsf(dy) < 0.00001)
        {
            break;
        }
    }
    return angle_pitch;
}

/*
@brief 世界坐标系转换到云台坐标系
@param xw:ROS坐标系下的x
@param yw:ROS坐标系下的y
@param zw:ROS坐标系下的z
@param vxw:ROS坐标系下的vx
@param vyw:ROS坐标系下的vy
@param vzw:ROS坐标系下的vz
@param timestamp_start:开始时间戳
@param pitch:rad  传出pitch
@param yaw:rad    传出yaw
@param aim_x:传出aim_x  打击目标的x
@param aim_y:传出aim_y  打击目标的y
@param aim_z:传出aim_z  打击目标的z
*/
void GimbalControlTransform(float xw, float yw, float zw,
                            float vxw, float vyw, float vzw,
                            int timestamp_start, float *pitch, float *yaw,
                            float *aim_x, float *aim_y, float *aim_z)
{
    float x_static = 0.19133; //相机前推的距离
    float z_static = 0.21265; //yaw轴电机到枪口水平面的垂直距离
    // TODO：获取当前时间戳
    int timestamp_now = timestamp_start + 1000; // 假设当前时间戳=开始时间戳+1000ms

    // 线性预测
    // 计算通信及解算时间戳延时+子弹飞行时间

    float timeDelay = (float)((timestamp_now - timestamp_start)/1000.0) + t;
    st.tar_yaw += st.v_yaw * t;
    //计算四块装甲板的位置
	int use_1 = 1;
	int i = 0;

	for (i = 0; i<4; i++) {
		float tmp_yaw = st.tar_yaw + i * PI/2.0;
		float r = use_1 ? st.tar_r1 : st.tar_r2;
		tar_position[i].x = xw - r*cos(tmp_yaw);
		tar_position[i].y = yw - r*sin(tmp_yaw);
		tar_position[i].z = use_1 ? zw : st.z2;
		tar_position[i].yaw = st.tar_yaw + i * PI/2.0;
		use_1 = !use_1;
	}

    //两种决策方案：
    //1.计算枪管到目标装甲板yaw小的那个装甲板
    //2.计算距离最近的装甲板

	//计算距离最近的装甲板
//	float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
//	int index = 0;
//	for (i = 1; i<4; i++)
//	{
//		float temp_dis_diff = sqrt(tar_position[i].x * tar_position[0].x + tar_position[i].y * tar_position[0].y);
//		if (temp_dis_diff < dis_diff_min)
//		{
//			dis_diff_min = temp_dis_diff;
//			index = i;
//		}
//	}
//
	//计算枪管到目标装甲板yaw小的那个装甲板
		float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
		int index = 0;
		for (i = 1; i<4; i++)
		{
			float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
			if (temp_yaw_diff < yaw_diff_min)
			{
				yaw_diff_min = temp_yaw_diff;
				index = i;
			}
		}


    *aim_z = tar_position[index].z + vzw * timeDelay;
    *pitch = -GimbalControlGetPitch(sqrt(tar_position[index].x * tar_position[index].x + tar_position[index].y * tar_position[index].y) + x_static,
            tar_position[index].z - z_static, st.current_v);

    *aim_x = tar_position[index].x + vxw * timeDelay;
    *aim_y = tar_position[index].y + vyw * timeDelay;

    *yaw = (float)(atan2(*aim_y, *aim_x));

}

// 从坐标轴正向看向原点，逆时针方向为正