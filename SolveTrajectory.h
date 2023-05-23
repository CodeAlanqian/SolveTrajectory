#ifndef __SOLVETRAJECTORY_H__
#define __SOLVETRAJECTORY_H__
#ifndef PI
#define PI 3.1415926535f
#endif
#define GRAVITY 9.78
typedef unsigned char uint8_t;
enum ARMOR_ID
{
    ARMOR_OUTPOST = 0,
    ARMOR_HERO = 1,
    ARMOR_ENGINEER = 2,
    ARMOR_INFANTRY3 = 3,
    ARMOR_INFANTRY4 = 4,
    ARMOR_INFANTRY5 = 5,
    ARMOR_GUARD = 6,
    ARMOR_BASE = 7
};

enum ARMOR_NUM
{
    ARMOR_NUM_BALANCE = 2,
    ARMOR_NUM_OUTPOST = 3,
    ARMOR_NUM_NORMAL = 4
};

enum BULLET_TYPE
{
    BULLET_17 = 0,
    BULLET_42 = 1
};


//设置参数
struct SolveTrajectoryParams
{
    float k;             //弹道系数

    //自身参数
    enum BULLET_TYPE bullet_type;  //自身机器人类型 0-步兵 1-英雄
    float current_v;      //当前弹速
    float current_pitch;  //当前pitch
    float current_yaw;    //当前yaw

    //目标参数
    float xw;             //ROS坐标系下的x
    float yw;             //ROS坐标系下的y
    float zw;             //ROS坐标系下的z
    float vxw;            //ROS坐标系下的vx
    float vyw;            //ROS坐标系下的vy
    float vzw;            //ROS坐标系下的vz
    float tar_yaw;        //目标yaw
    float v_yaw;          //目标yaw速度
    float r1;             //目标中心到前后装甲板的距离
    float r2;             //目标中心到左右装甲板的距离
    float dz;             //另一对装甲板的相对于被跟踪装甲板的高度差
    int bias_time;        //偏置时间
    float s_bias;         //枪口前推的距离
    float z_bias;         //yaw轴电机到枪口水平面的垂直距离
    enum ARMOR_ID armor_id;     //装甲板类型  0-outpost 6-guard 7-base
                                //1-英雄 2-工程 3-4-5-步兵 
    enum ARMOR_NUM armor_num;   //装甲板数字  2-balance 3-outpost 4-normal
};

//用于存储目标装甲板的信息
struct tar_pos
{
    float x;           //装甲板在世界坐标系下的x
    float y;           //装甲板在世界坐标系下的y
    float z;           //装甲板在世界坐标系下的z
    float yaw;         //装甲板坐标系相对于世界坐标系的yaw角
};
//单方向空气阻力模型
extern float monoDirectionalAirResistanceModel(float s, float v, float angle);
//完全空气阻力模型
extern float completeAirResistanceModel(float s, float v, float angle);
//pitch弹道补偿
extern float pitchTrajectoryCompensation(float s, float y, float v);
//根据最优决策得出被击打装甲板 自动解算弹道
extern void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z);


#endif /*__SOLVETRAJECTORY_H__*/