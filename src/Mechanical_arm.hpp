#include <math.h>
#ifndef _RKNN_YOLOV5_DEMO_Arm_H_
#define _RKNN_YOLOV5_DEMO_Arm_H_

/*

爪子            2800 ~ 5000      5000 是夹住               4300  夹住盒子
爪子转向        900  ~ 6500      3800 是平行与平台         2250 和 5450 是垂直与平台 

爪子关节 1 号   1300 ~ 5700      3800 是平行于后一臂       2200 和 5400 是垂直与后一臂   1300 向内                  长   14 CM
爪子关节 2 号   1300 ~ 6400      3800 是平行于后一臂       2200 和 5400 是垂直与后一臂   1300 向内                  长  9.8 CM
爪子关节 3 号   1800 ~ 4900      3400 是垂直与平台         1800 和 4900 是平行与平台     4900 向内                  长 10.5 CM

地盘            900  ~ 6400      3650 朝正前方             2000 朝右    5300 朝左

*/

/*  机械臂抽象图

      B________C
      ╱        ╲
     ╱          ╲
    ╱            ╲
 A ╱               D
■■■■■
┃ P ┃
*/
enum ArmComputeMode{
    OuterTriangle,//外三角( ABD, BCD )
    OuterTriangle_FixedAngle//外三角( ABD, BCD ) 固定角
};

//舵机工作参数
struct Arm_DutyCycle_Angle
{
    const char* Name;
    bool Direction;         //最小值的方向是朝内（true）还是朝外（false）
    unsigned int Min;       //最小值
    unsigned int VerticalT; //靠近最小值的垂直的数值
    unsigned int Parallel;  //平行于后一个机械臂的数值
    unsigned int VerticalF; //靠近最大值的垂直的数值
    unsigned int Max;       //最大值

    //获取朝内垂直的数值
    unsigned int GetVertical(){
        if(Direction)return VerticalT;
        else return VerticalF;
    }

    unsigned int GetPwm(float angle){
        int Pwm = VerticalF - VerticalT;
        Pwm *= angle / M_PI;
        Pwm = Parallel + (Pwm * (Direction ? -1 : 1));
        //printf("%s Ang: %f\n", Name,angle);
        //printf("%s pwm: %d\n", Name,Pwm);
        if(Pwm < Min){
            printf("[Error]: %s Pwm Min, angle:%f\n", Name, angle / M_PI * 180);
            Pwm = 0;
        }
        if(Pwm > Max){
            printf("[Error]: %s Pwm Max, angle:%f\n", Name, angle / M_PI * 180);
            Pwm = 0;
        }
        return Pwm;
    }
};

//机械臂状态
struct Arm_State
{
    //爪子空间坐标(底盘中心为原点)
    float x;
    float y;
    float z;
    float angle;//CD与底盘平面的夹角
    ArmComputeMode mode;//用什么工作模式获取的姿态
};

//机械臂
struct Arm_Result
{
    float AneleArm = 0;//机械臂爪子旋转
    float AngleBCD = 0;//BC与CD的夹角
    float AngleABC = 0;//AB与BC的夹角
    float AngleOAB = 0;//AB与底盘平面的夹角
    float AngleTerritory = 0;//底盘旋转
    bool Success;//是否成功
};

//机械臂参数
struct Arm_Parameter{
    Arm_DutyCycle_Angle ArmZ;   // 爪子 舵机
    Arm_DutyCycle_Angle ArmD;   // 旋转爪子 舵机
    float ArmCD;                // CD臂 长
    Arm_DutyCycle_Angle ArmC;   // CD臂 舵机
    float ArmBC;                // BC臂 长
    Arm_DutyCycle_Angle ArmB;   // BC臂 舵机
    float ArmAB;                // AB臂 长
    Arm_DutyCycle_Angle ArmA;   // AB臂 舵机
    Arm_DutyCycle_Angle ArmP;   // 地盘 舵机
};


// 绕 X 轴旋转函数
void rotateX(double angle, double& y, double& z);
// 绕 Y 轴旋转函数
void rotateY(double angle, double& x, double& z);
// 绕 Z 轴旋转函数
void rotateZ(double angle, double& x, double& y);

    
//机械臂运动学公式    外三角     返回值 是 坐标是否执行    ***推荐使用
Arm_Result arm_action_formula(Arm_Parameter& Parameter, Arm_State& State);//爪子角度不方便计算            推荐距离小于  34  CM   高度   -10 CM  —  34 CM

//机械臂运动学公式    外三角     返回值 是 坐标是否执行    ***可以控制爪子的朝向  向下为  90 度  顺时针  90 ~ -90 的范围  (可以理解为爪子中心就是原点 ，一平台为平面，形成的角度)
Arm_Result arm_action_formula_vertical(Arm_Parameter& Parameter, Arm_State& State);//爪子角度可以控制  缺点就是范围减少 

//机械臂 帧动作，单个单个轴移动
bool arm_action_frame(Arm_State& State, float x, float y, float z, bool lei);



#endif
