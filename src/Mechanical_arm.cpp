#include "Mechanical_arm.hpp"
#include <complex>


// 绕 X 轴旋转函数
void rotateX(double radian, double &y, double &z) {
    double tempY = y;
    y = y * cos(radian) - z * sin(radian);
    z = tempY * sin(radian) + z * cos(radian);
}

// 绕 Y 轴旋转函数
void rotateY(double radian, double &x, double &z) {
    double tempZ = z;
    z = z * cos(radian) - x * sin(radian);
    x = tempZ * sin(radian) + x * cos(radian);
}

// 绕 Z 轴旋转函数
void rotateZ(double radian, double &x, double &y) {
    double tempX = x;
    x = x * cos(radian) - y * sin(radian);
    y = tempX * sin(radian) + y * cos(radian);
}

//机械臂运动学公式    外三角
Arm_Result arm_action_formula(Arm_Parameter& Parameter, Arm_State& State)
{
    Arm_Result Result;
    Result.Success = false;
    if(State.z < -7){printf("[Error]: arm_action_formula z < -7");return Result;}
    int y_zf = 1; 
    int z_zf = 1;
    int x_zf = 1;
    int PWM[5];
     
    //判断   Y   Z    的正负值     用于需改方向
    if(State.x < 0)x_zf = -1;
    if(State.y < 0)y_zf = -1;
    if(State.z < 0)z_zf = -1;
     
    //获取 x y 平面的距离
    double XY_Distance =  sqrt(pow(State.x,2) + pow(State.y,2));
    //获取 目标 的距离
    double ArmAD = sqrt(pow(XY_Distance,2) + pow(State.z,2));                                                                   
    if(ArmAD < 3){printf("[Error]: arm_action_formula ArmAD < 3 距离过小");return Result;}//距离过小，强制结束。
    double AC_Max,AC_Min;//储存 设置边的最大最小值 的距离
    if((Parameter.ArmBC + Parameter.ArmCD) > (ArmAD + Parameter.ArmAB)){AC_Max = ArmAD + Parameter.ArmAB;}
    else{AC_Max = Parameter.ArmBC + Parameter.ArmCD;}
    if(fabs(Parameter.ArmBC - Parameter.ArmCD) > fabs(ArmAD - Parameter.ArmAB)){AC_Min = fabs(Parameter.ArmBC - Parameter.ArmCD);}
    else {AC_Min = fabs(ArmAD - Parameter.ArmAB);}
    double ArmBD = (AC_Min + AC_Max) / 2;//获取  范围内  的中间值
     
    //获的  A  B   C   内三角形三个角度。
    //double angle_BDA = acos((pow(ArmAD,2) + pow(ArmBD ,2) - pow(ArmAB,2)) / (2 * ArmBD * ArmAD));
    double angle_DAB = acos((pow(Parameter.ArmAB, 2) + pow(ArmAD, 2) - pow(ArmBD, 2)) / (2 * ArmAD * Parameter.ArmAB));
    double angle_ABD = acos((pow(Parameter.ArmAB, 2) + pow(ArmBD, 2) - pow(ArmAD, 2)) / (2 * Parameter.ArmAB * ArmBD));
    //获的  A  B   C   外三角形三个角度。
    //double angle_CDB = acos((pow(ArmCD,2) + pow(ArmBD ,2) - pow(ArmBC,2)) / (2 * ArmBD * ArmCD));                                
    double angle_DBC = acos((pow(Parameter.ArmBC, 2) + pow(ArmBD, 2) - pow(Parameter.ArmCD, 2)) / (2 * ArmBD * Parameter.ArmBC)); 
    double angle_BCD = acos((pow(Parameter.ArmCD, 2) + pow(Parameter.ArmBC, 2) - pow(ArmBD, 2)) / (2 * Parameter.ArmCD * Parameter.ArmBC));
     
    double di_pan;
    if(State.x != 0){
        di_pan = acos((pow((XY_Distance),2) + pow(State.x,2) - pow(State.y,2)) / (2 * fabs(State.x) * XY_Distance));//获得  地盘 的旋转角度
    }else if(State.y != 0){                                                                                               //分母为零的处理情况
        di_pan = M_PI / 2;
    }else{
        di_pan = 0;
    }
    if(x_zf == -1)di_pan = M_PI - di_pan;
    //获的  地盘中心到伪目标点  的线段  和   地盘平面的角度
    double angle_yang = z_zf * acos((pow(ArmAD,2) + pow(XY_Distance,2) - pow(State.z,2)) / (2 * XY_Distance * ArmAD));
    //if(x_zf == -1)angle_yang = M_PI - angle_yang;
     
    Result.AngleBCD = angle_BCD;
    Result.AngleABC = angle_ABD + angle_DBC;
    Result.AngleOAB = angle_yang + angle_DAB;
    Result.AngleTerritory = di_pan;
    
    Result.Success = true;
    return Result;
}


//机械臂运动学公式    外三角     返回值 是 坐标是否执行    ***可以控制爪子的朝向  向下为  90 度  顺时针  90 ~ -90 的范围  (可以理解为爪子中心就是原点 ，一平台为平面，形成的角度)
Arm_Result arm_action_formula_vertical(Arm_Parameter& Parameter, Arm_State& State)
{
    Arm_Result Result;
    Result.Success = false;
    if(State.z < -13){printf("[Error]: arm_action_formula z < -7");return Result;}
    int y_zf = 1; 
    int z_zf = 1;
    int x_zf = 1;
    int PWM[5];
    
    if(State.x < 0)x_zf = -1;
    if(State.y < 0)y_zf = -1;                                                                                                 //判断   Y   Z    的正负值     用于需改方向
    if(State.z < 0)z_zf = -1;
     
    double XY_Distance =  sqrt(pow(State.x,2) + pow(State.y,2));
    double ArmAD = sqrt(pow(XY_Distance,2) + pow(State.z,2));
    if(ArmAD < 3){printf("[Error]: arm_action_formula ArmAD < 3 距离过小");return Result;}
    double angle_yang = z_zf * acos((pow(ArmAD,2) + pow(XY_Distance,2) - pow(State.z,2)) / (2 * XY_Distance * ArmAD));              //获的  地盘中心到伪目标点  的线段  和   地盘平面的角度
    //if(x_zf == -1)angle_yang = pi - angle_yang;
    
    double AC_Max,AC_Min;                                                                                                //储存 设置边的最大最小值 的距离
    if((ArmAD + Parameter.ArmCD) > (Parameter.ArmBC + Parameter.ArmAB)){AC_Max = Parameter.ArmBC + Parameter.ArmAB;}
    else{AC_Max = ArmAD + Parameter.ArmCD;}
    if(fabs(ArmAD - Parameter.ArmCD) > fabs(Parameter.ArmAB - Parameter.ArmBC)){AC_Min = fabs(ArmAD - Parameter.ArmCD);}
    else{AC_Min = fabs(Parameter.ArmAB - Parameter.ArmBC);}
    
    double yuche_angle = (M_PI * (State.angle / 180)) - angle_yang;
    double ArmAC = sqrt(pow(Parameter.ArmCD,2) + pow(ArmAD,2) + (Parameter.ArmCD * ArmAD * cos(yuche_angle)));
    
    if(((AC_Max > ArmAC) && (ArmAC > AC_Min)) == 0) { printf("机械结构限制"); return Result; }
    
    double angle_BCA = acos((pow(Parameter.ArmBC, 2) + pow(ArmAC, 2) - pow(Parameter.ArmAB, 2)) / (2 * ArmAC * Parameter.ArmBC));                                //获的  A  B   C   内三角形三个角度。
    double angle_CAB = acos((pow(Parameter.ArmAB, 2) + pow(ArmAC, 2) - pow(Parameter.ArmBC, 2)) / (2 * ArmAC * Parameter.ArmAB)); 
    double angle_ABC = acos((pow(Parameter.ArmAB, 2) + pow(Parameter.ArmBC, 2) - pow(ArmAC, 2)) / (2 * Parameter.ArmAB * Parameter.ArmBC));
    
    double angle_DAC = acos((pow(ArmAD, 2) + pow(ArmAC, 2) - pow(Parameter.ArmCD, 2)) / (2 * ArmAC * ArmAD));                                //获的  A  B   C   外三角形三个角度。
    double angle_ACD = acos((pow(Parameter.ArmCD, 2) + pow(ArmAC, 2) - pow(ArmAD, 2)) / (2 * ArmAC * Parameter.ArmCD)); 
    //double angle_CDA = acos((pow(ArmCD, 2) + pow(ArmAD, 2) - pow(ArmAC, 2)) / (2 * ArmCD * ArmAD));
    
    
    double di_pan;
    if(State.x != 0){
       di_pan = acos((pow((XY_Distance),2) + pow(State.x,2) - pow(State.y,2)) / (2 * fabs(State.x) * XY_Distance));                            //获得  地盘 的旋转角度
    }else if(State.y != 0){                                                                                                  //分母为零的处理情况
       di_pan = M_PI / 2;
    }else{
       di_pan = 0;
    }
    if(x_zf == -1)di_pan = M_PI - di_pan;

    Result.AngleBCD = angle_BCA + angle_ACD;
    Result.AngleABC = angle_ABC;
    Result.AngleOAB = angle_yang + angle_CAB + angle_DAC;
    Result.AngleTerritory = di_pan;
    
    Result.Success = true;
    return Result;
}


//机械臂 帧动作，单个单个轴移动
bool arm_action_frame(Arm_State& State, float x, float y, float z,bool lei)
{
    float arm_pos_x;
    float arm_pos_y;
    float arm_pos_z;
    if((x - State.x) < 0)arm_pos_x = -0.1f;
    else arm_pos_x = 0.1f;
    if((y - State.y) < 0)arm_pos_y = -0.1f;
    else arm_pos_x = 0.1f;
    if((z - State.z) < 0)arm_pos_z = -0.1f;
    else arm_pos_x = 0.1f;

    while (1)
    {
        /* code */
    }
    
    
    return 1;
}
