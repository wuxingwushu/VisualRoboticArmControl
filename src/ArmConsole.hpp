#ifndef _RKNN_YOLOV5_DEMO_ArmConsole_H_
#define _RKNN_YOLOV5_DEMO_ArmConsole_H_

#include "Mechanical_arm.cpp"
#include "PWM.hpp"
#include <complex>
#include <iostream>
#include <chrono>
#include <thread>

class ArmConsole
{
private:
    PWM* mPWM10 = nullptr;
	PWM* mPWM9 = nullptr;
	PWM* mPWM8 = nullptr;
	PWM* mPWM0 = nullptr;
	PWM* mPWM11 = nullptr;
	PWM* mPWM6 = nullptr;

    Arm_State mState;
    Arm_Result mResult;
    Arm_Parameter mParameter;
public:
    ArmConsole(Arm_State state);
    ~ArmConsole();

    //
    bool ArmActionFormulaSingle(Arm_State State);
    //
    bool ArmActionFormula(Arm_State State);
    //
    bool AngleToPwm(Arm_Result Parameter);

    Arm_Result GetResult(){return mResult;};

    Arm_State GetState(){return mState;}

    void Claw(int px){
        mPWM10->SetDutyCycle(px);
    }
};

ArmConsole::ArmConsole(Arm_State state)
{
    mPWM10 = new PWM(10, 3000000);
	mPWM9 = new PWM(9, 3000000);
	mPWM8 = new PWM(8, 3000000);
	mPWM0 = new PWM(0, 3000000);
	mPWM11 = new PWM(11, 3000000);
	mPWM6 = new PWM(6, 3000000);

    mState = state;

    mPWM9->SetDutyCycle(1170000);

    mParameter = {
        {"爪子", true, 700000, 0, 0, 0, 1200000},
        {"旋转", true, 300000, 500000,1170000,1850000, 2650000},// 旋转爪子
        14,
        {"臂CD", false, 350000,460000,1100000,1750000,2150000},
        9.8,
        {"臂BC", true, 450000,880000,1520000,2180000,2600000},
        10.5,
        {"臂AB", true, 750000,700000,1360000,2000000,1950000},
        {"底盘", true, 350000,800000,1470000,2150000,2600000}
    };

    ArmActionFormulaSingle(mState);
}

ArmConsole::~ArmConsole()
{
    delete mPWM10;
    delete mPWM9;
    delete mPWM8;
    delete mPWM0;
    delete mPWM11;
    delete mPWM6;
}

 bool ArmConsole::ArmActionFormulaSingle(Arm_State State){
    mResult = arm_action_formula(mParameter, State);
    if(!mResult.Success){printf("失败\n");return false;}
    return AngleToPwm(mResult);
 }

//
bool ArmConsole::ArmActionFormula(Arm_State State){
    float arm_pos_x;
    float arm_pos_y;
    float arm_pos_z;
    bool Hu;
    if((State.x - mState.x) < 0)arm_pos_x = -0.1f;
    else arm_pos_x = 0.1f;
    if((State.y - mState.y) < 0)arm_pos_y = -0.1f;
    else arm_pos_y = 0.1f;
    if((State.z - mState.z) < 0){arm_pos_z = -0.1f;Hu = true;}
    else{arm_pos_z = 0.1f;Hu = false;}

    unsigned char BS = 3;
    while (BS)
    {
        if(Hu){
            if(arm_pos_x != 0){
                mState.x += arm_pos_x;
                if(fabs(mState.x - State.x) <= fabs(arm_pos_x*2)){
                    arm_pos_x = 0;
                    --BS;
                    if(arm_pos_y == 0){
                        Hu = false;
                    }
                }
            }
            if(arm_pos_y != 0){
                mState.y += arm_pos_y;
                if(fabs(mState.y - State.y) <= fabs(arm_pos_y*2)){
                    arm_pos_y = 0;
                    --BS;
                    if(arm_pos_x == 0){
                        Hu = false;
                    }
                }
            }
        }else{
            if(fabs(mState.z - State.z) <= fabs(arm_pos_z*2)){Hu = true;--BS;}
            else mState.z += arm_pos_z;
        }
        mResult = arm_action_formula(mParameter, mState);
        if(mResult.Success)AngleToPwm(mResult);
        std::chrono::milliseconds delay(10);
        std::this_thread::sleep_for(delay);
    }
    mResult = arm_action_formula(mParameter, State);
    mState = State;
    return AngleToPwm(mResult);
}


//
bool ArmConsole::AngleToPwm(Arm_Result Result){
    Arm_Result Parameter{};
    Parameter.AngleBCD = M_PI - Result.AngleBCD;
    Parameter.AngleABC = M_PI - Result.AngleABC;
    Parameter.AngleOAB = Result.AngleOAB - (M_PI / 2);
    Parameter.AngleTerritory = (M_PI / 2) - Result.AngleTerritory;

    //printf("AngleBCD:%f\n", Parameter.AngleBCD);
    //printf("AngleABC:%f\n", Parameter.AngleABC);
    //printf("AngleOAB:%f\n", Parameter.AngleOAB);
    //printf("AngleTerritory:%f\n", Parameter.AngleTerritory);

    unsigned int AngleBCD = mParameter.ArmC.GetPwm(Parameter.AngleBCD);
    if(AngleBCD == 0)return false;
    unsigned int AngleABC = mParameter.ArmB.GetPwm(Parameter.AngleABC);
    if(AngleABC == 0)return false;
    unsigned int AngleOAB = mParameter.ArmA.GetPwm(Parameter.AngleOAB);
    if(AngleOAB == 0)return false;
    unsigned int Territory = mParameter.ArmP.GetPwm(Parameter.AngleTerritory);
    if(Territory == 0)return false;
    
    //printf("AngleBCD:%d\n", AngleBCD);
    mPWM8->SetDutyCycle(AngleBCD);
    //printf("AngleABC:%d\n", AngleABC);
    mPWM0->SetDutyCycle(AngleABC);
    //printf("AngleOAB:%d\n", AngleOAB);
    mPWM11->SetDutyCycle(AngleOAB);
    //printf("Territory:%d\n", Territory);
    mPWM6->SetDutyCycle(Territory);

    return true;
}


#endif
