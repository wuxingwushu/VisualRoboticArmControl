#ifndef _RKNN_YOLOV5_DEMO_PWM_H_
#define _RKNN_YOLOV5_DEMO_PWM_H_

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>

class PWM
{
private:
    std::string PwnPhtn = "/sys/class/pwm/pwmchip";
    unsigned int mDutyCycle = 0;
    unsigned int mPeriod;
public:
    PWM(unsigned int PwmID, unsigned int period, bool polarity = true);
    ~PWM();

    unsigned int GetDutyCycle();
    void SetDutyCycle(unsigned int duty_cycle);
};

PWM::PWM(unsigned int PwmID, unsigned int period, bool polarity): mPeriod(period)
{
    printf("PWM : %d\n", PwmID);
    PwnPhtn += std::to_string(PwmID);
    // 导出PWM通道到用户空间
    FILE *pwm_export = fopen((PwnPhtn + "/export").c_str(), "w");
    if (!pwm_export) {
        perror("Failed to open PWM export");
        return;
    }
    fprintf(pwm_export, "0");
    fclose(pwm_export);

    // 设置PWM周期
    FILE *period_file = fopen((PwnPhtn +  "/pwm0/period").c_str(), "w");
    if (!period_file) {
        perror("Failed to open PWM period");
        return;
    }
    fprintf(period_file, "%d", mPeriod);
    fclose(period_file);

    // 设置PWM极性正常或翻转
    FILE *polarity_file = fopen((PwnPhtn +  "/pwm0/polarity").c_str(), "w");
    if (!polarity_file) {
        perror("Failed to open PWM polarity");
        return;
    }
    fprintf(polarity_file, polarity ? "normal" : "inversed");
    fclose(polarity_file);

    // 启用PWM信号的输出
    FILE *enable_file = fopen((PwnPhtn +  "/pwm0/enable").c_str(), "w");
    if (!enable_file) {
        perror("Failed to open PWM enable");
        return;
    }
    fprintf(enable_file, "1");
    fclose(enable_file);
}

PWM::~PWM()
{
    // 取消导出PWM通道到用户空间
    FILE *pwm_unexport = fopen((PwnPhtn + "/unexport").c_str(), "w");
    if (!pwm_unexport) {
        perror("Failed to open PWM unexport");
        return;
    }
    fprintf(pwm_unexport, "0");
    fclose(pwm_unexport);
}

unsigned int PWM::GetDutyCycle() {
    return mDutyCycle;
}

void PWM::SetDutyCycle(unsigned int duty_cycle) {
    // 设置PWM占空比
    FILE *duty_cycle_file = fopen((PwnPhtn + "/pwm0/duty_cycle").c_str(), "w");
    if (!duty_cycle_file) {
        perror("Failed to open PWM duty cycle");
        return;
    }
    if(duty_cycle > mPeriod){
        mDutyCycle = mPeriod;
    }else{
        mDutyCycle = duty_cycle;
    }
    fprintf(duty_cycle_file, "%d", mDutyCycle);
    fclose(duty_cycle_file);
}


#endif
