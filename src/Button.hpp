#ifndef _RKNN_YOLOV5_DEMO_Button_H_
#define _RKNN_YOLOV5_DEMO_Button_H_

#include "GPIO.hpp"

class Button
{
private:
    const bool zhi;
    bool state;
    unsigned int Count = 0;
    const unsigned int mCount;
    GPIO* Gpio = nullptr;
public:
    Button(unsigned int ID, const bool z, const unsigned int count = 10);
    ~Button();

    bool Get();
    bool GetLongPress();
};

Button::Button(unsigned int ID, const bool z, const unsigned int count)
:zhi(z), mCount(count)
{
    Gpio = new GPIO(ID, 0);
}

Button::~Button()
{
    delete Gpio;
}

bool Button::Get(){
    bool z = Gpio->GetValue();
    if((state != z) && (z == zhi)){
        state = z;
        return true;
    }else{
        state = z;
        return false;
    }
}

bool Button::GetLongPress(){
    bool z = Gpio->GetValue();
    if(z == zhi){
        ++Count;
        return false;
    }else{
        if(Count >= mCount){
            Count = 0;
            return true;
        }
        Count = 0;
    }
    return false;
}


#endif
