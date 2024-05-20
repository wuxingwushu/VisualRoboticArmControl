#ifndef _RKNN_YOLOV5_DEMO_MenuEvents_H_
#define _RKNN_YOLOV5_DEMO_MenuEvents_H_

#include <string>

enum MenuModes{
    MenuVariables,
    MenuSubmenu,
    MenuFunction,
    MenuTxet
};

class MenuEvents
{
private:
    const char* mName;
    const bool mExecute = false;
    MenuModes mModes;
    MenuEvents* wMenuEvents = nullptr;
public:
    std::string str{};

    MenuEvents(const char* name, MenuModes modes = MenuTxet, const bool Execute = false):mName(name), mModes(modes), mExecute(Execute){};
    ~MenuEvents(){};
    const char* Name(){return mName;};
    MenuModes Modes(){return mModes;}
    virtual void Refresh(){}

    virtual void W(){};
    virtual void S(){};
    virtual void A(){};
    virtual void D(){};

    virtual void LongPress(){};

    bool GetExecuteBool(){return mExecute;}
    virtual const char* Show(){ return str.c_str();};
    void SetParentMenu(MenuEvents* w){wMenuEvents = w;};
    MenuEvents* ParentMenu(){return wMenuEvents;};
};

enum VariablesT{
    Double,
    Float,
    Char,
    uChar,
    Bool,
    Int,
    uInt,
    Enum
};

MenuEvents* mMenuXX;

class Variables : public MenuEvents
{
private:
    void* mVariables;
    float mV = 1.0;
    const VariablesT mT;

    const float VMin;
    const float VMax;
public:
    Variables(const char* name, void* variables, const VariablesT t, const float min, const float max, const bool separate = false):
        MenuEvents(name, MenuVariables, separate), mVariables(variables), mT(t), VMin(min), VMax(max){Refresh();};
    ~Variables(){};

    float GetmV(){return mV;};

    virtual void W(){mV *= 10;};
    virtual void S(){mV *= 0.1;if(mT > Float){if(mV < 1)mV = 1;}};
    virtual void A(){add(-mV);};
    virtual void D(){add(mV);};

    virtual void LongPress(){MenuEvents* E = ParentMenu();if(E != nullptr)mMenuXX = ParentMenu();};

    void Minax(){
        if((*(float*)mVariables) > VMax)*(float*)mVariables = VMax;
        if((*(float*)mVariables) < VMin)*(float*)mVariables = VMin;
    }

    std::string removeTrailingZeros(const std::string& input) {
        std::string result = input;
        size_t decimalPos = input.find('.'); // 找到小数点的位置
        if (decimalPos != std::string::npos) {
            size_t lastNonZero = input.find_last_not_of('0'); // 找到最后一个非零字符的位置
            if (lastNonZero != std::string::npos && lastNonZero >= decimalPos) {
                size_t trailingZeros = input.size() - 1 - lastNonZero; // 计算连续零的数量
                if (trailingZeros > 1) {
                    result.erase(lastNonZero + 1, trailingZeros); // 移除连续零
                }
            }
        }
        return result;
    }

    

    virtual void Refresh(){
        std::string s;
        switch (mT)
        {
        case Bool:s = (*(bool*)mVariables ? "1" : "0");break;
        case Int:s = std::to_string(*(int*)mVariables);break;
        case uInt:s = std::to_string(*(unsigned int*)mVariables);break;
        case Float:s = removeTrailingZeros(std::to_string(*(float*)mVariables));break;
        case Double:s = removeTrailingZeros(std::to_string(*(double*)mVariables));break;
        case Char:s = std::to_string(*(int*)mVariables);break;
        case uChar:s = std::to_string(*(int*)mVariables);break;
        case Enum:s = std::to_string(*(int*)mVariables);break;
        default:
            break;
        }
        str = Name() + std::string(" : ") + s;
    }

    virtual const char* Show(){ Refresh();return str.c_str();};

    void add(float V){
        switch (mT)
        {
        case Bool:*(bool*)mVariables = (*(bool*)mVariables ? false : true);break;
        case Int:*(int*)mVariables += V;Minax();break;
        case uInt:*(unsigned int*)mVariables += V;Minax();break;
        case Float:*(float*)mVariables += V;Minax();break;
        case Double:*(double*)mVariables += V;Minax();break;
        case Char:*(char*)mVariables += V;Minax();break;
        case uChar:*(unsigned char*)mVariables += V;Minax();break;
        case Enum:*(int*)mVariables += V;Minax();break;
        default:
            break;
        }
        Refresh();
    }
};

class Submenu : public MenuEvents
{
private:
    unsigned int mID = 0;
    MenuEvents** mEventsS;
    const unsigned int mMax;
public:
    Submenu(const char* name, MenuEvents** EventsS, const unsigned int Max):MenuEvents(name, MenuSubmenu), mEventsS(EventsS), mMax(Max){};
    ~Submenu();

    virtual void W(){--mID;if(mID >= mMax)mID = mMax - 1;};
    virtual void S(){++mID;if(mID >= mMax)mID = 0;};
    virtual void A(){MenuEvents* E = ParentMenu();if(E != nullptr)mMenuXX = ParentMenu();};
    virtual void D(){
        if(mEventsS[mID]->GetExecuteBool()){
            mEventsS[mID]->D();
            return;
        }
        mEventsS[mID]->SetParentMenu(this);
        mMenuXX = mEventsS[mID];
    };

    unsigned int Max(){return mMax;};
    unsigned int ID(){return mID;};
    MenuEvents* GetMenuEvents(unsigned int I){return mEventsS[I];};
};

typedef void (*_Function)(void*, void*);
class Function : public MenuEvents
{
private:
    _Function mF;
    void* mData;
    void* mClass;
public:
    Function(const char* name, _Function F, void* Class, void* Data):MenuEvents(name, MenuFunction, true), mF(F), mData(Data), mClass(Class){};
    ~Function(){};

    virtual void A(){MenuEvents* E = ParentMenu();if(E != nullptr)mMenuXX = ParentMenu();};
    virtual void D(){mF(mClass, mData);};
};

#endif
