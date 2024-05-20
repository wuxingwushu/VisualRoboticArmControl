#ifndef _RKNN_YOLOV5_DEMO_GPIO_H_
#define _RKNN_YOLOV5_DEMO_GPIO_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

class GPIO
{
private:
    unsigned int ID;
    bool OutIn;
    FILE *GPIO_file = NULL;
    //char cat_command[100];
public:
    GPIO(unsigned int gpioID, bool OutFoIn);
    ~GPIO();

    inline void SetValue(bool V);
    inline bool GetValue();
};

GPIO::GPIO(unsigned int gpioID, bool OutFoIn):ID(gpioID), OutIn(OutFoIn)
{
    printf("GPIO : %d  Model : %s\n", ID, OutIn ? "out" : "in");
    //导出引脚
    FILE *export_file = fopen("/sys/class/gpio/export", "w");
    if (export_file == NULL) {
        perror("Failed to open GPIO export file");
        return;
    }
    fprintf(export_file, "%d", ID);
    fclose(export_file);

    //配置GPIO方向
    char direction_path[50];
    snprintf(direction_path, sizeof(direction_path), "/sys/class/gpio/gpio%d/direction", ID);
    FILE *direction_file = fopen(direction_path, "w");
    if (direction_file == NULL) {
        perror("Failed to open GPIO direction file");
        return;
    }
    fprintf(direction_file, OutIn ? "out" : "in");
    fclose(direction_file);

    //控制引脚
    char value_path[50];
    snprintf(value_path, sizeof(value_path), "/sys/class/gpio/gpio%d/value", ID);
    //snprintf(cat_command, sizeof(cat_command), "cat %s", value_path);
    GPIO_file = fopen(value_path, OutIn ? "w" : "r");
    if (GPIO_file == NULL) {
        perror("Failed to open GPIO value file");
        return;
    }
}

GPIO::~GPIO()
{
    fclose(GPIO_file);

    //取消导出引脚
    FILE *unexport_file = fopen("/sys/class/gpio/unexport", "w");
    if (unexport_file == NULL) {
        perror("Failed to open GPIO unexport file");
        return ;
    }
    fprintf(unexport_file, "%d", ID);
    fclose(unexport_file);
}

inline void GPIO::SetValue(bool V){
    /*if(!OutIn) {
        perror("Error GPIO Model : in\n");
        return;
    }*/
    fprintf(GPIO_file, V ? "1" : "0");
    fflush(GPIO_file);//刷新缓冲区  强制将缓冲区中的数据写入文件
    //system(cat_command);
}

inline bool GPIO::GetValue(){
    /*if(OutIn) {
        perror("Error GPIO Model : out\n");
        return 0;
    }*/
    fseek(GPIO_file, 0, SEEK_SET); // 将文件指针移动到文件开头
    //system(cat_command);
    return (fgetc(GPIO_file) - '0'); // 读取一个字符
}

#endif
