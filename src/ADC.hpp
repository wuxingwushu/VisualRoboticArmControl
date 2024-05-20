#ifndef _RKNN_YOLOV5_DEMO_ADC_H_
#define _RKNN_YOLOV5_DEMO_ADC_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

class ADC
{
private:
    FILE *inx_raw_file = nullptr;
    FILE *scale_file = nullptr;
public:
    ADC(unsigned int ID);
    ~ADC();

    float GetADC();
};

ADC::ADC(unsigned int ID)
{
    printf("Press Ctrl+C to quit\n");
    const char *adc_dir = "/sys/bus/iio/devices/iio:device0";
    char in_voltage1_raw_path[256];
    char in_voltage_scale_path[256];

    sprintf(in_voltage1_raw_path, "%s/in_voltage%d_raw", adc_dir, ID);
    sprintf(in_voltage_scale_path, "%s/in_voltage_scale", adc_dir);

    FILE *scale_file = fopen(in_voltage_scale_path, "r");
    FILE *inx_raw_file = fopen(in_voltage1_raw_path, "r");
}

ADC::~ADC()
{
    fclose(scale_file);
    fclose(inx_raw_file);
}

float ADC::GetADC(){
    char buffer[32];

    fseek(scale_file, 0, SEEK_SET);
    fseek(inx_raw_file, 0, SEEK_SET);

    fgets(buffer, sizeof(buffer), scale_file);
    float scale = strtof(buffer, NULL);


    fgets(buffer, sizeof(buffer), inx_raw_file);
    int in1_raw_value = atoi(buffer);

    float in1_voltage = (in1_raw_value * scale) / 1000.0;

    return in1_voltage;
}


#endif
