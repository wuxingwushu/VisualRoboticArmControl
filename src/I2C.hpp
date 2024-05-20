#ifndef _RKNN_YOLOV5_DEMO_I2C_H_
#define _RKNN_YOLOV5_DEMO_I2C_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

class I2C
{
private:
    int i2c_file;
    int address;
public:
    I2C(unsigned char ID, unsigned char Address = 0);
    ~I2C();

    bool I2Cwrite(unsigned char* data, unsigned int size);

    bool I2Cwrite(unsigned char data);
};

I2C::I2C(unsigned char ID, unsigned char Address)
{
    std::string I2C_DEVICE_PATH = "/dev/i2c-" + std::to_string(ID);

    printf("Set I2C : %s", I2C_DEVICE_PATH.c_str());

    uint8_t data[2] = {0x01,0x02};

    if ((i2c_file = open(I2C_DEVICE_PATH.c_str(), O_RDWR)) < 0) {
        perror("Failed to open I2C device");
        return;
    }
    
    ioctl(i2c_file, I2C_TENBIT, 0);
    ioctl(i2c_file, I2C_RETRIES, 5);

    if(Address != 0){
        if (ioctl(i2c_file, I2C_SLAVE, Address) < 0) {
            perror("Failed to set I2C slave address");
            return;
        }
        return;
    }

    printf("i2cdetect addr : ");
    for (int x = 0; x < 0x7f; x++)
    {
        if (ioctl(i2c_file, I2C_SLAVE, x) < 0) {
            perror("Failed to set I2C slave address");
            return;
        }
        
        if (write(i2c_file, data, 2) == 2)
        {
            address = x;
            printf("0x%x,", x);
            return;
        }
    }
}

I2C::~I2C()
{
    close(i2c_file);
}

bool I2C::I2Cwrite(unsigned char* data, unsigned int size){
    if (write(i2c_file, data, size) != size)
    {
        perror("Failed to set I2C slave address");
        return false;
    }
    return true;
}

bool I2C::I2Cwrite(unsigned char data){
    if (write(i2c_file, &data, 1) != 1)
    {
        perror("Failed to set I2C slave address");
        return false;
    }
    return true;
}


#endif
