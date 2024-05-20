#ifndef _RKNN_YOLOV5_DEMO_SPI_H_
#define _RKNN_YOLOV5_DEMO_SPI_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

#define SPI_DEVICE_PATH "/dev/spidev0.0"

class SPI
{
private:
    int spi_file;
    uint8_t mode;
    uint8_t bits_per_word;
    uint32_t speed_hz;

public:
    SPI(uint8_t mode = SPI_MODE_0, uint8_t bits_per_word = 8, uint32_t speed_hz = 49000000);
    ~SPI();

    bool SPIwrite(uint8_t* data, size_t size);
};

SPI::SPI(uint8_t mode, uint8_t bits_per_word, uint32_t speed_hz)
    : mode(mode), bits_per_word(bits_per_word), speed_hz(speed_hz)
{
    if ((spi_file = open(SPI_DEVICE_PATH, O_RDWR)) < 0) {
        perror("Failed to open SPI device");
        return;
    }

    if (ioctl(spi_file, SPI_IOC_WR_MODE, &mode) == -1) {
        perror("Failed to set SPI mode");
        close(spi_file);
        return;
    }

    if (ioctl(spi_file, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) == -1) {
        perror("Failed to set SPI bits per word");
        close(spi_file);
        return;
    }

    if (ioctl(spi_file, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) == -1) {
        perror("Failed to set SPI speed");
        close(spi_file);
        return;
    }
}

SPI::~SPI()
{
    close(spi_file);
}

bool SPI::SPIwrite(uint8_t* data, size_t size)
{
    struct spi_ioc_transfer transfer = {
        .tx_buf = (unsigned long)data,
        .rx_buf = NULL,
        .len = size,
        .speed_hz = speed_hz,
        .delay_usecs = 0,
        .bits_per_word = bits_per_word
    };

    if (ioctl(spi_file, SPI_IOC_MESSAGE(1), &transfer) == -1) {
        perror("Failed to perform SPI transfer");
        return false;
    }

    return true;
}


#endif
