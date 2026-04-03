#ifndef CAMERA_SPI_H
#define CAMERA_SPI_H

#include "stm32f7xx_hal.h"

#define ARDUCHIP_TEST1     0x00
#define ARDUCHIP_FIFO      0x04
#define FIFO_CLEAR_MASK    0x01
#define FIFO_START_MASK    0x02
#define ARDUCHIP_TRIG      0x41
#define CAP_DONE_MASK      0x08
#define FIFO_SIZE1         0x42
#define FIFO_SIZE2         0x43
#define FIFO_SIZE3         0x44
#define BURST_FIFO_READ    0x3C

// Register Structure for OV2640
struct sensor_reg {
    uint8_t reg;
    uint8_t val;
};

uint8_t Camera_WriteReg(uint8_t addr, uint8_t data);
uint8_t Camera_ReadReg(uint8_t addr);
uint8_t Camera_Init(uint8_t* out_spi_val);
void Camera_Start_DMA_Capture(uint16_t* dest_buffer);

#endif /* CAMERA_SPI_H */
