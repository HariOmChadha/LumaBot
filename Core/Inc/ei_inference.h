#ifndef EI_INFERENCE_H
#define EI_INFERENCE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t box_x;
    uint16_t box_y;
    uint16_t box_w;
    uint16_t box_h;
    float    confidence;
    uint8_t  detected;   // 1 if a hand was found above threshold
} EI_Detection_t;

// Call this once at startup
void EI_Init(void);

// Pass a 320x240 RGB565 buffer, get back the best hand detection
EI_Detection_t EI_Run(uint16_t* rgb565_buffer);

#ifdef __cplusplus
}
#endif

#endif /* EI_INFERENCE_H */
