// ei_inference.cpp
// C++ wrapper around Edge Impulse SDK — exposes plain C API for angle_compute.c

#include "ei_inference.h"

// Suppress the EI object detection count warning (we use bounding_boxes_count instead)
#define SILENCE_EI_CLASSFIER_OBJECT_DETECTION_COUNT_WARNING

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

// ============================================================
// Arena: the scratch RAM the model uses during inference.
// 289,580 bytes (~283KB) — placed in external SDRAM.
// ============================================================
static uint8_t ei_arena[EI_CLASSIFIER_TFLITE_LARGEST_ARENA_SIZE] \
    __attribute__((section(".sdram")));

// ============================================================
// Input buffer: 96x96 RGB floats, normalized [0.0, 1.0]
// Also placed in SDRAM to avoid eating internal SRAM.
// ============================================================
static float ei_input_buf[EI_CLASSIFIER_INPUT_WIDTH *
                          EI_CLASSIFIER_INPUT_HEIGHT * 3]
    __attribute__((section(".sdram")));

// ============================================================
// EI SDK callback: called by run_classifier() to fetch pixels
// ============================================================
static int ei_get_data(size_t offset, size_t length, float *out_ptr) {
    for (size_t i = 0; i < length; i++) {
        out_ptr[i] = ei_input_buf[offset + i];
    }
    return 0;
}

void EI_Init(void) {
    // Nothing needed for EON compiled models — no runtime init required
}

EI_Detection_t EI_Run(uint16_t* rgb565_buffer) {
    EI_Detection_t result = {0};

    // ----------------------------------------------------------
    // 1. Downsample 320x240 RGB565 → 96x96 RGB float [0.0, 1.0]
    //    Uses nearest-neighbor sampling
    // ----------------------------------------------------------
    const uint16_t SRC_W = 320;
    const uint16_t SRC_H = 240;
    const uint16_t DST_W = EI_CLASSIFIER_INPUT_WIDTH;   // 96
    const uint16_t DST_H = EI_CLASSIFIER_INPUT_HEIGHT;  // 96

    for (uint16_t row = 0; row < DST_H; row++) {
        for (uint16_t col = 0; col < DST_W; col++) {
            uint16_t src_row = (row * SRC_H) / DST_H;
            uint16_t src_col = (col * SRC_W) / DST_W;
            uint16_t pixel   = rgb565_buffer[src_row * SRC_W + src_col];

            // Extract RGB565 channels
            uint8_t r5 = (pixel >> 11) & 0x1F;
            uint8_t g6 = (pixel >> 5)  & 0x3F;
            uint8_t b5 = (pixel)       & 0x1F;

            // Scale to [0.0, 1.0]
            size_t idx = (row * DST_W + col) * 3;
            ei_input_buf[idx + 0] = (float)(r5 << 3) / 255.0f;
            ei_input_buf[idx + 1] = (float)(g6 << 2) / 255.0f;
            ei_input_buf[idx + 2] = (float)(b5 << 3) / 255.0f;
        }
    }

    // ----------------------------------------------------------
    // 2. Run FOMO inference
    // ----------------------------------------------------------
    signal_t signal;
    signal.total_length = DST_W * DST_H * 3;
    signal.get_data     = &ei_get_data;

    ei_impulse_result_t ei_result = {0};
    EI_IMPULSE_ERROR err = run_classifier(&signal, &ei_result, false);

    if (err != EI_IMPULSE_OK) {
        return result;  // detected = 0
    }

    // ----------------------------------------------------------
    // 3. Find highest-confidence hand detection
    // ----------------------------------------------------------
    float    best_conf = 0.0f;
    uint32_t best_idx  = 0;
    bool     found     = false;

    for (uint32_t i = 0; i < ei_result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = ei_result.bounding_boxes[i];
        if (bb.value >= EI_CLASSIFIER_OBJECT_DETECTION_THRESHOLD &&
            bb.value > best_conf) {
            best_conf = bb.value;
            best_idx  = i;
            found     = true;
        }
    }

    if (!found) {
        return result;  // detected = 0
    }

    // ----------------------------------------------------------
    // 4. Scale bounding box from 96x96 back to 320x240
    // ----------------------------------------------------------
    ei_impulse_result_bounding_box_t best = ei_result.bounding_boxes[best_idx];

    result.detected    = 1;
    result.confidence  = best.value;
    result.box_x       = (uint16_t)((best.x      * SRC_W) / DST_W);
    result.box_y       = (uint16_t)((best.y      * SRC_H) / DST_H);
    result.box_w       = (uint16_t)((best.width  * SRC_W) / DST_W);
    result.box_h       = (uint16_t)((best.height * SRC_H) / DST_H);

    return result;
}
