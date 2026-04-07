/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_control.h"
#include "camera_spi.h"
#include "ft5336.h"
#include "lcd_ui.h"
#include "angle_compute.h"
#include <string.h>
#include "lcd_display.h"

extern uint16_t* camBuffer_A;
extern uint16_t* camBuffer_B;
extern uint16_t* stable_camBuffer; // The safe pointer for the UI
extern uint16_t* debug_camBuffer;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osMessageQueueId_t motorCmdQueueHandle;
extern SPI_HandleTypeDef hspi2;

osSemaphoreId_t spiDmaSemaphore;

MotorAngles_t latest_vision_data = {0};

uint16_t internal_cam_buffer[76800];

osThreadId_t visionTaskHandle;
const osThreadAttr_t visionTask_attributes = {
  .name = "visionTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

osThreadId_t uiTaskHandle;
const osThreadAttr_t uiTask_attributes = {
  .name = "uiTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 4096 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartVisionTask(void *argument);
void StartMotorTask(void *argument);
void StartUITask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_HOST_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
	spiDmaSemaphore = osSemaphoreNew(1, 0, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	// Create a queue that can hold 10 motor commands
	  motorCmdQueueHandle = osMessageQueueNew(10, sizeof(MotorAngles_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	  visionTaskHandle = osThreadNew(StartVisionTask, NULL, &visionTask_attributes);
	  motorTaskHandle = osThreadNew(StartMotorTask, NULL, &motorTask_attributes);
	  uiTaskHandle = osThreadNew(StartUITask, NULL, &uiTask_attributes);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

//void StartVisionTask(void *argument) {
//    uint8_t out_spi_val = 0;
//    LCD_FillScreen(COLOR_BLACK);
//    UI_DrawStringCentered(0, 100, 480, 20, "INITIALIZING CAMERA...", COLOR_WHITE, COLOR_BLACK, 2);
//
//    if (Camera_Init(&out_spi_val) != 0) {
//        LCD_FillScreen(COLOR_RED);
//        UI_DrawStringCentered(0, 120, 480, 20, "CAMERA INIT FAILED", COLOR_WHITE, COLOR_RED, 2);
//        for(;;) { osDelay(1000); }
//    }
//
//    // Structure to hold our CV computation output
//    MotorAngles_t vision_angles = {0};
//
//    // 1. Set up our local alternating pointers
//	uint16_t* dma_write_buffer = camBuffer_A;
//	uint16_t* cpu_read_buffer = camBuffer_B;
//
//	// ==========================================================
//	// BOOTSTRAP: We must capture one initial frame before the loop
//	// so the CPU has something to process on the very first iteration.
//	// ==========================================================
//	Camera_WriteReg(0x04, 0x01);
//	Camera_WriteReg(0x04, 0x02);
//	while ((Camera_ReadReg(0x41) & 0x08) == 0) { osDelay(1); }
//
//	Camera_Start_DMA_Capture(cpu_read_buffer);
//	osSemaphoreAcquire(spiDmaSemaphore, osWaitForever); // Wait for Bootstrap to finish
//
//	// ==========================================================
//	// THE PARALLEL PIPELINE
//	// ==========================================================
//	for(;;) {
//		// 1. TRIGGER THE NEXT FRAME (Runs entirely in the background)
//		Camera_WriteReg(0x04, 0x01);
//		Camera_WriteReg(0x04, 0x02);
//		while ((Camera_ReadReg(0x41) & 0x08) == 0) { osDelay(1); }
//
//		Camera_Start_DMA_Capture(dma_write_buffer); // Returns instantly!
//
//		// 2. PROCESS CURRENT FRAME (CPU runs math WHILE DMA downloads)
//		// Give the UI the stable image that finished downloading last loop
//		stable_camBuffer = cpu_read_buffer;
//		SystemMode_t current_mode = UI_Get_Requested_Mode();
//
//		// Run heavy math. The DMA cannot corrupt this buffer!
//		Compute_Motor_Angles(current_mode, cpu_read_buffer, debug_camBuffer, &vision_angles);
//		latest_vision_data = vision_angles;
//
//		if (vision_angles.is_valid &&
//		   (current_mode == MODE_TRACKING || current_mode == MODE_AUTO || current_mode == MODE_DEBUG)) {
//			osMessageQueuePut(motorCmdQueueHandle, &vision_angles, 0, 0);
//		}
//
//
//		// 3. SYNC POINT (Wait for the background DMA transfer to finish)
//		// If your math took 90ms and the DMA only took 10ms, this semaphore
//		// will already be available and the task will fly past this line instantly!
//		osSemaphoreAcquire(spiDmaSemaphore, osWaitForever);
//
//		 4. SWAP BUFFERS
//		uint16_t* temp = cpu_read_buffer;
//		cpu_read_buffer = dma_write_buffer;
//		dma_write_buffer = temp;
//
//		osDelay(30);
//
//		// No osDelay() needed here. The semaphore perfectly syncs the loop to the camera speed.
//	}
//}

void StartVisionTask(void *argument) {
    uint8_t out_spi_val = 0;
    LCD_FillScreen(COLOR_BLACK);
    UI_DrawStringCentered(0, 100, 480, 20, "INITIALIZING CAMERA...", COLOR_WHITE, COLOR_BLACK, 2);

    if (Camera_Init(&out_spi_val) != 0) {
        LCD_FillScreen(COLOR_RED);
        UI_DrawStringCentered(0, 120, 480, 20, "CAMERA INIT FAILED", COLOR_WHITE, COLOR_RED, 2);
        for(;;) { osDelay(1000); }
    }

    MotorAngles_t vision_angles = {0};

    // ==========================================================
    // THE SINGLE BUFFER PIPELINE (Internal RAM Test)
    // ==========================================================
    for(;;) {
        // 1. TRIGGER THE NEXT FRAME
        Camera_WriteReg(0x04, 0x01);
        Camera_WriteReg(0x04, 0x02);
        while ((Camera_ReadReg(0x41) & 0x08) == 0) { osDelay(1); }

        // 2. DOWNLOAD DIRECTLY TO INTERNAL RAM
        Camera_Start_DMA_Capture(internal_cam_buffer);

        // 3. WAIT FOR DOWNLOAD TO FINISH (The CPU sleeps here)
        osSemaphoreAcquire(spiDmaSemaphore, osWaitForever);

        // 4. PROCESS THE COMPLETED FRAME
        // Point the UI to this internal buffer so you can still see it
        stable_camBuffer = internal_cam_buffer;
        SystemMode_t current_mode = UI_Get_Requested_Mode();

        // Run heavy math
        Compute_Motor_Angles(current_mode, internal_cam_buffer, debug_camBuffer, &vision_angles);
        latest_vision_data = vision_angles;

        // 5. SEND MOTOR COMMANDS
        if (vision_angles.is_valid &&
           (current_mode == MODE_TRACKING || current_mode == MODE_AUTO || current_mode == MODE_DEBUG)) {
            osMessageQueuePut(motorCmdQueueHandle, &vision_angles, 0, 0);
        }

        // Add a tiny delay to let the UI Task breathe since we lost parallel processing
        osDelay(10);
    }
}

void StartMotorTask(void *argument) {
    MotorAngles_t incoming_cmd;
    Motors_Start();

    for(;;) {
        // Wait up to 10ms for a new target command.
        // If a command arrives, it instantly updates the target.
        // If 10ms passes with no new command, it returns an error and moves to the next line.
        if (osMessageQueueGet(motorCmdQueueHandle, &incoming_cmd, NULL, 10) == osOK) {
            Motors_Set_Target(&incoming_cmd);
        }

        // Execute the 100Hz Slew Rate math
        Motors_Tick();
    }
}

void StartUITask(void *argument) {
    uint16_t touch_x = 0, touch_y = 0;
    UI_Init();

    for(;;) {
        uint8_t is_touching = FT5336_ReadTouch(&touch_x, &touch_y);
        UI_Process_Touch(touch_x, touch_y, is_touching);

        UI_Render_Screen(UI_Get_Requested_Mode(), &latest_vision_data, touch_x, touch_y, is_touching);

        osDelay(10); // ~30 FPS refresh rate
    }
}
/* USER CODE END Application */

