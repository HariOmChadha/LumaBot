# LumaBot: Hardware-Accelerated RTOS Robotic Lamp

**LumaBot** is an advanced, 5-DoF autonomous robotic lamp designed to actively eliminate shadows and assist users in real-time. Built on the **STM32F746G-DISCO** board, it features a true parallel-processing computer vision pipeline, a hardware-accelerated touchscreen UI, and decoupled FreeRTOS motor control to provide smooth, intelligent lighting assistance.

## 🌟 What LumaBot Does

LumaBot acts as an intelligent desktop companion. Using an onboard camera and computer vision, it understands its environment and physically moves its light source to optimize visibility for the user. Whether you need autonomous shadow elimination while writing, or a lamp that remembers your exact preferred lighting angles, LumaBot adapts to your workflow.

### 🎮 Operating Modes & Capabilities

The robotic lamp is controlled via a custom-built, hardware-accelerated touchscreen interface, offering four distinct modes:

* **🤖 AUTO MODE: Active Shadow Elimination**
    * **The Goal:** Provides perfect, unobstructed lighting.
    * **How it works:** LumaBot uses its camera and computer vision to analyze the workspace. If it detects an object or a hand casting a shadow over the work area, it autonomously calculates the optimal counter-angle and repositions its 5-axis arm to shine light under or around the obstruction, effectively "erasing" the shadow.
* **🖐️ TRACK MODE: Hand Tracking & Motion Recording**
    * **The Goal:** Dynamic, gesture-based lighting and automation.
    * **How it works:** LumaBot actively tracks your hand movements across the desk and follows you, ensuring the light is always exactly where you are working.
    * **Trajectory Recording:** Features a built-in state machine allowing you to hit "Record," perform a sweeping motion with your hand, and have LumaBot memorize that exact physical trajectory. You can then "Replay" the movement or save it to a preset.
* **🎛️ MANUAL MODE: Precision Control & Presets**
    * **The Goal:** Total granular control over your lighting setup.
    * **How it works:** Provides live-updating touchscreen sliders to manually adjust the angle of all 5 joints. 
    * **Preset Memory:** Found the perfect lighting angle for reading or soldering? Save the current physical pose into one of 3 memory slots. Tap a preset later, and LumaBot will instantly snap back to that exact configuration.
* **⚙️ DEBUG MODE: ML & Algorithm Tuning**
    * **The Goal:** Under-the-hood diagnostics.
    * **How it works:** Displays a real-time, side-by-side split screen showing the raw camera feed next to the Computer Vision output. Used to calibrate the bounding boxes, color thresholds, and tracking latency directly on the device.

## 🚀 Technical Highlights

* **True Parallel Vision Pipeline:** Utilizes SPI DMA with hardware interrupt chaining to download 153KB camera frames in the background. A "Ping-Pong" double-buffer architecture ensures the CPU runs Computer Vision (CV) math on Frame 1 while the DMA simultaneously downloads Frame 2, preventing memory corruption and maximizing FPS.
* **Tear-Free Hardware Graphics:** The custom UI engine uses the STM32's **Chrom-ART (DMA2D)** accelerator to offload pixel rendering from the CPU. It employs Display Double-Buffering and an LTDC VSYNC hardware line-interrupt to guarantee 60Hz, tear-free graphical updates with 0% CPU polling.
* **Decoupled RTOS Architecture:** Built on FreeRTOS with strictly isolated tasks (Vision, UI, Motor). A queue-based message dispatcher ensures that UI rendering and heavy CV matrix math never block physical PWM hardware generation.

## 🛠 Hardware Architecture

* **Microcontroller:** STM32F746G-Discovery (ARM Cortex-M7 @ 216 MHz)
* **Camera:** Arducam OV2640 (SPI2 @ 6.5MBit/s, I2C for config)
* **Actuators:** 5x Standard Hobby Servos (PWM controlled)
* **Display:** Onboard 4.3" 480x272 TFT LCD with capacitive touch (FT5336)

### ⚠️ Critical Power Notice
Standard USB ports cannot provide the transient current spikes required by 5 servo motors. 
**DO NOT pull motor power through the STM32 5V/3V3 pins.** 1. Power the motors using an external high-current power supply (e.g., bench supply or LiPo battery via buck converter).
2. **Common Ground:** You MUST tie the ground of the external power supply to a `GND` pin on the STM32 board, otherwise the PWM logic signals will float and the servos will violently jitter.

### Pin Mapping
| Component | STM32 Pin | Function | Arduino Header |
| :--- | :--- | :--- |
| **Servo 1 (Base)** | `PB4` | `TIM3_CH1` (PWM 50Hz) |  `D3` | 
| **Servo 2 (Shoulder)** | `PI0` | `TIM5_CH4` (PWM 50Hz) | `D10` |
| **Servo 3 (Elbow)** | `PH6` | `TIM12_CH1` (PWM 50Hz) | `D6` |
| **Servo 4 (Wrist Pitch)** | `PA15` | `TIM2_CH1` (PWM 50Hz) | `D9` |
| **Servo 5 (Wrist Roll)** | `PA8` | `TIM1_CH1` (PWM 50Hz) | `D5` |
| **Camera CS** | `PG7` | SPI Chip Select | `D4` |
| **Camera SPI** | `PI1, PB14, PB15` | `SPI2` (SCK, MISO, MOSI) | `D13, D12, D11` |
| **I2C** | `PB8, PB9` | `I2C2` (SCL, SDA) | `D15, D14` |

## 🧠 Software Architecture

The software is divided into three primary FreeRTOS tasks to ensure deterministic hardware behavior:

1.  **`StartVisionTask` (High Priority):** Manages the OV2640 camera state machine and runs the blocking Computer Vision math pipeline. It communicates with the DMA hardware via semaphores and dispatches resulting angles to the Motor Queue.
2.  **`StartUITask` (Medium Priority):** Sleeps dynamically using an LTDC VSYNC hardware interrupt. Upon waking, it reads the FT5336 touch controller, calculates state changes, commands DMA2D UI draws to the hidden Back Buffer, and swaps the LCD hardware pointer.
3.  **`StartMotorTask` (Hardware Bottleneck):** The only task allowed to write to the physical hardware timer registers. It sits blocked at `motorCmdQueueHandle`. When a struct arrives, it validates the data, checks for changes (to save CPU cycles), and scales the 0-180° request to a precise 1000us-2000us PWM pulse width.

## ⚙️ Getting Started

### Prerequisites
* STM32CubeIDE (v1.14.0 or higher recommended)
* STM32CubeProgrammer

### Building the Project
1.  Clone the repository.
2.  Open the project folder in STM32CubeIDE.
3.  Open the `.ioc` file and click **Generate Code** to ensure all HAL libraries are correctly linked for your environment.
4.  Build the project (`Project -> Build All`).
5.  Connect your STM32F7-DISCO via the `ST-LINK` USB port and click **Run**.

## 🔮 Future Improvements
* Migrate internal CV math to hardware-accelerated DSP instructions (CMSIS-DSP) to improve shadow-detection latency.
* Add a Deadband filter to the motor dispatcher to eliminate micro-hunting during Track Mode.
* Integrate depth-sensing algorithms for more precise Z-axis shadow manipulation.

---
*Developed for advanced embedded systems research, computer vision, and autonomous robotics control during MIE438*