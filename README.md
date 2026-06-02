# FlowBox: Real-Time Cellular Automata Embedded Toy

FlowBox is an interactive, bare-metal embedded device designed to simulate a physical container filled with granular particles or fluid (available materials are sand, water, and lava). Powered by an ESP32-S3 and written in pure C using the ESP-IDF framework, the system captures real-time 3-axis accelerometer data to dynamically orient a particle physics grid on an IPS display.

## Hardware Subsystem Architecture
The project utilizes an integrated development board configuration with the following peripheral matrix:
* **MCU:** ESP32-S3R8 (Dual-Core Xtensa 32-bit LX7 processor with 8MB packaged PSRAM).
* **Display:** 3.16" IPS LCD (320x820px) driven by an ST7701 controller over a high-speed parallel RGB565 bus.
* **Telemetry:** QMI8658 Inertial Measurement Unit (IMU) and an optional TSL2561 light sensor mapped onto an active I2C master bus to capture motion and ambient light values.
* **Storage:** Local MicroSD card slot wired over an SDMMC host layer.
* **I/O Expander:** An I2C-driven `esp_io_expander` handles static, low-speed panel control and configuration tasks (such as display resets and backlight adjustments) to maximize native pin availability for the parallel video bus.

## Software Design & Concurrency Model

The firmware is engineered on top of **ESP-IDF v4.6** and uses a dual-core **FreeRTOS** task infrastructure to enforce a strict separation of concerns:

### Core Allocation Matrix
* **Core 1 (Physics Core):** Runs the compute-heavy `simulation_task`. It processes accelerometer data, advances the granular engine states, and renders frames asynchronously. Pinning this task guarantees a stable frame loop immune to file-system delays.
* **Core 0 (System Core):** Executes the primary system infrastructure thread (`app_main`), processing runtime diagnostics, theme calculations, and background peripheral configurations, while interactive input events (`button_task`) are managed concurrently via the FreeRTOS scheduler.

### State Synchronization & Storage Safety
Memory-level race conditions over the shared SDMMC media interface are eliminated using a state-machine topology controlled by a volatile `g_system_state` flag:
1. When a user executes a **Long Hold (≥5s)** on the control button, the state changes to `SYS_STATE_USB_STORAGE`.
2. The core simulation loop on Core 1 intercepts this change and enters a low-overhead cooperative yield state (`vTaskDelay`).
3. With the bus safely cleared of physics operations, Core 0 unmounts the local virtual file system partition (`esp_vfs_fat_sdcard_unmount`) and initialises the native Espressif TinyUSB MSC stack.
4. The device safely re-enumerates on the host PC as a standard plug-and-play mass storage thumb drive, enabling zero-recompilation map editing.


## Physics Optimization Strategies

Squeezing real-time fluid and granular physics out of a microcontroller required implementing specialized low-level optimization patterns:

1. **In-Place Double Buffering:** The system structure instantiates two contiguous 1D linear buffers (`grid_buffer_0` and `grid_buffer_1`) allocated in memory. The engine reads cell states via a `current_grid` pointer and records changes into `next_grid`. At frame termination, the pointer addresses are swapped instantaneously, preventing memory fragmentation and expensive `memcpy` overhead across large arrays.
2. **Directional Traversal Sweeps:** Standard loops produce a visual "boiling" glitch where falling items jump multiple positions in a single frame. The execution engine dynamically flips the nested loop boundaries and steps (`1` or `-1`) to align precisely with the active sign of the gravity vector ($g$).
3. **Implicit Border Enforcements:** To minimize conditional branch mispredictions, all map files must define a mandatory static wall perimeter. This structural layout guarantee ensures nested coordinate logic never has to perform out-of-bounds neighbor checks along grid margins.
