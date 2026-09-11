# Third-party notices

This project vendors the following components from STM32CubeF4 V1.28.3:

- STM32F4 CMSIS device support and Arm CMSIS headers: see `Drivers/CMSIS/LICENSE.txt` and `Drivers/CMSIS/Device/ST/STM32F4xx/LICENSE.txt`.
- STM32F4 HAL: see `Drivers/STM32F4xx_HAL_Driver/LICENSE.txt`.
- LAN8742 component driver, used with the board's LAN8720A-compatible register interface: see `Drivers/BSP/Components/lan8742/LICENSE.txt`.
- The applicable STM32CubeF4 package terms are preserved in `licenses/STM32CubeF4_Package_license.md`.
- lwIP: see `Middlewares/Third_Party/LwIP/COPYING`.

The RobStride, eRob, force-sensor, application, and UDP source files in `App/` are clean protocol implementations written for this project. No source file from the RobStride reference repositories is included. Links to the protocol references are recorded in `docs/DRIVERS.md`.
