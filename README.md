# rp2040-freeRTOS-smp

FreeRTOS with SMP template for RP2040 microcontroller family.

SMP support in FreeRTOS Kernel enables one instance of the FreeRTOS kernel to schedule tasks across multiple identical processor cores. The core architectures must be identical and share the same memory.

Example applications are in boards directory. For now, only examples for [https://www.waveshare.com/rp2040-lcd-1.28.htm](Waveshare LCD 1.28) are availabe

# Build

Dev environment on Ubuntu
```sh
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
```

Update all submodule

```sh
git submodule update --init --recursive
```

Build the project

```sh
mkdir build
cd build
export PICO_SDK_PATH=../pico-sdk
export FREERTOS_KERNEL_PATH=../FreeRTOS-Kernel
cmake ..
make
```