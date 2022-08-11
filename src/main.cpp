extern "C" void app_main();

#include "NimBLEDevice.h"
#include <Esp32_BLE_HapticDial.hpp>

#define LOG_TAG "MAIN"
BleHapticDial dial;

bool clicked = false;

void app_main()
{
    INFO("Starting BLE work!");
    dial.begin();
    INFO("Setup done");
}