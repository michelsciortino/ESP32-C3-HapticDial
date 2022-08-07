#pragma once
#include "sdkconfig.h"

#if defined(CONFIG_BT_ENABLED)

#include <NimBLEDevice.h>

#define MS_VID 0x045E
#define MS_PID 0x0905
#define MS_VER 0x01

typedef union
{
    // report: 1 button, 15-bit rotation
    uint8_t whole8[0];
    uint16_t whole16[0];
    struct
    {
        uint16_t button : 1;
        uint16_t rotation : 15;
    };
} HID_DialReport_Data_t;

class BleHapticDial : public NimBLEServerCallbacks, public NimBLECharacteristicCallbacks
{
private:
    std::string _device_name, _device_manufacturer;
    uint16_t _vid, _pid, _version;
    uint8_t _battery_level; // 0-255
    bool _button_status = false;
    bool _connected = false;
    uint16_t _dial_degree = 0;
    uint32_t _debounce_ms; // ms
    uint32_t _ble_adv_time = 5000;
    uint32_t _ble_adv_sleep_s = 20;
    NimBLEServer *_server;
    NimBLEHIDDevice *_hid_device;
    NimBLECharacteristic *_radial_controller;
    NimBLECharacteristic *_haptic_feedback;

public:
    inline BleHapticDial(std::string deviceName = "ESP32 HapticDial",
                         std::string deviceManufacturer = "Espressif",
                         uint16_t vid = MS_VID,
                         uint16_t pid = MS_PID,
                         uint16_t version = MS_VER,
                         uint8_t batteryLevel = 255,
                         uint32_t debounceTime = 10);
    inline void begin(void);
    inline void end(void);
    inline void click(void);
    inline void press(void);
    inline void release(void);
    inline void rotate(int16_t degree);
    inline bool isPressed(void);
    inline int16_t getDialDegrees(void);
    inline bool isConnected(void);
    inline void setBatteryLevel(uint8_t level);

protected:
    inline void button(bool pressed);
    virtual void onStarted(void){};
    virtual void onConnect(void);
    virtual void onDisconnect(void);
    virtual void sendReport(void);
    inline void reset(void);
};

#endif // CONFIG_BT_ENABLED