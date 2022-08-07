#include "Esp32_BLE_HapticDial.hpp"
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include <NimBLEHIDDevice.h>
#include "HIDTypes.h"
#include <driver/adc.h>
#include "sdkconfig.h"
#include "esp_sleep.h"

#if !CONFIG_BT_NIMBLE_EXT_ADV
#error Must enable NimBLE extended advertising
#endif

#include "esp_log.h"
static const char *LOG_TAG = "HAPTIC_DIAL";

#define RADIAL_CONTROLLER_REPORT_ID 0x01
#define HAPTIC_FEEDBACK_REPORT_ID 0x02

// doc https://docs.microsoft.com/it-it/windows-hardware/design/component-guidelines/radial-controller-sample-report-descriptors
// Integrated Radial Controller TLC
static const uint8_t _HapticDialHIDReportDescriptor[] = {
    USAGE_PAGE(1), 0x01, // Generic Desktop
    USAGE(1), 0x0e,      // System Multi-Axis Controller
    COLLECTION(1), 0x01, // Application
    // Radial controller
    REPORT_ID(1), RADIAL_CONTROLLER_REPORT_ID, // Radial Controller
    USAGE_PAGE(1), 0x0d,                       // Digitizers
    USAGE(1), 0x21,                            // Puck
    COLLECTION(1), 0x00,                       // Physical
    // Press button
    USAGE_PAGE(1), 0x09,      // Buttons
    USAGE(1), 0x01,           // Button 1
    REPORT_COUNT(1), 0x01,    // 1
    REPORT_SIZE(1), 0x01,     // 1
    LOGICAL_MINIMUM(1), 0x00, // 0
    LOGICAL_MAXIMUM(1), 0x01, // 1
    HIDINPUT(1), 0x02,        // Data,Var,Abs
    // Rotating dial
    USAGE_PAGE(1), 0x01,             // Generic Desktop
    USAGE(1), 0x37,                  // Dial
    REPORT_COUNT(1), 0x01,           // 1
    REPORT_SIZE(1), 0x0f,            // 15
    UNIT_EXPONENT(1), 0x0f,          // -1
    UNIT(1), 0x14,                   // Degrees, English Rotation
    PHYSICAL_MINIMUM(2), 0xf0, 0xf1, // -3600
    PHYSICAL_MAXIMUM(1), 0x10, 0x0e, // 3600
    LOGICAL_MINIMUM(2), 0xf0, 0xf1,  // -3600
    LOGICAL_MAXIMUM(2), 0x10, 0x0e,  // 3600
    HIDINPUT(1), 0x06,               // Data,Var,Rel
    END_COLLECTION(0),               // END_COLLECTION
    // Haptic feedback
    REPORT_ID(1), HAPTIC_FEEDBACK_REPORT_ID, // Haptic Feedback
    USAGE_PAGE(1), 0x0e,                     // Haptics
    USAGE(1), 0x01,                          // Simple Haptic Controller
    COLLECTION(1), 0x02,                     // Logical
    // Waveform List
    USAGE_PAGE(1), 0x0e,             // Haptics
    USAGE(1), 0x10,                  // Waveform List
    COLLECTION(1), 0x02,             // Logical
    USAGE_PAGE(1), 0x0A,             // Ordinal
    USAGE(1), 0x03,                  // Ordinal 3
    REPORT_COUNT(1), 0x01,           // 1
    REPORT_SIZE(1), 0x08,            // 8
    LOGICAL_MINIMUM(1), 0x03,        // 3
    LOGICAL_MAXIMUM(1), 0x03,        // 3
    PHYSICAL_MINIMUM(1), 0x03, 0x10, // 0x1003
    PHYSICAL_MAXIMUM(1), 0x03, 0x10, // 0x1003
    FEATURE(1), 0x03,                // Cnst,Var,Abs
    USAGE(1), 0x04,                  // Ordinal 4
    LOGICAL_MINIMUM(1), 0x04,        // 4
    LOGICAL_MAXIMUM(1), 0x04,        // 4
    PHYSICAL_MINIMUM(1), 0x04, 0x10, // 0x1004
    PHYSICAL_MAXIMUM(1), 0x04, 0x10, // 0x1004
    FEATURE(1), 0x03,                // Cnst,Var,Abs
    END_COLLECTION(0),               // END_COLLECTION
    // Duration List
    USAGE_PAGE(1), 0x0e,            // Haptics
    USAGE(1), 0x11,                 // Duration List
    COLLECTION(1), 0x02,            // Logical
    USAGE_PAGE(1), 0x0a,            // Ordinal
    USAGE(1), 0x03,                 // Ordinal 3
    USAGE(1), 0x04,                 // Ordinal 4
    LOGICAL_MINIMUM(1), 0x00,       // 0
    LOGICAL_MAXIMUM(2), 0xff, 0x0f, // 4095
    REPORT_COUNT(1), 0x02,          // 2
    REPORT_SIZE(1), 0x08,           // 8
    FEATURE(1), 0x02,               // Data,Var,Abs
    END_COLLECTION(0),              // END_COLLECTION
    // Auto Trigger
    USAGE(1), 0x20,                 // Auto Trigger
    LOGICAL_MINIMUM(2), 0x00, 0x10, // 0x1000
    LOGICAL_MAXIMUM(2), 0x04, 0x10, // 0x1004
    REPORT_COUNT(1), 0x01,          // 1
    REPORT_SIZE(1), 0x10,           // 16
    FEATURE(1), 0x02,               // Data,Var,Abs
    // Auto Trigger Associated Control
    USAGE(1), 0x22,                             // Auto Trigger Associated Control
    LOGICAL_MINIMUM(3), 0x37, 0x00, 0x01, 0x00, // 0x00010037
    LOGICAL_MAXIMUM(3), 0x37, 0x00, 0x01, 0x00, // 0x00010037
    REPORT_COUNT(1), 0x01,                      // 1
    REPORT_SIZE(1), 0x20,                       // 32
    FEATURE(1), 0x03,                           // Cnst,Var,Abs
    // Intensivity
    USAGE(1), 0x23,           // Intensity
    LOGICAL_MINIMUM(1), 0x00, // 0
    LOGICAL_MAXIMUM(1), 0x7f, // 127
    REPORT_SIZE(1), 0x08,     // 8
    HIDOUTPUT(1), 0x02,       // Data,Var,Abs
    USAGE(1), 0x23,           // Intensity
    FEATURE(1), 0x02,         // Data,Var,Abs
    // Repeat Count
    USAGE(1), 0x24,     // Repeat Count
    HIDOUTPUT(1), 0x02, // Data,Var,Abs
    USAGE(1), 0x24,     // Repeat Count
    FEATURE(1), 0x02,   // Data,Var,Abs
    // Retrigger Period
    USAGE(1), 0x25,     // Retrigger Period
    HIDOUTPUT(1), 0x02, // Data,Var,Abs
    USAGE(1), 0x25,     // Retrigger Period
    FEATURE(1), 0x02,   // Data,Var,Abs
    // Waveform Cutoff Time Feature
    USAGE(1), 0x28,                 // Waveform Cutoff Time
    LOGICAL_MAXIMUM(2), 0xff, 0x7f, // 32,767
    REPORT_SIZE(1), 0x10,           // 16
    FEATURE(1), 0x02,               // Data,Var,Abs
    // Manual Trigger
    USAGE(1), 0x21,     // Manual Trigger
    HIDOUTPUT(1), 0x02, // Data,Var,Abs
    END_COLLECTION(0),  // END_COLLECTION
    END_COLLECTION(0)   // END_COLLECTION
};

static void delay_ms(uint64_t ms)
{
    uint64_t m = esp_timer_get_time();
    if (ms)
    {
        uint64_t e = (m + (ms * 1000));
        if (m > e) // overflow
            while (esp_timer_get_time() > e)
            {
            }
        while (esp_timer_get_time() < e)
        {
        }
    }
}

/* Callback class to handle advertising events */
class AdvCallbacks : public NimBLEExtAdvertisingCallbacks
{
    void onStopped(NimBLEExtAdvertising *pAdv, int reason, uint8_t inst_id)
    {
        /* Check the reason advertising stopped, don't sleep if client is connecting */
        printf("Advertising instance %u stopped\n", inst_id);
        switch (reason)
        {
        case 0:
            printf("Client connecting\n");
            return;
        case BLE_HS_ETIMEOUT:
            printf("Time expired - sleeping for %u seconds\n", _sleep_seconds);
            break;
        default:
            break;
        }

        esp_deep_sleep_start();
    }

private:
    uint32_t _sleep_seconds;

public:
    AdvCallbacks(uint32_t sleepSeconds) : _sleep_seconds(sleepSeconds){};
};

BleHapticDial::BleHapticDial(
    std::string deviceName, std::string deviceManufacturer,
    uint16_t vid, uint16_t pid, uint16_t version,
    uint8_t batteryLevel = 100, uint32_t debounceTime = 10)
    : _device_name(deviceName), _device_manufacturer(deviceManufacturer),
      _vid(vid), _pid(pid), _version(version),
      _battery_level(batteryLevel), _debounce_ms(debounceTime) {}

void BleHapticDial::reset(void)
{
    _button_status = false;
    _dial_degree = 0;
}

void BleHapticDial::begin(void)
{
    this->reset();

    NimBLEDevice::init(_device_name);
    NimBLEDevice::setSecurityAuth(true, true, true);

    // Creating BLE server
    _server = NimBLEDevice::createServer();

    // Setting this object as HID callbacks listener
    _server->setCallbacks(this);

    // Registering hid device on server
    _hid_device = new NimBLEHIDDevice(_server);
    _hid_device->manufacturer(_device_manufacturer);
    _hid_device->pnp(0x02, _vid, _pid, _version);
    _hid_device->hidInfo(0x00, 0x01);
    _hid_device->reportMap((uint8_t *)_HapticDialHIDReportDescriptor, sizeof(_HapticDialHIDReportDescriptor));
    _hid_device->setBatteryLevel(_battery_level);

    // Radial controller handle
    _radial_controller = _hid_device->inputReport(RADIAL_CONTROLLER_REPORT_ID);

    // Haptic feedback handle
    _haptic_feedback = _hid_device->outputReport(HAPTIC_FEEDBACK_REPORT_ID);
    _haptic_feedback->setCallbacks(this);

    // Starting HID service
    _hid_device->startServices();
    onStarted();

    // Setting advertising
    NimBLEExtAdvertisement extAdv(BLE_HCI_LE_PHY_CODED, BLE_HCI_LE_PHY_1M);
    extAdv.setConnectable(true);
    extAdv.setScannable(false);
    extAdv.setServiceData(_hid_device->hidService()->getUUID(), _device_name);
    extAdv.setCompleteServices16({NimBLEUUID(_hid_device->hidService()->getUUID())});

    auto adv = NimBLEDevice::getAdvertising();
    adv->setCallbacks(new AdvCallbacks(_ble_adv_sleep_s));

    if (adv->setInstanceData(0, extAdv))
    {
        if (adv->start(0, _ble_adv_time))
            printf("Started advertising\n");
        else
            printf("Failed to start advertising\n");
    }
    else
        printf("Failed to register advertisment data\n");

    esp_sleep_enable_timer_wakeup(_ble_adv_sleep_s * 1000000);
}

void BleHapticDial::end(void)
{
    this->reset();
}

bool BleHapticDial::isConnected(void)
{
    return _connected;
}

void BleHapticDial::setBatteryLevel(uint8_t level)
{
    _battery_level = level;
    if (_hid_device != nullptr)
        _hid_device->setBatteryLevel(_battery_level);
}

void BleHapticDial::sendReport(void)
{
    HID_DialReport_Data_t report;
    report.button = _button_status;
    report.rotation = _dial_degree;

    if (this->isConnected())
    {
        this->_radial_controller->setValue((uint8_t *)&report, sizeof(report));
        this->_radial_controller->notify();
        delay_ms(_debounce_ms);
    }
}

void BleHapticDial::onConnect(void)
{
    this->_connected = true;
}

void BleHapticDial::onDisconnect(void)
{
    this->_connected = false;
}

void BleHapticDial::button(bool pressed)
{
    if (pressed != _button_status)
    {
        _button_status = pressed;
        this->sendReport();
    }
}

void BleHapticDial::press(void)
{
    this->button(true);
}

void BleHapticDial::release(void)
{
    this->button(false);
}

void BleHapticDial::click(void)
{
    this->press();
    this->release();
}

bool BleHapticDial::isPressed(void)
{
    return this->_button_status;
}

int16_t BleHapticDial::getDialDegrees(void)
{
    return this->_dial_degree;
}