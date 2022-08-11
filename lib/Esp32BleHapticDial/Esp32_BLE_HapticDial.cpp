#include "Esp32_BLE_HapticDial.hpp"
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include <NimBLEHIDDevice.h>
#include "HIDTypes.h"
#include <driver/adc.h>
#include "sdkconfig.h"
#include "esp_sleep.h"
static const char *LOG_TAG = "HAPTIC_DIAL";

#define RADIAL_CONTROLLER_REPORT_ID 0x01
#define HAPTIC_FEEDBACK_REPORT_ID 0x02

// doc https://docs.microsoft.com/it-it/windows-hardware/design/component-guidelines/radial-controller-sample-report-descriptors
// Integrated Radial Controller TLC
static const uint8_t _HapticDialHIDReportDescriptor[] = {
    USAGE_PAGE(1), 0x01, // Generic Desktop
    USAGE(1), 0x0e,      // Serial Multi-Axis Controller
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
    PHYSICAL_MAXIMUM(2), 0x10, 0x0e, // 3600
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
    PHYSICAL_MINIMUM(2), 0x03, 0x10, // 0x1003
    PHYSICAL_MAXIMUM(2), 0x03, 0x10, // 0x1003
    FEATURE(1), 0x03,                // Cnst,Var,Abs
    USAGE(1), 0x04,                  // Ordinal 4
    LOGICAL_MINIMUM(1), 0x04,        // 4
    LOGICAL_MAXIMUM(1), 0x04,        // 4
    PHYSICAL_MINIMUM(2), 0x04, 0x10, // 0x1004
    PHYSICAL_MAXIMUM(2), 0x04, 0x10, // 0x1004
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

#if defined(CONFIG_BT_NIMBLE_EXT_ADV)
/* Callback class to handle advertising events */
class AdvCallbacks : public NimBLEExtAdvertisingCallbacks
{
    void onStopped(NimBLEExtAdvertising *pAdv, int reason, uint8_t inst_id)
    {
        /* Check the reason advertising stopped, don't sleep if client is connecting */
        INFO("Advertising instance %u stopped", inst_id);
        switch (reason)
        {
        case 0:
            INFO("Client connecting");
            return;
        case BLE_HS_ETIMEOUT:
            INFO("Time expired - sleeping for %u seconds", _sleep_seconds);
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
#endif

BleHapticDial::BleHapticDial(
    std::string deviceName, std::string deviceManufacturer,
    uint16_t vid, uint16_t pid, uint16_t version,
    uint8_t batteryLevel, uint32_t debounceTime)
    : _device_name(deviceName), _device_manufacturer(deviceManufacturer),
      _vid(vid), _pid(pid), _version(version),
      _battery_level(batteryLevel), _debounce_ms(debounceTime)
{
}

void BleHapticDial::reset(void)
{
    INFO("Resetting");
    _button_status = false;
    _dial_degree = 0;
}

void BleHapticDial::begin(void)
{
    xTaskCreate(this->taskServer, "server", 20000, (void *)this, 5, NULL); // BLE HID start process
}

void BleHapticDial::taskServer(void *pInstance)
{
    BleHapticDial *dial = (BleHapticDial *)pInstance; // static_cast<BleHapticDial *>(pInstance);

    INFO("Begin");
    dial->reset();

    INFO("Initializing BLE env");
    NimBLEDevice::init(dial->_device_name);
    INFO("Setting the authorization mode, enabling pairing");
    NimBLEDevice::setSecurityAuth(true, true, true);

    // Creating BLE server
    INFO("Creating server");
    dial->_server = NimBLEDevice::createServer();

    // Setting this object as HID callbacks listener
    INFO("Setting this object as HID callbacks listener");
    dial->_server->setCallbacks(dial);

    // Registering hid device on server
    INFO("Registering hid device on server");
    dial->_hid_device = new NimBLEHIDDevice(dial->_server);

    INFO("Setting pnp characteristic");
    dial->_hid_device->pnp(0x02, dial->_vid, dial->_pid, dial->_version);
    
    INFO("Setting manufacturer characteristic");
    dial->_hid_device->manufacturer();
    dial->_hid_device->manufacturer(dial->_device_manufacturer);
    
    INFO("Setting hidInfo characteristic");
    dial->_hid_device->hidInfo(0x00, 0x01);

    INFO("Setting HID Report");
    ESP_LOGD (LOG_TAG, " reportMap set: start " );
    ESP_LOGD (LOG_TAG, " reportMap set: size %D " , sizeof(_HapticDialHIDReportDescriptor) );
    dial->_hid_device->reportMap((uint8_t *)_HapticDialHIDReportDescriptor, sizeof(_HapticDialHIDReportDescriptor));
    
    // Radial controller input
    INFO("Creating Radial controller input");
    dial->_radial_controller = dial->_hid_device->inputReport(RADIAL_CONTROLLER_REPORT_ID);
    dial->_radial_controller->setCallbacks(dial);

    // Haptic feedback handle
    INFO("Creating Haptic feedback handle");
    dial->_haptic_feedback = dial->_hid_device->outputReport(HAPTIC_FEEDBACK_REPORT_ID);
    dial->_haptic_feedback->setCallbacks(dial);

    INFO("Setting battery level");
    dial->_hid_device->setBatteryLevel(dial->_battery_level);

    // Starting HID service
    INFO("Starting HID service");
    dial->_hid_device->startServices();
    dial->onStarted();

    // Setting advertising
    INFO("Setting advertising");
#if defined(CONFIG_BT_NIMBLE_EXT_ADV)
    NimBLEExtAdvertisement extAdv(BLE_HCI_LE_PHY_CODED, BLE_HCI_LE_PHY_1M);
    extAdv.setConnectable(true);
    extAdv.setScannable(false);
    extAdv.setServiceData(dial->_hid_device->hidService()->getUUID(), dial->_device_name);
    extAdv.setCompleteServices16({NimBLEUUID(dial->_hid_device->hidService()->getUUID())});

    auto adv = NimBLEDevice::getAdvertising();
    adv->setCallbacks(new AdvCallbacks(dial->_ble_adv_sleep_s));
    if (adv->setInstanceData(0, extAdv))
    {
        if (adv->start(0, dial->_ble_adv_time))
            INFO("Started advertising");
        else
            INFO("Failed to start advertising");
    }
    else
        INFO("Failed to register advertisment data");

        // INFO(("Enabling wakeup by timer of" + std::to_string(_ble_adv_sleep_s) + "s").c_str());
        // esp_sleep_enable_timer_wakeup(_ble_adv_sleep_s * 1000000);
#else
    auto adv = dial->_server->getAdvertising();
    adv->setAppearance(GENERIC_HID);
    adv->addServiceUUID(dial->_hid_device->hidService()->getUUID());
    adv->setScanResponse(false);
    adv->start();
#endif
    ESP_LOGD(LOG_TAG, "Advertising started!");
    vTaskDelay(portMAX_DELAY);
}

void BleHapticDial::end(void)
{
    INFO("End");
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
    INFO("Sending report");
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

/*
void BleHapticDial::onConnect(NimBLEServer *server)
{
    INFO("Device connected. No connection descriptor found.");
    this->_connected = true;
};*/

void BleHapticDial::onConnect(NimBLEServer *server, ble_gap_conn_desc *desc)
{
    this->_connected = true;
    INFO("Client connected. Client address: %s", NimBLEAddress(desc->peer_ota_addr).toString().c_str());
    INFO("Multi-connect support: start advertising");
};

/*
void BleHapticDial::onDisconnect(NimBLEServer *server)
{
    INFO("onDisconnect");
    this->_connected = false;
}
*/

void BleHapticDial::onDisconnect(NimBLEServer *server, ble_gap_conn_desc *desc)
{
    this->_connected = false;
    INFO("Client disconnected - start advertising");
}

void BleHapticDial::onRead(NimBLECharacteristic *characteristic)
{
    INFO("%s: onRead(), value: %s", characteristic->getUUID().toString().c_str(), characteristic->getValue().c_str());
};

void BleHapticDial::onWrite(NimBLECharacteristic *characteristic)
{
    INFO("%s: onWrite(), value: %s", characteristic->getUUID().toString().c_str(),characteristic->getValue().c_str());
};

// Called before notification or indication is sent,
// the value can be changed here before sending if desired.
void BleHapticDial::onNotify(NimBLECharacteristic *characteristic)
{
    INFO("Sending notification to clients");
};

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
    INFO("Pressing button");
    this->button(true);
}

void BleHapticDial::release(void)
{
    INFO("Releasing button");
    this->button(false);
}

void BleHapticDial::click(void)
{
    INFO("Clicking");
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