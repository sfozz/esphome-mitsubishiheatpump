#include "devicestate.h"

namespace devicestate {
    static const char* TAG = "DeviceStateManager"; // Logging tag

    bool same_float(const float left, const float right, const float delta) {
        return fabs(left - right) <= 0.001;
    }

    bool deviceStatusEqual(DeviceStatus left, DeviceStatus right) {
        return left.operating == right.operating &&
        same_float(left.currentTemperature, right.currentTemperature, 0.1) &&
        left.compressorFrequency == right.compressorFrequency;
    }

    bool deviceStateEqual(DeviceState left, DeviceState right) {
        return left.active == right.active &&
        left.mode == right.mode &&
        left.fanMode == right.fanMode &&
        left.swingMode == right.swingMode &&
        left.verticalSwingMode == right.verticalSwingMode &&
        left.horizontalSwingMode == right.horizontalSwingMode &&
        same_float(left.targetTemperature, right.targetTemperature, 0.1) &&
        left.connected == right.connected;
    }

    // Function to convert a Color enum value to its string 
    // representation 
    const char* deviceModeToString(DeviceMode mode) 
    { 
        switch (mode) {
            case DeviceMode::DeviceMode_Off:
            return "Off";
            case DeviceMode::DeviceMode_Heat:
            return "Heat";
            case DeviceMode::DeviceMode_Cool:
            return "Cool";
            case DeviceMode::DeviceMode_Dry:
            return "Dry";
            case DeviceMode::DeviceMode_Fan:
            return "Fan";
            case DeviceMode::DeviceMode_Auto:
            return "Auto";
            default:
            return "Unknown";
        }
    } 

    bool isDeviceActive(heatpumpSettings *currentSettings) {
        return strcmp(currentSettings->power, "ON") == 0;
    }

    SwingMode toSwingMode(heatpumpSettings *currentSettings) {
        if (strcmp(currentSettings->vane, "SWING") == 0 &&
                strcmp(currentSettings->wideVane, "SWING") == 0) {
            return SwingMode::SwingMode_Both;
        } else if (strcmp(currentSettings->vane, "SWING") == 0) {
            return SwingMode::SwingMode_Vertical;
        } else if (strcmp(currentSettings->wideVane, "SWING") == 0) {
            return SwingMode::SwingMode_Horizontal;
        } else {
            return SwingMode::SwingMode_Off;
        }
    }

    VerticalSwingMode toVerticalSwingMode(heatpumpSettings *currentSettings) {
        if (strcmp(currentSettings->vane, "SWING") == 0) {
            return VerticalSwingMode::VerticalSwingMode_Swing;
        } else if (strcmp(currentSettings->vane, "AUTO") == 0) {
            return VerticalSwingMode::VerticalSwingMode_Auto;
        } else if (strcmp(currentSettings->vane, "1") == 0) {
            return VerticalSwingMode::VerticalSwingMode_Up;
        } else if (strcmp(currentSettings->vane, "2") == 0) {
            return VerticalSwingMode::VerticalSwingMode_UpCenter;
        } else if (strcmp(currentSettings->vane, "3") == 0) {
            return VerticalSwingMode::VerticalSwingMode_Center;
        } else if (strcmp(currentSettings->vane, "4") == 0) {
            return VerticalSwingMode::VerticalSwingMode_DownCenter;
        } else if (strcmp(currentSettings->vane, "5") == 0) {
            return VerticalSwingMode::VerticalSwingMode_Down;
        } else {
            return VerticalSwingMode::VerticalSwingMode_Off;
        }
    }

    HorizontalSwingMode toHorizontalSwingMode(heatpumpSettings *currentSettings) {
        if (strcmp(currentSettings->wideVane, "SWING") == 0) {
            return HorizontalSwingMode::HorizontalSwingMode_Swing;
        } else if (strcmp(currentSettings->wideVane, "<>") == 0) {
            return HorizontalSwingMode::HorizontalSwingMode_Auto;
        } else if (strcmp(currentSettings->wideVane, "<<") == 0) {
            return HorizontalSwingMode::HorizontalSwingMode_Left;
        } else if (strcmp(currentSettings->wideVane, "<") == 0) {
            return HorizontalSwingMode::HorizontalSwingMode_LeftCenter;
        } else if (strcmp(currentSettings->wideVane, "|") == 0) {
            return HorizontalSwingMode::HorizontalSwingMode_Center;
        } else if (strcmp(currentSettings->wideVane, ">") == 0) {
            return HorizontalSwingMode::HorizontalSwingMode_RightCenter;
        } else if (strcmp(currentSettings->wideVane, ">>") == 0) {
            return HorizontalSwingMode::HorizontalSwingMode_Right;
        } else {
            return HorizontalSwingMode::HorizontalSwingMode_Off;
        }
    }

    FanMode toFanMode(heatpumpSettings *currentSettings) {
        if (strcmp(currentSettings->fan, "QUIET") == 0) {
            return FanMode::FanMode_Quiet;
        } else if (strcmp(currentSettings->fan, "1") == 0) {
            return FanMode::FanMode_Low;
        } else if (strcmp(currentSettings->fan, "2") == 0) {
            return FanMode::FanMode_Medium;
        } else if (strcmp(currentSettings->fan, "3") == 0) {
            return FanMode::FanMode_Middle;
        } else if (strcmp(currentSettings->fan, "4") == 0) {
            return FanMode::FanMode_High;
        } else { //case "AUTO" or default:
            return FanMode::FanMode_Auto;
        }
    }

    DeviceMode toDeviceMode(heatpumpSettings *currentSettings) {
        if (strcmp(currentSettings->mode, "HEAT") == 0) {
            return DeviceMode::DeviceMode_Heat;
        } else if (strcmp(currentSettings->mode, "DRY") == 0) {
            return DeviceMode::DeviceMode_Dry;
        } else if (strcmp(currentSettings->mode, "COOL") == 0) {
        return DeviceMode::DeviceMode_Cool;
        } else if (strcmp(currentSettings->mode, "FAN") == 0) {
            return DeviceMode::DeviceMode_Fan;
        } else if (strcmp(currentSettings->mode, "AUTO") == 0) {
            return DeviceMode::DeviceMode_Auto;
        } else {
            return DeviceMode::DeviceMode_Unknown;
        }
    }

    DeviceStatus toDeviceStatus(heatpumpStatus *currentStatus) {
        DeviceStatus deviceStatus;
        deviceStatus.currentTemperature = currentStatus->roomTemperature;
        deviceStatus.operating = currentStatus->operating;
        deviceStatus.compressorFrequency = currentStatus->compressorFrequency;
        return deviceStatus;
    }

    DeviceState toDeviceState(heatpumpSettings *currentSettings) {
        /*
        * ************ HANDLE POWER AND MODE CHANGES ***********
        * https://github.com/geoffdavis/HeatPump/blob/stream/src/HeatPump.h#L125
        * const char* POWER_MAP[2]       = {"OFF", "ON"};
        * const char* MODE_MAP[5]        = {"HEAT", "DRY", "COOL", "FAN", "AUTO"};
        */
        DeviceState deviceState;
        deviceState.active = isDeviceActive(currentSettings);
        deviceState.mode = toDeviceMode(currentSettings);
        deviceState.fanMode = toFanMode(currentSettings);
        deviceState.swingMode = toSwingMode(currentSettings);
        deviceState.verticalSwingMode = toVerticalSwingMode(currentSettings);
        deviceState.horizontalSwingMode = toHorizontalSwingMode(currentSettings);
        deviceState.targetTemperature = currentSettings->temperature;
        deviceState.connected = currentSettings->connected;
        return deviceState;
    }

    DeviceStateManager::DeviceStateManager(
      esphome::binary_sensor::BinarySensor* internal_power_on,
      esphome::binary_sensor::BinarySensor* device_state_connected,
      esphome::binary_sensor::BinarySensor* device_state_active,
      esphome::sensor::Sensor* device_state_last_updated,
      esphome::binary_sensor::BinarySensor* device_status_operating,
      esphome::sensor::Sensor* device_status_compressor_frequency,
      esphome::sensor::Sensor* device_status_last_updated
    ) {
        this->internal_power_on = internal_power_on;
        this->device_state_connected = device_state_connected;
        this->device_state_active = device_state_active;
        this->device_state_last_updated = device_state_last_updated;
        this->device_status_operating = device_status_operating;
        this->device_status_compressor_frequency = device_status_compressor_frequency;
        this->device_status_last_updated = device_status_last_updated;

        this->deviceStateLastUpdated = 0;
        this->deviceStatusLastUpdated = 0;

        ESP_LOGCONFIG(TAG, "Initializing new HeatPump object.");
        this->hp = new HeatPump();

        #ifdef USE_CALLBACKS
            hp->setSettingsChangedCallback(
                    [this]() {
                        ESP_LOGW(TAG, "Callback hpSettingsChanged");
                        this->hpSettingsChanged();
                    }
            );

            hp->setStatusChangedCallback(
                    [this](heatpumpStatus currentStatus) {
                        ESP_LOGW(TAG, "Callback hpStatusChanged");
                        this->hpStatusChanged(currentStatus);
                    }
            );

            hp->setPacketCallback(this->log_packet);
        #endif
    }

    void DeviceStateManager::hpSettingsChanged() {
        heatpumpSettings currentSettings = hp->getSettings();
        if (currentSettings.power == NULL) {
            /*
            * We should always get a valid pointer here once the HeatPump
            * component fully initializes. If HeatPump hasn't read the settings
            * from the unit yet (hp->connect() doesn't do this, sadly), we'll need
            * to punt on the update. Likely not an issue when run in callback
            * mode, but that isn't working right yet.
            */
            ESP_LOGW(TAG, "Waiting for HeatPump to read the settings the first time.");
            esphome::delay(10);
            return;
        }

        const DeviceState deviceState = devicestate::toDeviceState(&currentSettings);
        if (this->settingsInitialized) {
            if (this->internalPowerOn != deviceState.active) {
                ESP_LOGI(TAG, "Device active on change: deviceState.active={%s} internalPowerOn={%s}", YESNO(deviceState.active), YESNO(this->internalPowerOn));
            }
        } else {
            if (this->internalPowerOn != deviceState.active) {
                ESP_LOGW(TAG, "Initializing internalPowerOn state from %s to %s", ONOFF(this->internalPowerOn), ONOFF(deviceState.active));
                this->internalPowerOn = deviceState.active;
            }
            this->settingsInitialized = true;
        }
        this->deviceState = deviceState;

        this->device_state_connected->publish_state(this->deviceState.connected);
        this->device_state_active->publish_state(this->deviceState.active);

        this->deviceStateLastUpdated += 1;
        this->device_state_last_updated->publish_state(this->deviceStateLastUpdated);
    }

    /**
     * Report changes in the current temperature sensed by the HeatPump.
     */
    void DeviceStateManager::hpStatusChanged(heatpumpStatus currentStatus) {
        if (!this->statusInitialized) {
            ESP_LOGW(TAG, "HeatPump status initialized.");
        }
        this->statusInitialized = true;
        this->deviceStatus = devicestate::toDeviceStatus(&currentStatus);

        this->device_status_operating->publish_state(this->deviceStatus.operating);
        this->device_status_compressor_frequency->publish_state(this->deviceStatus.compressorFrequency);

        this->deviceStatusLastUpdated += 1;
        this->device_status_last_updated->publish_state(this->deviceStatusLastUpdated);
    }

    void DeviceStateManager::log_packet(byte* packet, unsigned int length, char* packetDirection) {
        String packetHex;
        char textBuf[15];

        for (int i = 0; i < length; i++) {
            memset(textBuf, 0, 15);
            sprintf(textBuf, "%02X ", packet[i]);
            packetHex += textBuf;
        }
        
        ESP_LOGV(TAG, "PKT: [%s] %s", packetDirection, packetHex.c_str());
    }

    bool DeviceStateManager::isInitialized() {
        return this->settingsInitialized && this->statusInitialized;
    }

    DeviceStatus DeviceStateManager::getDeviceStatus() {
        return this->deviceStatus;
    }

    DeviceState DeviceStateManager::getDeviceState() {
        return this->deviceState;
    }

    void DeviceStateManager::update() {
        // This will be called every "update_interval" milliseconds.
        //this->dump_config();
        this->hp->sync();
    #ifndef USE_CALLBACKS
        this->hpSettingsChanged();
        heatpumpStatus currentStatus = hp->getStatus();
        this->hpStatusChanged(currentStatus);
    #endif
    }

    bool DeviceStateManager::isInternalPowerOn() {
        return this->internalPowerOn;
    }

    void DeviceStateManager::setCool() {
        this->turnOn("COOL");
    }

    void DeviceStateManager::setHeat() {
        this->turnOn("HEAT");
    }

    void DeviceStateManager::setDry() {
        this->turnOn("DRY");
    }

    void DeviceStateManager::setAuto() {
        this->turnOn("AUTO");
    }

    void DeviceStateManager::setFan() {
        this->turnOn("FAN");
    }

    bool DeviceStateManager::turnOn(const char* mode) {
        this->hp->setModeSetting(mode);
        this->hp->setPowerSetting("ON");

        if (this->hp->update()) {
            this->internalPowerOn = true;
            this->internal_power_on->publish_state(this->internalPowerOn);
            ESP_LOGW(TAG, "Performed turn on!");
            return true;
        } else {
            ESP_LOGW(TAG, "Failed to perform turn on!");
            return false;
        }
    }

    bool DeviceStateManager::turnOff() {
        this->hp->setPowerSetting("OFF");
        if (this->hp->update()) {
            this->internalPowerOn = false;
            this->internal_power_on->publish_state(this->internalPowerOn);
            ESP_LOGW(TAG, "Performed turn off!");
            return true;
        } else {
            ESP_LOGW(TAG, "Failed to perform turn off!");
            return false;
        }
    }

    bool DeviceStateManager::internalTurnOn() {
        const uint32_t end = esphome::millis();
        const int durationInMilliseconds = end - this->lastInternalPowerUpdate;
        const int durationInSeconds = durationInMilliseconds / 1000;
        const int remaining = 60 - durationInSeconds; 
        if (remaining > 0) {
            ESP_LOGD(TAG, "Throttling internal turn on: %i seconds remaining", remaining);
            return false;
        }

        this->hp->setPowerSetting("ON");
        if (this->hp->update()) {
            this->lastInternalPowerUpdate = end;
            this->internalPowerOn = true;
            this->internal_power_on->publish_state(this->internalPowerOn);
            ESP_LOGW(TAG, "Performed internal turn on!");
            return true;
        } else {
            ESP_LOGW(TAG, "Failed to perform internal turn on!");
            return false;
        }
    }

    bool DeviceStateManager::internalTurnOff() {
        const uint32_t end = esphome::millis();
        const int durationInMilliseconds = end - this->lastInternalPowerUpdate;
        const int durationInSeconds = durationInMilliseconds / 1000;
        const int remaining = 60 - durationInSeconds; 
        if (remaining > 0) {
            ESP_LOGD(TAG, "Throttling internal turn off: %i seconds remaining", remaining);
            return false;
        }

        this->hp->setPowerSetting("OFF");
        if (this->hp->update()) {
            this->lastInternalPowerUpdate = end;
            this->internalPowerOn = false;
            this->internal_power_on->publish_state(this->internalPowerOn);
            ESP_LOGW(TAG, "Performed internal turn off!");
            return true;
        } else {
            ESP_LOGW(TAG, "Failed to perform internal turn off!");
            return false;
        }
    }
}