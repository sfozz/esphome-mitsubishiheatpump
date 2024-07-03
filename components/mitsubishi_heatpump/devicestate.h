#define USE_CALLBACKS

#include "HeatPump.h"
#include "esphome.h"

#ifndef DEVICESTATE_H
#define DEVICESTATE_H

#include <chrono>

namespace devicestate {
  enum DeviceMode {
    DeviceMode_Off,
    DeviceMode_Heat,
    DeviceMode_Cool,
    DeviceMode_Dry,
    DeviceMode_Fan,
    DeviceMode_Auto,
    DeviceMode_Unknown
  };
  DeviceMode toDeviceMode(heatpumpSettings *currentSettings);

  enum FanMode {
    FanMode_Auto,
    FanMode_Quiet,
    FanMode_Low,
    FanMode_Medium,
    FanMode_Middle,
    FanMode_High
  };
  FanMode toFanMode(heatpumpSettings *currentSettings);

  enum SwingMode {
    SwingMode_Both,
    SwingMode_Vertical,
    SwingMode_Horizontal,
    SwingMode_Off
  };
  SwingMode toSwingMode(heatpumpSettings *currentSettings);

  enum VerticalSwingMode {
    VerticalSwingMode_Swing,
    VerticalSwingMode_Auto,
    VerticalSwingMode_Up,
    VerticalSwingMode_UpCenter,
    VerticalSwingMode_Center,
    VerticalSwingMode_DownCenter,
    VerticalSwingMode_Down,
    VerticalSwingMode_Off
  };
  VerticalSwingMode toVerticalSwingMode(heatpumpSettings *currentSettings);

  enum HorizontalSwingMode {
    HorizontalSwingMode_Swing,
    HorizontalSwingMode_Auto,
    HorizontalSwingMode_Left,
    HorizontalSwingMode_LeftCenter,
    HorizontalSwingMode_Center,
    HorizontalSwingMode_RightCenter,
    HorizontalSwingMode_Right,
    HorizontalSwingMode_Off
  };
  HorizontalSwingMode toHorizontalSwingMode(heatpumpSettings *currentSettings);

  const char* deviceModeToString(DeviceMode mode);

  struct DeviceStatus {
    bool operating;
    float currentTemperature;
  };

  DeviceStatus toDeviceState(heatpumpStatus *currentStatus);

  struct DeviceState {
    bool active;
    DeviceMode mode;
    FanMode fanMode;
    SwingMode swingMode;
    VerticalSwingMode verticalSwingMode;
    HorizontalSwingMode horizontalSwingMode;
    float targetTemperature;
    bool connected;
  };

  DeviceState toDeviceState(heatpumpSettings *currentSettings);

  class DeviceStateManager {
    private:
      uint32_t lastInternalPowerUpdate = esphome::millis();
      bool internalPowerOn;

      bool settingsInitialized;
      DeviceState deviceState;

      bool statusInitialized;
      DeviceStatus deviceStatus;

      void hpSettingsChanged();
      void hpStatusChanged(heatpumpStatus currentStatus);
      static void log_packet(byte* packet, unsigned int length, char* packetDirection);
    public:
      DeviceStateManager();

      // HeatPump object using the underlying Arduino library.
      HeatPump* hp;

      DeviceStatus getDeviceStatus();
      DeviceState getDeviceState();

      bool isInitialized();
      void update();
      bool isInternalPowerOn();

      void setCool();
      void setHeat();
      void setDry();
      void setAuto();
      void setFan();
      
      void turnOn(const char* mode);
      void turnOff();

      bool internalTurnOn();
      bool internalTurnOff();
  };
}
#endif