# ESPHome StackM5 Core 2 Components

## AXP192
This custom component it to implement support for the AXP192 on the M5Stack Core2, building on top of martydingo's code and the official m5core2 arduino libary. 

## Installation

Copy the components to a custom_components directory next to your .yaml configuration file.

## Configuration

Sample configurations can be found within `/sample-config`. 
The AXP192 currently has an option to select the model, but only the m5 core 2 is supported in the code.

### M5Stack Core2

```yaml
sensor:
  - platform: axp192
    model: M5CORE2
    address: 0x34
    i2c_id: bus_a
    update_interval: 30s
    battery_level:
      name: "${upper_devicename} Battery Level"
      id: "${devicename}_batterylevel"
```
