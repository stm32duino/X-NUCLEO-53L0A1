# X-NUCLEO-53L0A1

Arduino library to support the X-NUCLEO-53L0A1 based on VL53L0X Time-of-Flight and gesture-detection sensor
This sensor uses I2C to communicate. An I2C instance is required to access to the sensor.
The APIs provide simple distance measure, single swipe gesture detection,
directional (left/right) swipe gesture detection and single tap gesture detection.

## Examples

There are 4 examples with the  X-NUCLEO-53L0A1 library.
* X_NUCLEO_53L0A1_HelloWorld: This example code is to show how to get proximity
  values of the onboard VL53L0X sensor and eventually of the two satellites

* X_NUCLEO_53L0A1_Gesture_DirSwipe: This example code is to show how to combine the
  proximity values of the two VL53L0X sensor satellites together with the gesture library
  in order to detect a directional swipe gesture.

* X_NUCLEO_53L0A1_Gesture_Swipe1: This example code is to show how to combine the
  proximity value of the onboard VL53L0X sensor together with the gesture library
  in order to detect a simple swipe gesture without considering the direction.

* X_NUCLEO_53L0A1_Gesture_Tap1: This example code is to show how to combine the
  proximity value of the onboard VL53L0X sensor together with the gesture
  library in order to detect a simple tap gesture.

## Dependencies

This package requires the following Arduino libraries:
* STM32duino VL53L0X: https://github.com/stm32duino/VL53L0X
* STM32duino Proximity Gesture: https://github.com/stm32duino/Proximity_Gesture
  
## Note

The maximum detection distance is influenced by the color of the target and
the indoor or outdoor situation due to absence or presence of external
infrared.
The detection range can be comprise between ~40cm and ~120cm. (see chapter 5 of
the VL53L0X datasheet).
If you need an higher accuracy (up to +200cm), you should implement your own
function.

## Documentation

You can find the source files at  
https://github.com/stm32duino/X-NUCLEO-53L0A1

The VL53L0X datasheet is available at  
http://www.st.com/content/st_com/en/products/imaging-and-photonics-solutions/proximity-sensors/vl53l0x.html
