/**
 ******************************************************************************
 * @file    X_NUCLEO_53L0A1_Gesture_DirSwipe.ino
 * @author  AST
 * @version V1.0.0
 * @date    21 April 2017
 * @brief   Arduino test application for the STMicrolectronics X-NUCLEO-53L0A1
 *          proximity sensor expansion board based on FlightSense and gesture library.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <vl53l0x_x_nucleo_53l0a1_class.h>
#include <stmpe1600_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <tof_gestures.h>
#include <tof_gestures_DIRSWIPE_1.h>

#ifdef ARDUINO_SAM_DUE
#define DEV_I2C Wire1
#elif defined(ARDUINO_ARCH_STM32)
#define DEV_I2C Wire
#else
#define DEV_I2C Wire
#endif
#define SerialPort Serial

//For SAM compatibility where D8 and D2 are undefined
#ifndef D8
#define D8 8
#endif


#ifndef D2
#define D2 2
#endif

// Components.
STMPE1600DigiOut *xshutdown_top;
STMPE1600DigiOut *xshutdown_left;
STMPE1600DigiOut *xshutdown_right;
VL53L0X_X_NUCLEO_53L0A1 *sensor_vl53l0x_top;
VL53L0X_X_NUCLEO_53L0A1 *sensor_vl53l0x_left;
VL53L0X_X_NUCLEO_53L0A1 *sensor_vl53l0x_right;

// Gesture structure.
Gesture_DIRSWIPE_1_Data_t gestureDirSwipeData;

// Range values
uint32_t distance_left, distance_right;

/**
 *  Setup all sensors for single shot mode
 */
void SetupSingleShot(void){
  int status;
  uint8_t VhvSettings;
  uint8_t PhaseCal;
  uint32_t refSpadCount;
  uint8_t isApertureSpads;

  status = sensor_vl53l0x_left->StaticInit();
  if( status ){
    SerialPort.println("StaticInit left sensor failed");
  }

  status = sensor_vl53l0x_left->PerformRefCalibration(&VhvSettings, &PhaseCal);
  if( status ){
    SerialPort.println("PerformRefCalibration left sensor failed");
  }

  status = sensor_vl53l0x_left->PerformRefSpadManagement(&refSpadCount, &isApertureSpads);
  if( status ){
    SerialPort.println("PerformRefSpadManagement left sensor failed");
  }

  status = sensor_vl53l0x_left->SetDeviceMode(VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
  if( status ){
    SerialPort.println("SetDeviceMode left sensor failed");
  }

  status = sensor_vl53l0x_left->SetMeasurementTimingBudgetMicroSeconds(20*1000);
  if( status ){
    SerialPort.println("SetMeasurementTimingBudgetMicroSeconds left sensor failed");
  }

  status = sensor_vl53l0x_right->StaticInit();
  if( status ){
    SerialPort.println("StaticInit right sensor failed");
  }

  status = sensor_vl53l0x_right->PerformRefCalibration(&VhvSettings, &PhaseCal);
  if( status ){
    SerialPort.println("PerformRefCalibration right sensor failed");
  }

  status = sensor_vl53l0x_right->PerformRefSpadManagement(&refSpadCount, &isApertureSpads);
  if( status ){
    SerialPort.println("PerformRefSpadManagement right sensor failed");
  }

  status = sensor_vl53l0x_right->SetDeviceMode(VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
  if( status ){
    SerialPort.println("SetDeviceMode right sensor failed");
  }

  status = sensor_vl53l0x_right->SetMeasurementTimingBudgetMicroSeconds(20*1000);
  if( status ){
    SerialPort.println("SetMeasurementTimingBudgetMicroSeconds right sensor failed");
  }
}

/* Setup ---------------------------------------------------------------------*/

void setup() {
  int status;
  // Led.
  pinMode(13, OUTPUT);

  // Initialize serial for output.
  SerialPort.begin(115200);

//NOTE: workaround in order to unblock the I2C bus on the Arduino Due
#ifdef ARDUINO_SAM_DUE
   pinMode(71, OUTPUT);
   pinMode(70, OUTPUT);

   for (int i = 0; i<10; i++){
     digitalWrite(70, LOW);
     delay(3);
     digitalWrite(71, HIGH);
     delay(3);
     digitalWrite(70, HIGH);
     delay(3);
     digitalWrite(71, LOW);
     delay(3);
   }
   pinMode(70, INPUT);
   pinMode(71, INPUT);
#endif
//End of workaround

  // Initialize I2C bus.
  DEV_I2C.begin();
  DEV_I2C.setClock(400000);

  // Create VL53L0X top component.
  xshutdown_top = new STMPE1600DigiOut(&DEV_I2C, GPIO_15, (0x42 * 2));
  sensor_vl53l0x_top = new VL53L0X_X_NUCLEO_53L0A1(&DEV_I2C, xshutdown_top, A2);
  
  // Switch off VL53L0X top component.
  sensor_vl53l0x_top->VL53L0X_Off();
  
  // Create (if present) VL53L0X left component.
  xshutdown_left = new STMPE1600DigiOut(&DEV_I2C, GPIO_14, (0x43 * 2));
  sensor_vl53l0x_left = new VL53L0X_X_NUCLEO_53L0A1(&DEV_I2C, xshutdown_left, D8);
  
  // Switch off (if present) VL53L0X left component.
  sensor_vl53l0x_left->VL53L0X_Off();
  
  // Create (if present) VL53L0X right component.
  xshutdown_right = new STMPE1600DigiOut(&DEV_I2C, GPIO_15, (0x43 * 2));
  sensor_vl53l0x_right = new VL53L0X_X_NUCLEO_53L0A1(&DEV_I2C, xshutdown_right, D2);
  
  // Switch off (if present) VL53L0X right component.
  sensor_vl53l0x_right->VL53L0X_Off();

  // Initialize VL53L0X left component.
  status = sensor_vl53l0x_left->InitSensor(0x12);
  if(status)
  {
    SerialPort.println("Init sensor_vl53l0x_left failed...");
  }

  // Initialize VL53L0X right component.
  status = sensor_vl53l0x_right->InitSensor(0x14);
  if(status)
  {
    SerialPort.println("Init sensor_vl53l0x_right failed...");
  }
  
  // Initialize VL6180X gesture library.
  tof_gestures_initDIRSWIPE_1(400, 0, 1000, &gestureDirSwipeData);

  SetupSingleShot();
}


/* Loop ----------------------------------------------------------------------*/

void loop() {
  int gesture_code;

  sensor_vl53l0x_left->StartMeasurement();
  sensor_vl53l0x_right->StartMeasurement();

  int left_done = 0;
  int right_done = 0;
  uint8_t NewDataReady=0;
  VL53L0X_RangingMeasurementData_t pRangingMeasurementData;

  do
  {
    if(left_done == 0)
    {
      NewDataReady = 0;
      int status = sensor_vl53l0x_left->GetMeasurementDataReady(&NewDataReady);

      if( status ){
        SerialPort.println("GetMeasurementDataReady left sensor failed");
      }
      
      if(NewDataReady)
      {
        status = sensor_vl53l0x_left->ClearInterruptMask(0);
        if( status ){
          SerialPort.println("ClearInterruptMask left sensor failed");
        }

        status = sensor_vl53l0x_left->GetRangingMeasurementData(&pRangingMeasurementData);
        if( status ){
          SerialPort.println("GetRangingMeasurementData left sensor failed");
        }

        if (pRangingMeasurementData.RangeStatus == 0) {
          // we have a valid range.
          distance_left = pRangingMeasurementData.RangeMilliMeter;
        }else {
          distance_left = 1200;
        }
        
        left_done = 1;
      }
    }
    
    if(right_done == 0)
    {
      NewDataReady = 0;
      int status = sensor_vl53l0x_right->GetMeasurementDataReady(&NewDataReady);

      if( status ){
        SerialPort.println("GetMeasurementDataReady right sensor failed");
      }
      
      if(NewDataReady)
      {
        status = sensor_vl53l0x_right->ClearInterruptMask(0);
        if( status ){
          SerialPort.println("ClearInterruptMask right sensor failed");
        }

        status = sensor_vl53l0x_right->GetRangingMeasurementData(&pRangingMeasurementData);
        if( status ){
          SerialPort.println("GetRangingMeasurementData right sensor failed");
        }

        if (pRangingMeasurementData.RangeStatus == 0) {
          // we have a valid range.
          distance_right = pRangingMeasurementData.RangeMilliMeter;
        }else {
          distance_right = 1200;
        }
        
        right_done = 1;
      }
    }
  }while(left_done == 0 || right_done == 0);

  // Launch gesture detection algorithm.
  gesture_code = tof_gestures_detectDIRSWIPE_1(distance_left, distance_right, &gestureDirSwipeData);

  // Check the result of the gesture detection algorithm.
  switch(gesture_code)
  {
    case GESTURES_SWIPE_LEFT_RIGHT:
      SerialPort.println("From LEFT to RIGHT --->");
      break;
    case GESTURES_SWIPE_RIGHT_LEFT:
      SerialPort.println("From RIGHT to LEFT <---");
      break;
    default:
      // Do nothing
      break;
  }
}

