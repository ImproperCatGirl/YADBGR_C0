/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

 #include <stdlib.h>
 #include <stdio.h>
#include <stm32c071xx.h>
#include <stm32c0xx_hal_cortex.h>
#include <stm32c0xx_hal_gpio.h>
 #include <string.h>
 
 #include "tusb.h"
 
#include <stm32c0xx_hal.h>
 /* This example demonstrate HID Generic raw Input & Output.
  * It will receive data from Host (In endpoint) and echo back (Out endpoint).
  * HID Report descriptor use vendor for usage page (using template TUD_HID_REPORT_DESC_GENERIC_INOUT)
  *
  * There are 2 ways to test the sketch
  * 1. Using nodejs
  * - Install nodejs and npm to your PC
  *
  * - Install excellent node-hid (https://github.com/node-hid/node-hid) by
  *   $ npm install node-hid
  *
  * - Run provided hid test script
  *   $ node hid_test.js
  *
  * 2. Using python
  * - Install `hid` package (https://pypi.org/project/hid/) by
  *   $ pip install hid
  *
  * - hid package replies on hidapi (https://github.com/libusb/hidapi) for backend,
  *   which already available in Linux. However on windows, you may need to download its dlls from their release page and
  *   copy it over to folder where python is installed.
  *
  * - Run provided hid test script to send and receive data to this device.
  *   $ python3 hid_test.py
  */
 
 //--------------------------------------------------------------------+
 // MACRO CONSTANT TYPEDEF PROTYPES
 //--------------------------------------------------------------------+
 
 /* Blink pattern
  * - 250 ms  : device not mounted
  * - 1000 ms : device mounted
  * - 2500 ms : device is suspended
  */
 enum  {
   BLINK_NOT_MOUNTED = 250,
   BLINK_MOUNTED = 1000,
   BLINK_SUSPENDED = 2500,
 };
 
 static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

extern volatile uint32_t system_ticks;
 
 void led_blinking_task(void);

static GPIO_InitTypeDef  GPIO_InitStruct;
 
 /*------------- MAIN -------------*/
 int usb_init(void)
 {
  
  // Enable peripheral clocks.
  __HAL_RCC_USB_CLK_ENABLE();
  __HAL_RCC_USB_FORCE_RESET();
  __HAL_RCC_USB_RELEASE_RESET();
  /*GPIO_InitStruct.Alternate = GPIO_AF2_USB;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;*/
  //HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   tusb_rhport_init_t dev_init = {
     .role = TUSB_ROLE_DEVICE,
     .speed = TUSB_SPEED_AUTO
   };

  //NVIC_EnableIRQ(USB_DRD_FS_IRQn);

  NVIC_SetPriority(USB_DRD_FS_IRQn, 6);
   tusb_init(BOARD_TUD_RHPORT, &dev_init);

 }


 void usb_task(void *pvParameters)
 {
  usb_init();
    while (1) {
      // put this thread to waiting state until there is new events
      tud_task();
    }
 }
 
 
 //--------------------------------------------------------------------+
 // Device callbacks
 //--------------------------------------------------------------------+
 
uint32_t board_millis(void) {
    return system_ticks;
  }
 // Invoked when device is mounted
 void tud_mount_cb(void)
 {
   blink_interval_ms = BLINK_MOUNTED;
 }
 
 // Invoked when device is unmounted
 void tud_umount_cb(void)
 {
   blink_interval_ms = BLINK_NOT_MOUNTED;
 }
 
 // Invoked when usb bus is suspended
 // remote_wakeup_en : if host allow us  to perform remote wakeup
 // Within 7ms, device must draw an average of current less than 2.5 mA from bus
 void tud_suspend_cb(bool remote_wakeup_en)
 {
   (void) remote_wakeup_en;
   blink_interval_ms = BLINK_SUSPENDED;
 }
 
 // Invoked when usb bus is resumed
 void tud_resume_cb(void)
 {
   blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
 }
 
