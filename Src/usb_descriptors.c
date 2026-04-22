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

#include "tusb.h"

/* A combination of interfaces must have a unique product id, since PC will save device driver after the first plug.
 * Same VID/PID with different interface e.g MSC (first), then CDC (later) will possibly cause system error on PC.
 *
 * Auto ProductID layout's Bitmap:
 *   [MSB]         HID | MSC | CDC          [LSB]
 */
#define PID_MAP(itf, n)  ((CFG_TUD_##itf) ? (1 << (n)) : 0)
#define USB_PID           (0x4000 | PID_MAP(CDC, 0) | PID_MAP(MSC, 1) | PID_MAP(HID, 2) | \
                           PID_MAP(MIDI, 3) | PID_MAP(VENDOR, 4) )

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
static tusb_desc_device_t const desc_device_HID =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0x0D28,
    .idProduct          = 0x0204,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

tusb_desc_device_t const desc_device = {
  .bLength            = sizeof(tusb_desc_device_t),
  .bDescriptorType    = TUSB_DESC_DEVICE,
  .bcdUSB             = 0x0200,         // USB 2.1 (Needed for BOS/WinUSB)
  .bDeviceClass       = 0xEF,           // Miscellaneous
  .bDeviceSubClass    = 0x02,           // Common Class
  //.bDeviceProtocol    = 0x01,           // Interface Association Descriptor
  .bDeviceProtocol    = MISC_PROTOCOL_IAD,
  .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
  
  .idVendor           = 0xcafe,         // Your VID
  .idProduct          = 0x4002,         // Your PID
  .bcdDevice          = 0x0100,
  
  .iManufacturer      = 0x01,
  .iProduct           = 0x02,           // String "CMSIS-DAP" is usually here
  .iSerialNumber      = 0x03,
  .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void)
{
  return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// HID Report Descriptor
//--------------------------------------------------------------------+



//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

enum
{
  ITF_NUM_DAP,
  ITF_NUM_CDC,
  ITF_NUM_CDC_DATA,
  ITF_NUM_VENDOR,
  ITF_NUM_TOTAL
};

#define  CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN + TUD_CDC_DESC_LEN + TUD_VENDOR_DESC_LEN)


#define EPNUM_DAP_OUT       0x01
#define EPNUM_DAP_IN        0x81
#define EPNUM_CDC_NOTIF     0x82
#define EPNUM_CDC_OUT       0x03
#define EPNUM_CDC_IN        0x83
#define DDMI_IN 0x84
#define DDMI_OUT 0x04

uint8_t const desc_configuration[] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  //TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, 0x00, 100),
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

  // Interface 0: CMSIS-DAP v2
  // bLength, bDescriptorType, bInterfaceNumber, bAlternateSetting, 
  // bNumEndpoints, bInterfaceClass, bInterfaceSubClass, bInterfaceProtocol, iInterface
  0x09, TUSB_DESC_INTERFACE, 0x00, 0x00, 0x02, 0xFF, 0x00, 0x00, 0x04,
  //                                     ^-- CHANGED TO 2 (Bulk IN + Bulk OUT)

  // Endpoint 1: Bulk OUT (Commands)
  0x07, TUSB_DESC_ENDPOINT, EPNUM_DAP_OUT, TUSB_XFER_BULK, 0x40, 0x00, 0x00,

  // Endpoint 2: Bulk IN (Responses)
  0x07, TUSB_DESC_ENDPOINT, EPNUM_DAP_IN, TUSB_XFER_BULK, 0x40, 0x00, 0x00,

  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 5, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),

  0x09, TUSB_DESC_INTERFACE, ITF_NUM_VENDOR, 0x00, 0x02, 0xFF, 0x00, 0x00, 0x06,
  0x07, TUSB_DESC_ENDPOINT, DDMI_OUT, TUSB_XFER_BULK, 0x40, 0x00, 0x00,
  0x07, TUSB_DESC_ENDPOINT, DDMI_IN, TUSB_XFER_BULK, 0x40, 0x00, 0x00

};

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index; // for multiple configurations
  return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// String Descriptor Index
enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
};

char const* string_desc_arr [] = {
  (const char[]) { 0x09, 0x04 }, // 0: Supported lang (English)
  "YourProject",                 // 1: Manufacturer
  "CMSIS-DAP Debugger",          // 2: Product (Crucial: must contain CMSIS-DAP)
  "123456",                      // 3: Serials
  "CMSIS-DAP v2 Interface",      // 4: Interface 0 String
  "Target UART",                 // 5: Interface 1/2 String
  "DDMI PoC"          // 6: Interface 3/4 String
};

static uint16_t _desc_str[32 + 1];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void) langid;
  size_t chr_count;

  switch ( index ) {
    case STRID_LANGID:
      memcpy(&_desc_str[1], string_desc_arr[0], 2);
      chr_count = 1;
      break;

    case STRID_SERIAL:
      chr_count = 0;
      break;

    default:
      // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
      // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

      if ( !(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) ) return NULL;

      const char *str = string_desc_arr[index];

      // Cap at max char
      chr_count = strlen(str);
      size_t const max_count = sizeof(_desc_str) / sizeof(_desc_str[0]) - 1; // -1 for string type
      if ( chr_count > max_count ) chr_count = max_count;

      // Convert ASCII string into UTF-16
      for ( size_t i = 0; i < chr_count; i++ ) {
        _desc_str[1 + i] = str[i];
      }
      break;
  }

  // first byte is length (including header), second byte is string type
  _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));

  return _desc_str;
}
