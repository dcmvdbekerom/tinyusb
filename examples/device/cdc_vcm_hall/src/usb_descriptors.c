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

#include "bsp/board_api.h"
#include "tusb.h"

// Unique PID per example: guarantees re-enumeration on re-flash and a fresh host driver match.
#define USB_PID   0x133F

#define USB_VID   0xCafe
#define USB_BCD   0x0200

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
static tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             =  0x0200, //USB_BCD,

    // Use Interface Association Descriptor (IAD) for CDC
    // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const *tud_descriptor_device_cb(void) {
  return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
enum {
  ITF_NUM_CDC_0 = 0,
  ITF_NUM_CDC_0_DATA,
  // ITF_NUM_CDC_1,
  // ITF_NUM_CDC_1_DATA,
  ITF_NUM_VENDOR,
  ITF_NUM_TOTAL
};

//#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + CFG_TUD_CDC * TUD_CDC_DESC_LEN)
#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_VENDOR_DESC_LEN)

#define EPNUM_CDC_0_NOTIF   0x81
#define EPNUM_CDC_0_OUT     0x02
#define EPNUM_CDC_0_IN      0x82

#define EPNUM_VENDOR_OUT    0x03  // New Vendor OUT Endpoint
#define EPNUM_VENDOR_IN     0x83  // New Vendor IN Endpoint

static uint8_t const desc_fs_configuration[] = {
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

  // 1st CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, 4, EPNUM_CDC_0_NOTIF, 16, EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN, 64),

  // 2nd CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, 5, EPNUM_VENDOR_OUT, EPNUM_VENDOR_IN, 64),
  //TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_1, 4, EPNUM_CDC_1_NOTIF, 16, EPNUM_CDC_1_OUT, EPNUM_CDC_1_IN, 64),
};

#if TUD_OPT_HIGH_SPEED
// Per USB specs: high speed capable device must report device_qualifier and other_speed_configuration
static uint8_t const desc_hs_configuration[] = {
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

  // 1st CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, 4, EPNUM_CDC_0_NOTIF, 16, EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN, 512),

  // 2nd CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, 5, EPNUM_VENDOR_OUT, EPNUM_VENDOR_IN, 512),

  //TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_1, 4, EPNUM_CDC_1_NOTIF, 16, EPNUM_CDC_1_OUT, EPNUM_CDC_1_IN, 512),
};

// device qualifier is mostly similar to device descriptor since we don't change configuration based on speed
static tusb_desc_device_qualifier_t const desc_device_qualifier = {
  .bLength            = sizeof(tusb_desc_device_qualifier_t),
  .bDescriptorType    = TUSB_DESC_DEVICE_QUALIFIER,
  .bcdUSB             = USB_BCD,

  .bDeviceClass       = TUSB_CLASS_MISC,
  .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
  .bDeviceProtocol    = MISC_PROTOCOL_IAD,

  .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
  .bNumConfigurations = 0x01,
  .bReserved          = 0x00
};

// Invoked when received GET DEVICE QUALIFIER DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete.
// device_qualifier descriptor describes information about a high-speed capable device that would
// change if the device were operating at the other speed. If not highspeed capable stall this request.
uint8_t const *tud_descriptor_device_qualifier_cb(void) {
  return (uint8_t const *) &desc_device_qualifier;
}

// Invoked when received GET OTHER SEED CONFIGURATION DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
// Configuration descriptor in the other speed e.g if high speed then this is for full speed and vice versa
uint8_t const *tud_descriptor_other_speed_configuration_cb(uint8_t index) {
  (void) index;// for multiple configurations

  // if link speed is high return fullspeed config, and vice versa
  return (tud_speed_get() == TUSB_SPEED_HIGH) ? desc_fs_configuration : desc_hs_configuration;
}

#endif// highspeed

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
  (void) index; // for multiple configurations

#if TUD_OPT_HIGH_SPEED
  // Although we are highspeed, host may be fullspeed.
  return (tud_speed_get() == TUSB_SPEED_HIGH) ? desc_hs_configuration : desc_fs_configuration;
#else
  return desc_fs_configuration;
#endif
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
  STRID_MS_OS_10 = 0xEE,
};

// array of pointer to string descriptors
static char const *string_desc_arr[] = {
  (const char[]) { 0x09, 0x04 }, // 0: is supported language is English (0x0409)
  "TinyUSB",                     // 1: Manufacturer
  "TinyUSB Device",              // 2: Product
  NULL,                          // 3: Serials will use unique ID if possible
  "TinyUSB CDC",                 // 4: CDC Interface
  "TinyUSB Vendor",              // 5: Vendor class interface
};

static uint16_t _desc_str[32 + 1];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete

#define VENDOR_CODE_WCID  0x01  // Arbitrary single byte token for control transfers

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void) langid;
  size_t chr_count;

  switch ( index ) {
    case STRID_LANGID:
      memcpy(&_desc_str[1], string_desc_arr[0], 2);
      chr_count = 1;
      break;

    case STRID_SERIAL:
      chr_count = board_usb_get_serial(_desc_str + 1, 32);
      break;

    case STRID_MS_OS_10:
        // Microsoft OS 1.0 String Descriptor
        // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors
        static uint8_t const ms_os_desc[] = {
            0x12, TUSB_DESC_STRING,
            'M', 0, 'S', 0, 'F', 0, 'T', 0, '1', 0, '0', 0, '0', 0, // "MSFT100"
            VENDOR_CODE_WCID,                                      // Vendor Command Code
            0x00                                                   // Padding byte
        };
        return (uint16_t const*) ms_os_desc;
        break;

    default:
      if ( !(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) ) { return NULL; }

      const char *str = string_desc_arr[index];

      // Cap at max char
      chr_count = strlen(str);
      size_t const max_count = sizeof(_desc_str) / sizeof(_desc_str[0]) - 1; // -1 for string type
      if ( chr_count > max_count ) { chr_count = max_count; }

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

// ====================================================================
// 4. MICROSOFT EXTENDED COMPATIBILITY ID DESCRIPTOR (WCID)
// ====================================================================
typedef struct TU_ATTR_PACKED {
    // Header
    uint32_t dwLength;
    uint16_t bcdVersion;
    uint16_t wIndex;
    uint8_t  bCount;
    uint8_t  reserved[7];
    // Function section
    uint8_t  bFirstInterfaceNumber;
    uint8_t  reserved2;
    uint8_t  compatibleID[8];
    uint8_t  subCompatibleID[8];
    uint8_t  reserved3[6];
} ms_compat_id_desc_t;

static ms_compat_id_desc_t const desc_ms_compat_id = {
    .dwLength               = sizeof(ms_compat_id_desc_t),
    .bcdVersion             = 0x0100, // MS OS Descriptors v1.0
    .wIndex                 = 0x0004, // Extended Compat ID Index
    .bCount                 = 1,      // Number of interfaces configured here
    .reserved               = {0, 0, 0, 0, 0, 0, 0},
    
    .bFirstInterfaceNumber  = ITF_NUM_VENDOR,        // Matches Vendor Interface
    .reserved2              = 0x01,        // Must be 0x01
    .compatibleID           = "WINUSB\0\0", // 8 bytes padded with nulls
    .subCompatibleID        = {0,0,0,0,0,0,0,0},
    .reserved3              = {0,0,0,0,0,0}
};
_Static_assert(sizeof(ms_compat_id_desc_t) == 40, "bad WCID size");


typedef struct TU_ATTR_PACKED {
    uint32_t dwLength;              // 142
    uint16_t bcdVersion;            // 0x0100
    uint16_t wIndex;                // 0x0005
    uint16_t wCount;                // 1 property section

    // --- one property section ---
    uint32_t dwSize;                // 132
    uint32_t dwPropertyDataType;    // 1 = REG_SZ
    uint16_t wPropertyNameLength;   // 40
    uint16_t bPropertyName[20];     // "DeviceInterfaceGUID" + NUL, UTF-16LE
    uint32_t dwPropertyDataLength;  // 78
    uint16_t bPropertyData[39];     // "{3f966bd9-...}" + NUL, UTF-16LE
} ms_ext_prop_desc_t;

// Generate your OWN GUID for this — don't reuse this one in production
// (e.g. `python -c "import uuid; print(uuid.uuid4())"`)
static ms_ext_prop_desc_t const desc_ms_ext_prop = {
    .dwLength               = sizeof(ms_ext_prop_desc_t),
    .bcdVersion             = 0x0100,
    .wIndex                 = 0x0005,
    .wCount                 = 1,

    .dwSize                 = 132,
    .dwPropertyDataType     = 1, // REG_SZ
    .wPropertyNameLength    = 40,
    .bPropertyName          = u"DeviceInterfaceGUID",
    .dwPropertyDataLength   = 78,
    .bPropertyData          = u"{3f966bd9-fa04-4ec5-991c-d326973b5128}",
};

_Static_assert(sizeof(ms_ext_prop_desc_t) == 142, "bad ext props size");



// ====================================================================
// 5. VENDOR CONTROL TRANSFER CALLBACK
// ====================================================================
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request) {
    if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR ||
        request->bRequest != VENDOR_CODE_WCID) {
        return false;
    }

    if (stage == CONTROL_STAGE_SETUP) {
        // Extended Compat ID — Device recipient, wIndex 0x0004
        if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE &&
            request->wIndex == 0x0004) {
            uint16_t total_len = request->wLength;
            if (total_len > sizeof(desc_ms_compat_id)) total_len = sizeof(desc_ms_compat_id);
            return tud_control_xfer(rhport, request, (void*)(uintptr_t)&desc_ms_compat_id, total_len);
        }

        // Extended Properties — Interface recipient, wIndex 0x0005, wValue low byte = interface number
        if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_INTERFACE &&
            request->wIndex == 0x0005 &&
            (request->wValue & 0xFF) == ITF_NUM_VENDOR) {
            uint16_t total_len = request->wLength;
            if (total_len > sizeof(desc_ms_ext_prop)) total_len = sizeof(desc_ms_ext_prop);
            return tud_control_xfer(rhport, request, (void*)(uintptr_t)&desc_ms_ext_prop, total_len);
        }

        return false; // stall anything else
    }

    return true; // ACK data/status stage
}
