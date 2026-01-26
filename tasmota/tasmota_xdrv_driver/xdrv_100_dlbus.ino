/*
  xdrv_100_dlbus.ino - DL-Bus driver for Tasmota
  
  Receives DL-Bus messages from UVR1611 solar controllers via the ESP32 RMT peripheral.
  
  DL-Bus Protocol:
  - 488 Hz carrier frequency (~2048µs period, ~1024µs half-cycle)
  - UART 8N1: Start bit (0), 8 data bits LSB first, Stop bit (1)  
  - Bit encoding: "0" = carrier oscillating, "1" = steady HIGH (no carrier)
  - Each UART bit = 2 carrier cycles = ~4096µs
  - Sync: Long HIGH (~30ms) followed by start bit
  
  Configuration:
  - Enable with #define USE_XDRV_100_DLBUS in user_config_override.h
  - Assign GPIO_DLBUS_RX to the input pin in Tasmota configuration
*/

#ifdef ESP32
#ifdef USE_XDRV_100_DLBUS

#define XDRV_100 100

#include "DlBusRmt.h"

static DlBusHandle dlbus_handle = nullptr;

static void DlBusDriverInit(void) {
  if (!PinUsed(GPIO_DLBUS_RX)) {
    AddLog(LOG_LEVEL_INFO, PSTR("DLBUS: No pin configured"));
    return;
  }

  int32_t pin = Pin(GPIO_DLBUS_RX);
  dlbus_handle = DlBusInit(pin);
  
  if (dlbus_handle) {
    AddLog(LOG_LEVEL_INFO, PSTR("DLBUS: Driver started on GPIO%d"), pin);
  }
}

static void DlBusDriverEverySecond(void) {
  if (!DlBusIsInitialized(dlbus_handle)) {
    return;
  }

  // Process any queued data first
  DlBusProcess(dlbus_handle);

  // Debug: show activity every second
  static uint32_t last_isr_count = 0;
  static uint32_t seconds = 0;
  seconds++;
  
  uint32_t isr_count = DlBusGetIsrCount(dlbus_handle);
  uint32_t symbol_count = DlBusGetSymbolCount(dlbus_handle);
  int gpio_pin = DlBusGetGpioPin(dlbus_handle);
  int gpio_state = digitalRead(gpio_pin);
  
  // Log every 5 seconds or when ISR count changes
  if ((seconds % 5) == 0 || isr_count != last_isr_count) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("DLBUS: t=%u isr=%u buf=%u gpio=%d"), 
           seconds, isr_count, symbol_count, gpio_state);
    last_isr_count = isr_count;
  }
}

bool Xdrv100(uint32_t function) {
  switch (function) {
    case FUNC_INIT:
      DlBusDriverInit();
      break;

    case FUNC_EVERY_SECOND:
      DlBusDriverEverySecond();
      break;
  }
  return false;
}

#endif  // USE_XDRV_100_DLBUS
#endif  // ESP32
