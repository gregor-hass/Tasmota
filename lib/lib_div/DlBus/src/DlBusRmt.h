/*
  DlBusRmt.h - DL-Bus RMT receiver for ESP32

  Copyright (C) 2024  Tasmota

  This library is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
  DL-Bus Protocol (UVR1611):
  - 488 Hz carrier frequency (~2048µs period, ~1024µs half-cycle)
  - UART 8N1: Start bit (0), 8 data bits LSB first, Stop bit (1)  
  - Bit encoding: "0" = carrier oscillating, "1" = steady HIGH (no carrier)
  - Each UART bit = 2 carrier cycles = ~4096µs
  - Sync: Long HIGH (~30ms) followed by start bit
*/

#ifndef DLBUS_RMT_H
#define DLBUS_RMT_H

#ifdef ESP32

#include <stdint.h>
#include <stddef.h>
#include "driver/rmt_rx.h"

// DL-Bus timing constants (in microseconds)
#define DLBUS_MAX_SYMBOLS             128     // Max symbols per ISR batch

// Opaque handle for the DL-Bus receiver
typedef struct DlBusReceiver* DlBusHandle;

// Callback type for decoded data
typedef void (*DlBusDataCallback_t)(const uint8_t* data, size_t len);

// Callback function (defined in driver, called by library)
extern void DlBusDataCallback(const uint8_t* data, size_t len);

// Initialize the DL-Bus RMT receiver on the specified GPIO pin
// Returns handle on success, nullptr on failure
DlBusHandle DlBusInit(int gpio_pin);

// Process received symbols from ISR queue into page buffer
// Calls DlBusProcessPage internally when a complete page is received
// Should be called periodically (e.g., every 100ms)
// Returns number of symbols in current page buffer
uint32_t DlBusProcess(DlBusHandle handle);

// Get number of symbols in current page buffer
uint32_t DlBusGetSymbolCount(DlBusHandle handle);

// Get pointer to page buffer (for decoder)
// Returns pointer to page buffer, sets length if not null
const rmt_symbol_word_t* DlBusGetPageBuffer(DlBusHandle handle, uint32_t *length);

// Get ISR call count (for debugging)
uint32_t DlBusGetIsrCount(DlBusHandle handle);

// Get the configured GPIO pin
int DlBusGetGpioPin(DlBusHandle handle);

// Check if initialized
bool DlBusIsInitialized(DlBusHandle handle);

// Deinitialize and free resources
void DlBusDeinit(DlBusHandle handle);

#endif // ESP32
#endif // DLBUS_RMT_H
