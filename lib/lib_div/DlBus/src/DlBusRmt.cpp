/*
  DlBusRmt.cpp - DL-Bus RMT receiver implementation for ESP32

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
*/

#ifdef ESP32

#include "DlBusRmt.h"
#include <Arduino.h>
#include "driver/rmt_rx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <string.h>

#define DLBUS_MIN_PULSE_NS 1000      // 1us minimum pulse
#define DLBUS_TIMEOUT_NS 5000000    // 5ms idle timeout

// Page buffer size for accumulated symbols (must be power of 2)
#define DLBUS_PAGE_BUFFER_SIZE 1024

// Enable AddLog support within a C++ library
extern void AddLog(uint32_t loglevel, PGM_P formatP, ...);
enum LoggingLevels {LOG_LEVEL_NONE, LOG_LEVEL_ERROR, LOG_LEVEL_INFO, LOG_LEVEL_DEBUG, LOG_LEVEL_DEBUG_MORE};

// Queue item: contains a batch of symbols from one RMT receive
struct DlBusRxBatch {
    rmt_symbol_word_t symbols[DLBUS_MAX_SYMBOLS];
    size_t num_symbols;
    bool is_frame_end;  // True if this batch ended due to idle timeout
};

// Internal receiver structure
struct DlBusReceiver {
    // RMT resources
    rmt_channel_handle_t rx_chan;
    QueueHandle_t receive_queue;
    rmt_symbol_word_t *rx_symbol_buf;
    size_t rx_symbol_buf_size;
    
    // Page buffer for accumulated symbols (one complete transmission)
    rmt_symbol_word_t page_buffer[DLBUS_PAGE_BUFFER_SIZE];
    uint32_t page_len;  // Number of symbols in current page
    
    // Debug counters
    volatile uint32_t isr_call_count;
    volatile uint32_t symbols_received;
    volatile uint32_t symbols_dropped;
    volatile uint32_t page_count;
    volatile uint32_t rearm_count;
    volatile uint32_t last_isr_time;  // millis() at last ISR
    
    // Status
    bool initialized;
    int gpio_pin;
};

// Global pointer for ISR access
static DlBusReceiver* g_dlbus_instance = nullptr;

// Static buffer for RMT hardware (must be in internal RAM)
static rmt_symbol_word_t g_dlbus_rx_symbol_buf[128];

// ISR callback for RMT receive done
// Called each time mem_block_symbols fills (partial) or idle timeout (frame end)
static bool IRAM_ATTR DlBusRmtRxDoneCallback(rmt_channel_handle_t channel, 
                                              const rmt_rx_done_event_data_t *edata, 
                                              void *user_data)
{
    DlBusReceiver *receiver = (DlBusReceiver *)user_data;
    if (!receiver) return false;
    
    receiver->isr_call_count++;
    receiver->last_isr_time = xTaskGetTickCount();  // Track activity

    BaseType_t high_task_wakeup = pdFALSE;

    // Process the received data
    static DlBusRxBatch batch;
    batch.num_symbols = (edata->num_symbols < DLBUS_MAX_SYMBOLS) ? edata->num_symbols : DLBUS_MAX_SYMBOLS;
    
    // Check if this batch ended due to idle (last symbol has duration1 = 0)
    //batch.is_frame_end = (edata->num_symbols > 0 && 
    //                      edata->received_symbols[edata->num_symbols - 1].duration1 == 0);
    batch.is_frame_end = edata->flags.is_last;
    
    for (size_t i = 0; i < batch.num_symbols; i++) {
        batch.symbols[i] = edata->received_symbols[i];
    }
    
    // Send the batch to the queue
    xQueueSendFromISR(receiver->receive_queue, &batch, &high_task_wakeup);

    // ALWAYS re-arm receive to prevent stalls
    // The RMT driver handles the case where receive is already active
    rmt_receive_config_t rx_conf;
    memset(&rx_conf, 0, sizeof(rx_conf));
    rx_conf.signal_range_min_ns = DLBUS_MIN_PULSE_NS;
    rx_conf.signal_range_max_ns = DLBUS_TIMEOUT_NS;
    rx_conf.flags.en_partial_rx = true;
    
    esp_err_t ret = rmt_receive(receiver->rx_chan, receiver->rx_symbol_buf, 
                                 receiver->rx_symbol_buf_size, &rx_conf);
    if (ret == ESP_OK) {
        receiver->rearm_count++;
    }

    return (high_task_wakeup == pdTRUE);
}

// Public API implementation

DlBusHandle DlBusInit(int gpio_pin) {
    if (gpio_pin < 0 || gpio_pin >= GPIO_NUM_MAX) {
        AddLog(LOG_LEVEL_ERROR, PSTR("DLBUS: Invalid GPIO pin %d"), gpio_pin);
        return nullptr;
    }
    
    // Allocate receiver structure
    DlBusReceiver *receiver = new DlBusReceiver();
    if (!receiver) {
        AddLog(LOG_LEVEL_ERROR, PSTR("DLBUS: Failed to allocate receiver"));
        return nullptr;
    }
    
    memset(receiver, 0, sizeof(DlBusReceiver));
    receiver->gpio_pin = gpio_pin;
    receiver->page_len = 0;
    
    // Use static buffer for RMT symbols (must be in internal RAM)
    receiver->rx_symbol_buf = g_dlbus_rx_symbol_buf;
    receiver->rx_symbol_buf_size = sizeof(g_dlbus_rx_symbol_buf);
    
    AddLog(LOG_LEVEL_INFO, PSTR("DLBUS: Initializing on GPIO%d"), gpio_pin);
    AddLog(LOG_LEVEL_DEBUG, PSTR("DLBUS: Ring buffer: %u symbols (%u bytes)"), 
           DLBUS_PAGE_BUFFER_SIZE, sizeof(receiver->page_buffer));
    
    // Configure RMT RX channel
    rmt_rx_channel_config_t rx_chan_config;
    memset(&rx_chan_config, 0, sizeof(rx_chan_config));
    rx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rx_chan_config.resolution_hz = 1 * 1000 * 1000;  // 1 MHz = 1us resolution
    rx_chan_config.mem_block_symbols = 64;           // ESP32-C3 has limited RMT memory
    rx_chan_config.gpio_num = static_cast<gpio_num_t>(gpio_pin);
    
    esp_err_t ret = rmt_new_rx_channel(&rx_chan_config, &receiver->rx_chan);
    if (ret != ESP_OK) {
        AddLog(LOG_LEVEL_ERROR, PSTR("DLBUS: rmt_new_rx_channel failed: %d %s"), ret, esp_err_to_name(ret));
        delete receiver;
        return nullptr;
    }
    AddLog(LOG_LEVEL_DEBUG, PSTR("DLBUS: RMT channel created"));
    
    // Create receive queue (small, just for ISR->task handoff)
    receiver->receive_queue = xQueueCreate(16, sizeof(DlBusRxBatch));
    if (!receiver->receive_queue) {
        AddLog(LOG_LEVEL_ERROR, PSTR("DLBUS: Failed to create queue"));
        rmt_del_channel(receiver->rx_chan);
        delete receiver;
        return nullptr;
    }
    
    // Register ISR callback
    rmt_rx_event_callbacks_t cbs;
    memset(&cbs, 0, sizeof(cbs));
    cbs.on_recv_done = DlBusRmtRxDoneCallback;
    ret = rmt_rx_register_event_callbacks(receiver->rx_chan, &cbs, receiver);
    if (ret != ESP_OK) {
        AddLog(LOG_LEVEL_ERROR, PSTR("DLBUS: Failed to register callbacks: %d %s"), ret, esp_err_to_name(ret));
        vQueueDelete(receiver->receive_queue);
        rmt_del_channel(receiver->rx_chan);
        delete receiver;
        return nullptr;
    }
    
    // Prepare receive config with partial receive enabled
    rmt_receive_config_t rx_conf;
    memset(&rx_conf, 0, sizeof(rx_conf));
    rx_conf.signal_range_min_ns = DLBUS_MIN_PULSE_NS;
    rx_conf.signal_range_max_ns = DLBUS_TIMEOUT_NS;
    rx_conf.flags.en_partial_rx = true;  // Enable continuous partial receive

    // Enable channel
    ret = rmt_enable(receiver->rx_chan);
    if (ret != ESP_OK) {
        AddLog(LOG_LEVEL_ERROR, PSTR("DLBUS: rmt_enable failed: %d %s"), ret, esp_err_to_name(ret));
        vQueueDelete(receiver->receive_queue);
        rmt_del_channel(receiver->rx_chan);
        delete receiver;
        return nullptr;
    }
    
    // Start receiving
    ret = rmt_receive(receiver->rx_chan, receiver->rx_symbol_buf, receiver->rx_symbol_buf_size, &rx_conf);
    if (ret != ESP_OK) {
        AddLog(LOG_LEVEL_ERROR, PSTR("DLBUS: rmt_receive failed: %d %s"), ret, esp_err_to_name(ret));
        rmt_disable(receiver->rx_chan);
        vQueueDelete(receiver->receive_queue);
        rmt_del_channel(receiver->rx_chan);
        delete receiver;
        return nullptr;
    }
    
    receiver->initialized = true;
    g_dlbus_instance = receiver;
    
    AddLog(LOG_LEVEL_INFO, PSTR("DLBUS: Initialized, waiting for signal"));
    return receiver;
}

uint32_t nFramingErrors = 0;
void CaptureBit (uint8_t const level, int8_t& current_bit, uint8_t& byte,uint8_t*& pWritePointer){
    if(current_bit == -1){
        // waiting for start
        if(level == 0){
            current_bit = 0;
            byte = 0;
        }
    } else if(current_bit >=0 && current_bit <=7){
        // data bits
        if(level == 1){
            byte |= (level << current_bit);
        }else{
            byte &= ~(1 << current_bit);
        }
        current_bit++;
    } else if(current_bit == 8){
        *pWritePointer = byte;
        pWritePointer++;
        // stop bit
        if(level == 1){
            // valid stop
        }else{
            // framing error
            nFramingErrors++;
        }
        current_bit = -1; // reset for next byte
    }
}
/*
void AdvanceTime(uint32_t& current_time, uint32_t& next_capture_time) {
    current_time += FULL_BIT_US;
    next_capture_time += FULL_BIT_US;
}
*/

// Process a complete page of symbols (called when idle timeout triggers)
static void DlBusProcessPage(DlBusReceiver *receiver) {
    if (receiver->page_len == 0) return;    

    // Debug: track time-to-next-capture at intervals to detect drift
    int16_t debug_offsets[20];  // Store first 20 offset samples
    uint32_t debug_offset_count = 0;
    
    const uint32_t FULL_BIT_US = 2000;
    const uint32_t HALF_BIT_US = 1000;
    const uint32_t QUARTER_BIT_US = 500;

    uint8_t decoded_bytes[256];
    uint8_t* pWritePointer = &decoded_bytes[0];
    
    uint8_t constructing_byte = 0;
    int8_t current_bit = -1;  // -1 = waiting for start, 0-7 = data, 8 = stop

    const uint8_t syncbits_target = 16;
    uint8_t syncbits_captured = 0;

    uint32_t current_time = 0;
    uint32_t next_capture_time = QUARTER_BIT_US;  // Sample at CENTER of each bit

    uint32_t time_accumulated = 0;
    for(int j = 0; j< receiver->page_len; j++){
        rmt_symbol_word_t *sym = &receiver->page_buffer[j];
        uint32_t d0 = sym->duration0;
        uint32_t d1 = sym->duration1;
        time_accumulated += d0 + d1;
    }

    uint32_t i = 0;
    for ( ;i < receiver->page_len && pWritePointer < (decoded_bytes+sizeof(decoded_bytes)); i++) {
        rmt_symbol_word_t *sym = &receiver->page_buffer[i];
        
        // Process duration0/level0
        uint32_t d0 = sym->duration0;
        uint8_t l0 = sym->level0;
        
        if (d0 > 0) {
            uint32_t end_time = current_time + d0;
            
            // Process all capture points within this duration
            while (next_capture_time <= end_time && pWritePointer < (decoded_bytes+sizeof(decoded_bytes))) {
                if (syncbits_captured < syncbits_target) {
                    // Still looking for sync (16 consecutive HIGH bits)
                    if (l0 == 1) {
                        syncbits_captured++;
                    } else {
                        syncbits_captured = 0;
                    }
                } else {
                    // Decoding data
                    CaptureBit(l0, current_bit, constructing_byte, pWritePointer);
                }
                next_capture_time += FULL_BIT_US;
            }
            uint32_t estimated_next_capture_time = next_capture_time;  // downgrade to estimation

            current_time = end_time;

            uint32_t wanted_capture_quarter_periods = 0;
            wanted_capture_quarter_periods = (estimated_next_capture_time - current_time) / QUARTER_BIT_US;
            if(((estimated_next_capture_time - current_time) % QUARTER_BIT_US) >= (QUARTER_BIT_US/2)) {
                // there is one more wanted quarter period, but we have already drifted
                wanted_capture_quarter_periods++;
            }
            next_capture_time = current_time + wanted_capture_quarter_periods * QUARTER_BIT_US; // calculate next real capture time

            // Record offset for debugging (sample every ~50 symbols)
            if (debug_offset_count < 20 && (i % 50) == 0) {
                debug_offsets[debug_offset_count++] = (int16_t)(estimated_next_capture_time - next_capture_time);
            }
            
        }
        
        // Process duration1/level1
        uint32_t d1 = sym->duration1;
        uint8_t l1 = sym->level1;
        
        if (d1 > 0) {
            uint32_t end_time = current_time + d1;
            
            while (next_capture_time <= end_time && pWritePointer < (decoded_bytes+sizeof(decoded_bytes))) {
                if (syncbits_captured < syncbits_target) {
                    if (l1 == 1) {
                        syncbits_captured++;
                    } else {
                        syncbits_captured = 0;
                    }
                } else {
                    CaptureBit(l1, current_bit, constructing_byte, pWritePointer);
                }

                next_capture_time += FULL_BIT_US;
            }
            uint32_t estimated_next_capture_time = next_capture_time;  // downgrade to estimation
            
            
            current_time = end_time;
            
            uint32_t wanted_capture_quarter_periods = 0;
            wanted_capture_quarter_periods = (estimated_next_capture_time - current_time) / QUARTER_BIT_US;
            if(((estimated_next_capture_time - current_time) % QUARTER_BIT_US) >= (QUARTER_BIT_US/2)) {
                // there is one more wanted quarter period, but we have already drifted
                wanted_capture_quarter_periods++;
            }
            next_capture_time = current_time + wanted_capture_quarter_periods * QUARTER_BIT_US; // calculate next real capture time
        }
    }

    if(current_bit == 8){
        // last stop bit will be missed, as it has time 0 (timeout last bit)
        CaptureBit(1, current_bit, constructing_byte, pWritePointer);
    }
    
    receiver->page_count++;

    // Build debug string with offsets
    char offset_str[120];
    char *p = offset_str;
    for (uint32_t k = 0; k < debug_offset_count && p < offset_str + sizeof(offset_str) - 8; k++) {
        p += sprintf(p, "%d ", debug_offsets[k]);
    }
    if (p > offset_str) *(p-1) = '\0';
    else offset_str[0] = '\0';

    AddLog(LOG_LEVEL_DEBUG, PSTR("DLBUS: Page %u: %u sym, %.1fms total, parsed %u, %.1fms | FE:%u | CurBit: %d | offsets: %s"),
           receiver->page_count, receiver->page_len, time_accumulated/1000.0f, i, current_time/1000.0f, nFramingErrors, current_bit, offset_str);

    // Calculate decoded length
    size_t decoded_len = pWritePointer - decoded_bytes;
    
    // Call the callback with decoded data (if any)
    if (decoded_len > 0) {
        DlBusDataCallback(decoded_bytes, decoded_len);
    }
    
    receiver->page_len = 0;
}

uint32_t DlBusProcess(DlBusHandle handle) {
    if (!handle || !handle->initialized) return 0;
    
    DlBusRxBatch batch;
    static uint32_t batches_processed = 0;
    
    // Process all queued batches and add to page buffer
    while (xQueueReceive(handle->receive_queue, &batch, 0) == pdPASS) {
        batches_processed++;
        
        // Add symbols to page buffer
        for (size_t i = 0; i < batch.num_symbols; i++) {
            if (handle->page_len < DLBUS_PAGE_BUFFER_SIZE) {
                handle->page_buffer[handle->page_len++] = batch.symbols[i];
                handle->symbols_received++;
            } else {
                handle->symbols_dropped++;
            }
        }
        
        // Check if page is complete (idle timeout triggered)
        if (batch.is_frame_end) {
            if (handle->page_len > 0) {
                AddLog(LOG_LEVEL_DEBUG, PSTR("DLBUS: Frame end, %u batches, %u symbols, dropped %u"),
                    batches_processed, handle->page_len, handle->symbols_dropped);
                DlBusProcessPage(handle);
            }

            // reset and handle next batch as new page
            batches_processed = 0;
            handle->page_len = 0;
            handle->symbols_received = 0;
            handle->symbols_dropped = 0;
        }
    }
    
    // Only process if we got a frame end AND we have enough symbols
    // This prevents processing partial data if DlBusProcess is called too frequently
    
    
    // Watchdog: if no ISR activity for 5 seconds but we expect signal, try to restart
    /*
    static uint32_t last_check = 0;
    uint32_t now = xTaskGetTickCount();
    if (now - last_check > pdMS_TO_TICKS(5000)) {
        last_check = now;
        uint32_t idle_time = now - handle->last_isr_time;
        
        if (idle_time > pdMS_TO_TICKS(5000) && handle->isr_call_count > 0) {
            // Had activity before but now stalled - try to restart
            AddLog(LOG_LEVEL_INFO, PSTR("DLBUS: Stall detected, restarting RMT (idle %ums, isr=%u, rearm=%u)"),
                   (idle_time * 1000) / configTICK_RATE_HZ,
                   handle->isr_call_count, handle->rearm_count);
            
            // Try to restart receive
            rmt_receive_config_t rx_conf;
            memset(&rx_conf, 0, sizeof(rx_conf));
            rx_conf.signal_range_min_ns = DLBUS_MIN_PULSE_NS;
            rx_conf.signal_range_max_ns = DLBUS_TIMEOUT_NS;
            rx_conf.flags.en_partial_rx = true;
            
            esp_err_t ret = rmt_receive(handle->rx_chan, handle->rx_symbol_buf, 
                                        handle->rx_symbol_buf_size, &rx_conf);
            if (ret != ESP_OK) {
                AddLog(LOG_LEVEL_ERROR, PSTR("DLBUS: Restart failed: %d %s"), ret, esp_err_to_name(ret));
                
                // More aggressive recovery: disable and re-enable
                rmt_disable(handle->rx_chan);
                ret = rmt_enable(handle->rx_chan);
                if (ret == ESP_OK) {
                    ret = rmt_receive(handle->rx_chan, handle->rx_symbol_buf, 
                                     handle->rx_symbol_buf_size, &rx_conf);
                    AddLog(LOG_LEVEL_INFO, PSTR("DLBUS: Full restart %s"), 
                           ret == ESP_OK ? "OK" : "FAILED");
                }
            } else {
                AddLog(LOG_LEVEL_INFO, PSTR("DLBUS: Restart OK"));
            }
        }
    }
    */
    
    return handle->page_len;
}

// Get symbols in current page buffer
uint32_t DlBusGetSymbolCount(DlBusHandle handle) {
    if (!handle) return 0;
    return handle->page_len;
}

// Get pointer to page buffer (for decoder)
const rmt_symbol_word_t* DlBusGetPageBuffer(DlBusHandle handle, uint32_t *length) {
    if (!handle) return nullptr;
    if (length) *length = handle->page_len;
    return handle->page_buffer;
}

uint32_t DlBusGetIsrCount(DlBusHandle handle) {
    return handle ? handle->isr_call_count : 0;
}

int DlBusGetGpioPin(DlBusHandle handle) {
    return handle ? handle->gpio_pin : -1;
}

bool DlBusIsInitialized(DlBusHandle handle) {
    return handle && handle->initialized;
}

void DlBusDeinit(DlBusHandle handle) {
    if (!handle) return;
    
    if (handle->rx_chan) {
        rmt_disable(handle->rx_chan);
        rmt_del_channel(handle->rx_chan);
    }
    
    if (handle->receive_queue) {
        vQueueDelete(handle->receive_queue);
    }
    
    if (g_dlbus_instance == handle) {
        g_dlbus_instance = nullptr;
    }
    
    delete handle;
    AddLog(LOG_LEVEL_INFO, PSTR("DLBUS: Deinitialized"));
}

#endif // ESP32
