/*
  xdrv_100_dlbus.ino - DL-Bus driver for Tasmota
  
  Receives DL-Bus messages from UVR1611 solar controllers via the ESP32 RMT peripheral.
  
  DL-Bus Protocol:
  - 488 Hz carrier frequency (~2048µs period, ~1024µs half-cycle)
  - UART 8N1: Start bit (0), 8 data bits LSB first, Stop bit (1)  
  - Bit encoding: "0" = HIGH-LOW, "1" = LOW-HIGH (Manchester-like)
  - Each UART bit = 2 half-cycles = ~2000µs
  - Sync: Long HIGH (~30ms) followed by start bit
  
  Configuration:
  - Enable with #define USE_XDRV_100_DLBUS in user_config_override.h
  - Assign GPIO_DLBUS_RX to the input pin in Tasmota configuration
  
  Commands:
  - DlBusUdp <ip> <port>  - Set UDP destination (e.g., DlBusUdp 192.168.1.100 5000)
  - DlBusUdp              - Show current UDP settings
  - DlBusUdp 0            - Disable UDP forwarding
*/

#ifdef ESP32
#ifdef USE_XDRV_100_DLBUS

#define XDRV_100 100

#include "DlBusRmt.h"
#include <WiFiUdp.h>

// Command name
const char kDlBusCommands[] PROGMEM = "DlBus|Udp";

// UDP configuration (stored in RAM, persisted to UFS when available)
// Runtime values used by the driver
static IPAddress dlbus_udp_ip(0, 0, 0, 0);
static uint16_t dlbus_udp_port = 0;
static WiFiUDP dlbus_udp;

// Persistent driver settings (UFS key = "drvset100")
typedef struct DlBusSettings_t {
  uint32_t crc32;          // must be first for GetCfgCrc32()
  uint32_t udp_ip;         // IPv4 as uint32
  uint16_t udp_port;       // UDP port
  uint8_t  _pad;           // padding for alignment
} DlBusSettings_t;

#ifdef USE_UFILESYS
#define XDRV_100_KEY "drvset100"

static DlBusSettings_t DlBusSettings; // stored representation

bool DlBusLoadData(void) {
  char key[] = XDRV_100_KEY;
  String json = UfsJsonSettingsRead(key);
  if (json.length() == 0) { return false; }

  JsonParser parser((char*)json.c_str());
  JsonParserObject root = parser.getRootObject();
  if (!root) { return false; }

  DlBusSettings.crc32 = root.getUInt(PSTR("Crc"), DlBusSettings.crc32);
  DlBusSettings.udp_ip = root.getUInt(PSTR("Ip"), DlBusSettings.udp_ip);
  DlBusSettings.udp_port = root.getUInt(PSTR("Port"), DlBusSettings.udp_port);
  return true;
}

bool DlBusSaveData(void) {
  Response_P(PSTR("{\"" XDRV_100_KEY "\":{\"Crc\":%u,\"Ip\":%u,\"Port\":%u}}"),
             DlBusSettings.crc32, DlBusSettings.udp_ip, DlBusSettings.udp_port);
  return UfsJsonSettingsWrite(ResponseData());
}

void DlBusDeleteData(void) {
  char key[] = XDRV_100_KEY;
  UfsJsonSettingsDelete(key);
}
#endif // USE_UFILESYS

// Load persistent settings (called at driver init)
void DlBusSettingsLoad(bool erase) {
  // defaults
  DlBusSettings.udp_ip = 0;
  DlBusSettings.udp_port = 0;

#ifdef USE_UFILESYS
  if (erase) {
    DlBusDeleteData();
  } else if (DlBusLoadData()) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("DLBUS: Loaded cfg from file"));
  } else {
    AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("DLBUS: No cfg in file system or file system not ready"));
  }
#endif

  // apply to runtime vars
  if (DlBusSettings.udp_ip != 0 && DlBusSettings.udp_port != 0) {
    dlbus_udp_ip = IPAddress((uint32_t)DlBusSettings.udp_ip);
    dlbus_udp_port = DlBusSettings.udp_port;
  }
}

// Save persistent settings (called after changes)
void DlBusSettingsSave(void) {
#ifdef USE_UFILESYS
  DlBusSettings.udp_ip = (uint32_t)dlbus_udp_ip;
  DlBusSettings.udp_port = dlbus_udp_port;
  DlBusSettings.crc32 = GetCfgCrc32((uint8_t*)&DlBusSettings + 4, sizeof(DlBusSettings) - 4);
  DlBusSaveData();
#else
  (void)dlbus_udp_ip; (void)dlbus_udp_port; // no-op when filesystem isn't available
#endif
}


static DlBusHandle dlbus_handle = nullptr;

// Forward declaration
void CmndDlBusUdp(void);

// Command function pointer array
void (* const DlBusCommand[])(void) PROGMEM = {
  &CmndDlBusUdp
};

struct UVR1611Data {
  uint8_t cDeviceType;
  uint8_t cDeviceTypeInverted;
  uint8_t cDontCare;
  uint8_t cTimeMinute;
  uint8_t cTimeHour;
  uint8_t cTimeDay;
  uint8_t cTimeMonth;
  uint8_t cTimeYear;
  uint8_t cSensor1[2];
  uint8_t cSensor2[2];
  uint8_t cSensor3[2];
  uint8_t cSensor4[2];
  uint8_t cSensor5[2];
  uint8_t cSensor6[2];
  uint8_t cSensor7[2];
  uint8_t cSensor8[2];
  uint8_t cSensor9[2];
  uint8_t cSensor10[2];
  uint8_t cSensor11[2];
  uint8_t cSensor12[2];
  uint8_t cSensor13[2];
  uint8_t cSensor14[2];
  uint8_t cSensor15[2];
  uint8_t cSensor16[2];
  uint8_t cOutputs[2];
  uint8_t cRevolutionsA1;
  uint8_t cRevolutionsA2;
  uint8_t cRevolutionsA6;
  uint8_t cRevolutionsA7;
  uint8_t cThermicEnergy;
  uint8_t cMomentaryPower1[4];
  uint8_t cKWh1[2];
  uint8_t cMWh1[2];
  uint8_t cMomentaryPower2[4];
  uint8_t cKWh2[2];
  uint8_t cMWh2[2];
  uint8_t cChecksum;
};

// JSON format string for UVR1611 data
const char UVR1611_JSON_FMT[] PROGMEM = 
  "{\"Time\":\"%02u:%02u %02u.%02u.%02u\","
  "\"%s1\":%.1f%s,\"%s2\":%.1f%s,\"%s3\":%.1f%s,\"%s4\":%.1f%s,"
  "\"%s5\":%.1f%s,\"%s6\":%.1f%s,\"%s7\":%.1f%s,\"%s8\":%.1f%s,"
  "\"%s9\":%.1f%s,\"%s10\":%.1f%s,\"%s11\":%.1f%s,\"%s12\":%.1f%s,"
  "\"%s13\":%.1f%s,\"%s14\":%.1f%s,\"%s15\":%.1f%s,\"%s16\":%.1f%s,"
  "\"A1\":%u,\"A2\":%u,\"A3\":%u,\"A4\":%u,\"A5\":%u,\"A6\":%u,\"A7\":%u,"
  "\"A8\":%u,\"A9\":%u,\"A10\":%u,\"A11\":%u,\"A12\":%u,\"A13\":%u,"
  "\"Power1\":%.2f,\"Power2\":%.2f}";

// Helper to convert 2-byte sensor value to float (little-endian, 0.1 resolution)

const char* SensorToNameStr(const uint8_t* bytes) {
  uint8_t cUnitIndex = (bytes[1] >> 4) & 0x7;
  static const char* name_strings[] = {
    "Unused",
    "Dig",
    "Temp",
    "Flow",
    "Unused2",
    "Unused3",
    "Rad",
    "RoomT"
  };
  if (cUnitIndex < sizeof(name_strings) / sizeof(name_strings[0])) {
    return name_strings[cUnitIndex];
  }
  return "unknown";
}
const char* SensorToUnitStr(const uint8_t* bytes) {
  uint8_t cUnitIndex = (bytes[1] >> 4) & 0x7;
  static const char* unit_strings[] = {
    "",
    "",
    "C",
    "l/h",
    "",
    "",
    "W/(m*m)",
    "C"
  };
  if (cUnitIndex < sizeof(unit_strings) / sizeof(unit_strings[0])) {
    return unit_strings[cUnitIndex];
  }
  return "unknown";
}

static float SensorToFloat(const uint8_t* bytes) {
  
  uint8_t cUnitIndex = (bytes[1] >> 4) & 0x7;

  uint8_t cHigh;
  if(bytes[1] & 0x80){
    cHigh = bytes[1] | 0xF0; // recover 2s complement
  }else{
    cHigh = bytes[1] & 0x0F;
  }

  int16_t raw = (int16_t)(bytes[0] | (cHigh << 8));

  switch(cUnitIndex) {
    case 0: // Unused
    case 4: // Unused
    case 5: // Unused
      return 0.0f;
    case 1: // Digital
      if(bytes[1] & (1<<7)){
        return 1.0f;
      }else{
        return 0.0f;
      }
    case 6: // Radiation
      return raw;
    case 2: // Temperature
    case 7: // Room Temperature
      return raw / 10.0f;
    case 3: // Flow
      return raw * 4.0f;
  }
  return raw / 10.0f;
}

// Helper to convert 4-byte power value to float (in kW)
// bytes[0] = low_low, bytes[1] = low_high, bytes[2] = high_low, bytes[3] = high_high
static float PowerToFloat(const uint8_t* bytes) {
  uint8_t low_low = bytes[0];
  uint8_t low_high = bytes[1];
  uint8_t high_low = bytes[2];
  uint8_t high_high = bytes[3];
  
  float power;
  if (high_high & (1<<7)) {
    // Negative value (highest bit = 1)
    power = (10.0f*(65536.0f*high_high + 256.0f*high_low + low_high) + (low_low*10.0f/256.0f)) / 100.0f;
    
  } else {
    // Positive value
    power = (10.0f*(65536.0f*high_high + 256.0f*high_low + low_high - 65536.0f) - (low_low*10.0f/256.0f)) / 100.0f;    
  }
  return power;
}

// Send data via UDP as JSON
static void DlBusSendUdp(const uint8_t* data, size_t len) {
  if (dlbus_udp_port == 0 || dlbus_udp_ip == IPAddress(0, 0, 0, 0)) {
    return;  // UDP not configured
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    return;  // No WiFi
  }
  
  // Check minimum length for UVR1611 data
  if (len < (sizeof(UVR1611Data)-1)) {
    AddLog(LOG_LEVEL_ERROR, PSTR("DLBUS: Data too short for UVR1611 (%u < %u)"), len, sizeof(UVR1611Data));
    return;
  }
  
  const UVR1611Data* uvr = (const UVR1611Data*)data;

  AddLog(LOG_LEVEL_ERROR, PSTR("DLBUS: got %.1f from [%02X | %02X | %02X | %02X]"), PowerToFloat(uvr->cMomentaryPower1), uvr->cMomentaryPower1[0], uvr->cMomentaryPower1[1], uvr->cMomentaryPower1[2], uvr->cMomentaryPower1[3]);
  AddLog(LOG_LEVEL_ERROR, PSTR("DLBUS: got %.1f from [%02X | %02X | %02X | %02X]"), PowerToFloat(uvr->cMomentaryPower2), uvr->cMomentaryPower2[0], uvr->cMomentaryPower2[1], uvr->cMomentaryPower2[2], uvr->cMomentaryPower2[3]);

  AddLog(LOG_LEVEL_ERROR, PSTR("DLBUS: 15 got %.1f from [ %02X | %02X]"), SensorToFloat(uvr->cSensor15), uvr->cSensor15[0], uvr->cSensor15[1]);   
  AddLog(LOG_LEVEL_ERROR, PSTR("DLBUS: 16 got %.1f from [ %02X | %02X]"), SensorToFloat(uvr->cSensor16), uvr->cSensor16[0], uvr->cSensor16[1]); 
 
  // Extract output states from cOutputs (A1-A13, bits 0-12)
  uint16_t outputs = uvr->cOutputs[0] | (uvr->cOutputs[1] << 8);

  // Format JSON packet
  char json_buf[600];
  snprintf_P(json_buf, sizeof(json_buf), UVR1611_JSON_FMT,
     uvr->cTimeHour&0x1F, uvr->cTimeMinute, uvr->cTimeDay, uvr->cTimeMonth, uvr->cTimeYear,
    SensorToNameStr(uvr->cSensor1), SensorToFloat(uvr->cSensor1), SensorToUnitStr(uvr->cSensor1), SensorToNameStr(uvr->cSensor2),  SensorToFloat(uvr->cSensor2),    SensorToUnitStr(uvr->cSensor2), 
    SensorToNameStr(uvr->cSensor3), SensorToFloat(uvr->cSensor3), SensorToUnitStr(uvr->cSensor3), SensorToNameStr(uvr->cSensor4),  SensorToFloat(uvr->cSensor4),    SensorToUnitStr(uvr->cSensor4), 
    SensorToNameStr(uvr->cSensor5), SensorToFloat(uvr->cSensor5), SensorToUnitStr(uvr->cSensor5), SensorToNameStr(uvr->cSensor6),  SensorToFloat(uvr->cSensor6),    SensorToUnitStr(uvr->cSensor6), 
    SensorToNameStr(uvr->cSensor7), SensorToFloat(uvr->cSensor7), SensorToUnitStr(uvr->cSensor7), SensorToNameStr(uvr->cSensor8),  SensorToFloat(uvr->cSensor8),    SensorToUnitStr(uvr->cSensor8), 
    SensorToNameStr(uvr->cSensor9), SensorToFloat(uvr->cSensor9), SensorToUnitStr(uvr->cSensor9), SensorToNameStr(uvr->cSensor10), SensorToFloat(uvr->cSensor10),   SensorToUnitStr(uvr->cSensor10),
    SensorToNameStr(uvr->cSensor11),SensorToFloat(uvr->cSensor11),SensorToUnitStr(uvr->cSensor11),SensorToNameStr(uvr->cSensor12), SensorToFloat(uvr->cSensor12),   SensorToUnitStr(uvr->cSensor12),
    SensorToNameStr(uvr->cSensor13),SensorToFloat(uvr->cSensor13),SensorToUnitStr(uvr->cSensor13),SensorToNameStr(uvr->cSensor14), SensorToFloat(uvr->cSensor14),   SensorToUnitStr(uvr->cSensor14),
    SensorToNameStr(uvr->cSensor15),SensorToFloat(uvr->cSensor15),SensorToUnitStr(uvr->cSensor15),SensorToNameStr(uvr->cSensor16), SensorToFloat(uvr->cSensor16),   SensorToUnitStr(uvr->cSensor16),
    (outputs >> 0) & 1, (outputs >> 1) & 1, (outputs >> 2) & 1, (outputs >> 3) & 1,
    (outputs >> 4) & 1, (outputs >> 5) & 1, (outputs >> 6) & 1, (outputs >> 7) & 1,
    (outputs >> 8) & 1, (outputs >> 9) & 1, (outputs >> 10) & 1, (outputs >> 11) & 1, (outputs >> 12) & 1,
    PowerToFloat(uvr->cMomentaryPower1), PowerToFloat(uvr->cMomentaryPower2));
  
  dlbus_udp.beginPacket(dlbus_udp_ip, dlbus_udp_port);
  dlbus_udp.write((const uint8_t*)json_buf, strlen(json_buf));
  dlbus_udp.endPacket();
}

// Console command: DlBusUdp [ip] [port]
void CmndDlBusUdp(void) {
  if (XdrvMailbox.data_len > 0) {
    // Parse arguments
    char* args = XdrvMailbox.data;
    
    // Check for disable command
    if (strcmp(args, "0") == 0) {
      dlbus_udp_ip = IPAddress(0, 0, 0, 0);
      dlbus_udp_port = 0;
      // Remove persisted configuration
      DlBusDeleteData();
      ResponseCmndDone();
      return;
    }
    
    // Parse IP and port
    char* ip_str = strtok(args, " ");
    char* port_str = strtok(nullptr, " ");
    
    if (ip_str && port_str) {
      IPAddress new_ip;
      if (new_ip.fromString(ip_str)) {
        uint16_t new_port = atoi(port_str);
        if (new_port > 0) {
          dlbus_udp_ip = new_ip;
          dlbus_udp_port = new_port;
          // Persist new target
          DlBusSettingsSave();
          AddLog(LOG_LEVEL_INFO, PSTR("DLBUS: UDP target set to %s:%d"), 
                 dlbus_udp_ip.toString().c_str(), dlbus_udp_port);
          ResponseCmndDone();
          return;
        }
      }
    }
    ResponseCmndError();
  } else {
    // Show current settings
    if (dlbus_udp_port > 0) {
      ResponseCmndChar(dlbus_udp_ip.toString().c_str());
      ResponseAppend_P(PSTR(":%d"), dlbus_udp_port);
    } else {
      ResponseCmndChar("disabled");
    }
  }
}

static void DlBusDriverInit(void) {
  // Load persistent configuration first
  DlBusSettingsLoad(0);

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

static void DlBusDriverEvery50ms(void) {
  if (!DlBusIsInitialized(dlbus_handle)) {
    return;
  }

  // Process any queued data frequently to keep up with ISR
  DlBusProcess(dlbus_handle);
}

static void DlBusDriverEverySecond(void) {
  if (!DlBusIsInitialized(dlbus_handle)) {
    return;
  }

  // Debug: show activity every 5 seconds
  static uint32_t last_isr_count = 0;
  static uint32_t seconds = 0;
  seconds++;
  
  uint32_t isr_count = DlBusGetIsrCount(dlbus_handle);
  uint32_t symbol_count = DlBusGetSymbolCount(dlbus_handle);
  int gpio_pin = DlBusGetGpioPin(dlbus_handle);
  int gpio_state = digitalRead(gpio_pin);
  
  // Log every 5 seconds or when ISR count changes
  if ((seconds % 5) == 0 || isr_count != last_isr_count) {
    //AddLog(LOG_LEVEL_DEBUG, PSTR("DLBUS: t=%u isr=%u buf=%u gpio=%d udp=%s:%d"), 
    //       seconds, isr_count, symbol_count, gpio_state,
    //       dlbus_udp_ip.toString().c_str(), dlbus_udp_port);
    last_isr_count = isr_count;
  }
}

// Callback when decoded data is available (called from DlBusProcess)
void DlBusDataCallback(const uint8_t* data, size_t len) {
  if (len == 0) return;
  AddLog(LOG_LEVEL_DEBUG, PSTR("DLBUS: Processing %d bytes"), len);
  
  uint8_t cChecksum = 0;
  for(int i = 0; i< len-1; i++){
    cChecksum += data[i];
  }
  if (cChecksum != data[len-1]) {
    // Invalid checksum
    AddLog(LOG_LEVEL_ERROR, PSTR("DLBUS: Invalid checksum calculated 0x%02X (data[len-1] = 0x%02X)"), 
           cChecksum, data[len-1]);
  }else{
  }

  // Send via UDP if configured
  DlBusSendUdp(data, len);

  
  // Log first few bytes
  char hex_str[100];
  char *p = hex_str;
  for (size_t i = 0; i < len && i < 16; i++) {
    p += sprintf(p, "%02X ", data[i]);
  }
  if (p > hex_str) *(p-1) = '\0';
  AddLog(LOG_LEVEL_DEBUG, PSTR("DLBUS: Rx %u bytes: %s%s"), 
         len, hex_str, len > 16 ? "..." : "");
}

bool Xdrv100(uint32_t function) {
  bool result = false;
  
  switch (function) {
    case FUNC_INIT:
      DlBusDriverInit();
      break;

    case FUNC_EVERY_50_MSECOND:
      DlBusDriverEvery50ms();
      break;

    case FUNC_EVERY_SECOND:
      DlBusDriverEverySecond();
      break;
      
    case FUNC_COMMAND:
      result = DecodeCommand(kDlBusCommands, DlBusCommand);
      break;
  }
  return result;
}

#endif  // USE_XDRV_100_DLBUS
#endif  // ESP32
