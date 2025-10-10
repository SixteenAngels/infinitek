/*
  Infinitek.ino - Infinitek firmware for iTead Sonoff, Wemos, NodeMCU, ESP8266 and ESP32 hardwares

  Copyright (C) 2021  Theo Arends

  This program is free software: you can redistribute it and/or modify
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

// Location specific includes
#if __has_include("core_version.h")         // ESP32 Stage has no core_version.h file. Disable include via PlatformIO Option
#include <core_version.h>                   // Arduino_Esp8266 version information (ARDUINO_ESP8266_RELEASE and ARDUINO_ESP8266_RELEASE_2_7_1)
#endif  // ESP32_STAGE
#include "include/infinitek_compat.h"
#include "include/infinitek_version.h"        // Infinitek version information
#include "include/infinitek.h"                // Enumeration used in my_user_config.h
#include "my_user_config.h"                 // Fixed user configurable options
#ifdef USE_TLS
#include <t_bearssl.h>                      // We need to include before "Infinitek_globals.h" to take precedence over the BearSSL version in Arduino
#endif  // USE_TLS
#include "include/infinitek_globals.h"        // Function prototypes and global configuration
#include "include/i18n.h"                   // Language support configured by my_user_config.h
#include "include/infinitek_template.h"       // Hardware configuration

// ------------------------------------------------------------------------------------------
// If IPv6 is not support by the underlying esp-idf, disable it
// ------------------------------------------------------------------------------------------
#if !LWIP_IPV6
#undef USE_IPV6
#endif

// Libraries
#include <WiFiHelper.h>
#include <ESP8266HTTPClient.h>              // Ota
#include <ESP8266httpUpdate.h>              // Ota
#ifdef ESP32
#ifdef USE_TLS
#include "HTTPUpdateLight.h"                // Ota over HTTPS for ESP32
#endif  // USE_TLS
#endif  // ESP32
#include <StreamString.h>                   // Webserver, Updater
#include <ext_printf.h>
#include <SBuffer.hpp>
#include <LList.h>
#include <JsonParser.h>
#include <JsonGenerator.h>
#ifdef ESP8266
#ifdef USE_ARDUINO_OTA
#include <ArduinoOTA.h>                     // Arduino OTA
#ifndef USE_DISCOVERY
#define USE_DISCOVERY
#endif  // USE_DISCOVERY
#endif  // USE_ARDUINO_OTA
#endif  // ESP8266
#ifdef USE_DISCOVERY
#include <ESP8266mDNS.h>                    // MQTT, Webserver, Arduino OTA
#endif  // USE_DISCOVERY
#include <Wire.h>                           // I2C support library
#ifdef USE_SPI
#include <SPI.h>                            // SPI support, TFT, SDcard
#endif  // USE_SPI

// --- Sensor Libraries ---
#if defined(USE_DHT) || defined(USE_DHT11) || defined(USE_DHT22)
#include <DHT.h>
#endif
#if defined(USE_BME280) || defined(USE_BMP)
#include <Adafruit_BME280.h>
#endif
#if defined(USE_BMP280) || defined(USE_BMP)
#include <Adafruit_BMP280.h>
#endif
#if defined(USE_BME680) || defined(USE_BME68X)
#include <Adafruit_BME680.h>
#endif
#if defined(USE_SHT3X)
#include "include/SHT3x.h"
#endif
#if defined(USE_SHT4X)
#include <SHT4x.h>
#endif
#if defined(USE_CCS811) || defined(USE_CCS811_V2)
#include <Adafruit_CCS811.h>
#endif
#if defined(USE_SGP30)
#include <Adafruit_SGP30.h>
#endif
#if defined(USE_SCD30)
#include <FrogmoreScd30.h>
#endif
#if defined(USE_MHZ19)
#include <MHZ19.h>
#endif
#if defined(USE_PMS5003)
#include <PMS.h>
#endif

// --- Sensor Instances ---
#if defined(USE_DHT) || defined(USE_DHT11) || defined(USE_DHT22)
DHT dht;
#endif
#if defined(USE_BME280) || defined(USE_BMP)
Adafruit_BME280 bme280;
#endif
#if defined(USE_BMP280) || defined(USE_BMP)
Adafruit_BMP280 bmp280;
#endif
#if defined(USE_BME680) || defined(USE_BME68X)
Adafruit_BME680 bme680;
#endif
#if defined(USE_SHT3X)
SHT3x sht3x;
#endif
#if defined(USE_SHT4X)
SHT4x sht4x;
#endif
#if defined(USE_CCS811) || defined(USE_CCS811_V2)
Adafruit_CCS811 ccs811;
#endif
#if defined(USE_SGP30)
Adafruit_SGP30 sgp30;
#endif
#if defined(USE_SCD30)
FrogmoreScd30 scd30;
#endif
#if defined(USE_MHZ19)
MHZ19 mhz19;
#endif
#if defined(USE_PMS5003)
PMS pms(Serial);
#endif

#ifdef USE_UFILESYS
#ifdef ESP8266
#include <LittleFS.h>
#include <SPI.h>
#ifdef USE_SDCARD
#include <SD.h>
#include <SdFat.h>
#endif  // USE_SDCARD
#endif  // ESP8266
#ifdef ESP32
#include <LittleFS.h>
#ifdef USE_SDCARD
#include <SD.h>
#ifdef SOC_SDMMC_HOST_SUPPORTED
#include <SD_MMC.h>
#endif  // SOC_SDMMC_HOST_SUPPORTED
#endif  // USE_SDCARD
#include "FFat.h"
#include "FS.h"
#endif  // ESP32
#endif  // USE_UFILESYS

#ifdef ESP32
#include "include/tasconsole.h"
#if SOC_USB_SERIAL_JTAG_SUPPORTED
#include "hal/usb_serial_jtag_ll.h"
#include "esp_private/rtc_clk.h"
#endif  // SOC_USB_SERIAL_JTAG_SUPPORTED
#ifdef CONFIG_IDF_TARGET_ESP32
#include "driver/gpio.h"
#include "soc/efuse_reg.h"
#include "bootloader_common.h"
#endif
#endif  // ESP32

// Structs
#include "include/infinitek_types.h"

/*********************************************************************************************\
 * Global variables
\*********************************************************************************************/

const uint32_t VERSION_MARKER[] PROGMEM = { 0x5AA55AA5, 0xFFFFFFFF, 0xA55AA55A };

struct WIFI {
  int last_tx_pwr;
  uint32_t last_event = 0;                 // Last wifi connection event
  uint32_t downtime = 0;                   // Wifi down duration
  uint16_t link_count = 0;                 // Number of wifi re-connect
  uint8_t counter;
  uint8_t retry_init;
  uint8_t retry;
  uint8_t max_retry;
  uint8_t status;
  uint8_t config_type = 0;
  uint8_t config_counter = 0;
  uint8_t scan_state;
  uint8_t bssid[6];
  int8_t best_network_db;
  uint8_t wifiTest = WIFI_NOT_TESTING;
  uint8_t wifi_test_counter = 0;
  uint16_t save_data_counter = 0;
  uint8_t old_wificonfig = MAX_WIFI_OPTION; // means "nothing yet saved here"
  uint8_t phy_mode = 0;
  bool wifi_test_AP_TIMEOUT = false;
  bool wifi_Test_Restart = false;
  bool wifi_Test_Save_SSID2 = false;
} Wifi;

typedef struct {
  uint16_t      valid;                     // 280  (RTC memory offset 100 - sizeof(RTCRBT))
  uint8_t       fast_reboot_count;         // 282
  uint8_t       free_003[1];               // 283
} TRtcReboot;
TRtcReboot RtcReboot;
#ifdef ESP32
static RTC_NOINIT_ATTR TRtcReboot RtcDataReboot;
#endif  // ESP32

typedef struct {
  uint16_t      valid;                     // 290  (RTC memory offset 100)
  uint8_t       oswatch_blocked_loop;      // 292
  uint8_t       ota_loader;                // 293
  uint32_t      ex_energy_kWhtoday;        // 294
  uint32_t      ex_energy_kWhtotal;        // 298
  volatile uint32_t pulse_counter[MAX_COUNTERS];  // 29C - See #9521 why volatile
  power_t       power;                     // 2AC
  EnergyUsage   energy_usage;              // 2B0
  uint32_t      nextwakeup;                // 2C8
  uint32_t      baudrate;                  // 2CC
  uint32_t      ultradeepsleep;            // 2D0
  uint16_t      deepsleep_slip;            // 2D4
  uint8_t       improv_state;              // 2D6

  uint8_t       free_2d7[1];               // 2D7

  int32_t       energy_kWhtoday_ph[3];     // 2D8
  int32_t       energy_kWhtotal_ph[3];     // 2E4
  int32_t       energy_kWhexport_ph[3];    // 2F0
  uint32_t      utc_time;                  // 2FC
} TRtcSettings;
TRtcSettings RtcSettings;
#ifdef ESP32
static RTC_NOINIT_ATTR TRtcSettings RtcDataSettings;
#endif  // ESP32

struct TIME_T {
  uint32_t      nanos;
  uint8_t       second;
  uint8_t       minute;
  uint8_t       hour;
  uint8_t       day_of_week;               // sunday is day 1
  uint8_t       day_of_month;
  uint8_t       month;
  char          name_of_month[4];
  uint16_t      day_of_year;
  uint16_t      year;
  uint32_t      days;
  uint32_t      valid;
} RtcTime;

struct XDRVMAILBOX {
  bool          grpflg;
  bool          usridx;
  uint16_t      command_code;
  uint32_t      index;
  uint32_t      data_len;
  int32_t       payload;
  char         *topic;
  char         *data;
  char         *command;
} XdrvMailbox;

WiFiUDP PortUdp;                            // UDP Syslog and Alexa

#ifdef ESP32
/*
#if CONFIG_IDF_TARGET_ESP32C3 ||            // support USB via HWCDC using JTAG interface
    CONFIG_IDF_TARGET_ESP32C5 ||            // support USB via HWCDC using JTAG interface
    CONFIG_IDF_TARGET_ESP32C6 ||            // support USB via HWCDC using JTAG interface
    CONFIG_IDF_TARGET_ESP32S2 ||            // support USB via USBCDC
    CONFIG_IDF_TARGET_ESP32S3               // support USB via HWCDC using JTAG interface or USBCDC
*/
#if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4

//#if CONFIG_TINYUSB_CDC_ENABLED              // This define is not recognized here so use USE_USB_CDC_CONSOLE
#ifdef USE_USB_CDC_CONSOLE
//#warning **** TasConsole use USB ****
bool tasconsole_serial = false;

#if ARDUINO_USB_MODE
//#warning **** TasConsole ARDUINO_USB_MODE ****
#if !ARDUINO_USB_CDC_ON_BOOT
HWCDC HWCDCSerial;
#endif  // ARDUINO_USB_CDC_ON_BOOT
TASCONSOLE TasConsole{HWCDCSerial};         // ESP32C3/C6/S3 embedded USB using JTAG interface
//#warning **** TasConsole uses HWCDC ****
#else   // No ARDUINO_USB_MODE
#include "USB.h"
#include "USBCDC.h"
#if !ARDUINO_USB_CDC_ON_BOOT
USBCDC USBSerial;                           // Already defined in USBCDC.cpp
#endif  // ARDUINO_USB_CDC_ON_BOOT
TASCONSOLE TasConsole{USBSerial};           // ESP32Sx embedded USB interface
//#warning **** TasConsole uses USBCDC ****
#endif  // ARDUINO_USB_MODE

#else   // No USE_USB_CDC_CONSOLE
TASCONSOLE TasConsole{Serial};
bool tasconsole_serial = true;
//#warning **** TasConsole uses Serial ****
#endif  // USE_USB_CDC_CONSOLE
#else   // No ESP32C3, S2 or S3
TASCONSOLE TasConsole{Serial};
bool tasconsole_serial = true;
//#warning **** TasConsole uses Serial ****
#endif  // ESP32C3, S2 or S3

#else   // No ESP32
HardwareSerial TasConsole = Serial;         // Only serial interface
#endif  // ESP32

char EmptyStr[1] = { 0 };                   // Provide a pointer destination to an empty char string

struct InfinitekGlobal_t {
  uint32_t global_update;                   // Timestamp of last global temperature and humidity update
  uint32_t baudrate;                        // Current Serial baudrate
  uint32_t pulse_timer[MAX_PULSETIMERS];    // Power off timer
  uint32_t blink_timer;                     // Power cycle timer
  uint32_t backlog_timer;                   // Timer for next command in backlog
  uint32_t loop_load_avg;                   // Indicative loop load average
  uint32_t log_buffer_pointer;              // Index in log buffer
  uint32_t uptime;                          // Counting every second until 4294967295 = 130 year
  uint32_t zc_time;                         // Zero-cross moment (microseconds)
  uint32_t zc_offset;                       // Zero cross moment offset due to monitoring chip processing (microseconds)
  uint32_t zc_code_offset;                  // Zero cross moment offset due to executing power code (microseconds)
  uint32_t zc_interval;                     // Zero cross interval around 8333 (60Hz) or 10000 (50Hz) (microseconds)
  GpioOptionABits gpio_optiona;             // GPIO Option_A flags
  void *log_buffer_mutex;                   // Control access to log buffer

  power_t power;                            // Current copy of Settings->power
  power_t power_latching;                   // Current state of single pin latching power
  power_t rel_inverted;                     // Relay inverted flag (1 = (0 = On, 1 = Off))
  power_t rel_bistable;                     // Relay bistable bitmap
  power_t last_power;                       // Last power set state
  power_t blink_power;                      // Blink power state
  power_t blink_powersave;                  // Blink start power save state
  power_t blink_mask;                       // Blink relay active mask
  power_t power_on_delay_state;

  int serial_in_byte_counter;               // Index in receive buffer

  float temperature_celsius;                // Provide a global temperature to be used by some sensors
  float humidity;                           // Provide a global humidity to be used by some sensors
  float pressure_hpa;                       // Provide a global pressure to be used by some sensors

  uint16_t gpio_pin[MAX_GPIO_PIN];          // GPIO functions indexed by pin number
  myio my_module;                           // Active copy of Module GPIOs (17 x 16 bits)
  uint16_t blink_counter;                   // Number of blink cycles
  uint16_t seriallog_timer;                 // Timer to disable Seriallog
  uint16_t syslog_timer;                    // Timer to re-enable syslog_level
  uint16_t tele_period;                     // Tele period timer
  int16_t save_data_counter;                // Counter and flag for config save to Flash
  RulesBitfield rules_flag;                 // Rule state flags (16 bits)

  StateBitfield global_state;               // Global states (currently Wifi and Mqtt) (8 bits)
  uint16_t pwm_inverted;                    // PWM inverted flag (1 = inverted) - extended to 16 bits for ESP32
#ifdef ESP32
  int16_t pwm_cur_value[MAX_PWMS];          // Current effective values of PWMs as applied to GPIOs
  int16_t pwm_cur_phase[MAX_PWMS];          // Current phase values of PWMs as applied to GPIOs
  int16_t pwm_value[MAX_PWMS];              // Wanted values of PWMs after update - -1 means no change
  int16_t pwm_phase[MAX_PWMS];              // Wanted phase of PWMs after update - -1 means no change
#endif  // ESP32

  bool serial_local;                        // Handle serial locally
  bool fallback_topic_flag;                 // Use Topic or FallbackTopic
  bool no_mqtt_response;                    // Respond with rule processing only
  bool backlog_nodelay;                     // Execute all backlog commands with no delay
  bool backlog_mutex;                       // Command backlog pending
  bool backlog_no_mqtt_response;            // Set respond with rule processing only
  bool stop_flash_rotate;                   // Allow flash configuration rotation
  bool blinkstate;                          // LED state
  bool pwm_present;                         // Any PWM channel configured with SetOption15 0
  bool i2c_enabled[2];                      // I2C configured for all possible buses (1 or 2)
#ifdef ESP32
  bool camera_initialized;                  // For esp32-webcam, to be used in discovery
  bool ota_factory;                         // Select safeboot binary
#endif  // ESP32
  bool ntp_force_sync;                      // Force NTP sync
  bool skip_light_fade;                     // Temporarily skip light fading
  bool restart_halt;                        // Do not restart but stay in wait loop
  bool restart_deepsleep;                   // Do not restart but do deepsleep
  bool module_changed;                      // Indicate module changed since last restart
  bool wifi_stay_asleep;                    // Allow sleep only incase of ESP32 BLE
  bool no_autoexec;                         // Disable autoexec

  uint8_t user_globals[3];                  // User set global temp/hum/press
  uint8_t busy_time;                        // Time in ms to allow executing of time critical functions
  uint8_t skip_sleep;                       // Skip sleep loops
  uint8_t init_state;                       // Infinitek init state
  uint8_t heartbeat_inverted;               // Heartbeat pulse inverted flag
  uint8_t spi_enabled;                      // SPI configured (bus1)
  uint8_t spi_enabled2;                     // SPI configured (bus2)
  uint8_t soft_spi_enabled;                 // Software SPI configured
  uint8_t blinks;                           // Number of LED blinks
  uint8_t restart_flag;                     // Infinitek restart flag
  uint8_t ota_state_flag;                   // OTA state flag
  uint8_t wifi_state_flag;                  // Wifi state flag
  uint8_t mqtt_cmnd_blocked;                // Ignore flag for publish command
  uint8_t mqtt_cmnd_blocked_reset;          // Count down to reset if needed
  uint8_t state_250mS;                      // State 250msecond per second flag
  uint8_t latching_relay_pulse;             // Latching relay pulse timer
  uint8_t active_device;                    // Active device in ExecuteCommandPower
  uint8_t sleep;                            // Current copy of Settings->sleep
  uint8_t leds_present;                     // Max number of LED supported
  uint8_t led_inverted;                     // LED inverted flag (1 = (0 = On, 1 = Off))
  uint8_t led_power;                        // LED power state
  uint8_t ledlnk_inverted;                  // Link LED inverted flag (1 = (0 = On, 1 = Off))
  // uint8_t pwm_inverted;                  // duplicate of 16-bit field above; remove to avoid redefinition
  uint8_t energy_driver;                    // Energy monitor configured
  uint8_t light_driver;                     // Light module configured
  uint8_t light_type;                       // Light types
  uint8_t serial_in_byte;                   // Received byte
  uint8_t serial_skip;                      // Skip number of received messages
  uint8_t devices_present;                  // Max number of devices supported
  uint8_t maxlog_level;                     // Max allowed log level
  uint8_t masterlog_level;                  // Master log level used to override set log level
  uint8_t seriallog_level;                  // Current copy of Settings->seriallog_level
  uint8_t syslog_level;                     // Current copy of Settings->syslog_level
  uint8_t templog_level;                    // Temporary log level to be used by HTTP cm and Telegram
  uint8_t module_type;                      // Current copy of Settings->module or user template type
  uint8_t emulated_module_type;             // Emulated module type as requested by ESP32
  uint8_t last_source;                      // Last command source
  uint8_t shutters_present;                 // Number of actual define shutters
  uint8_t discovery_counter;                // Delayed discovery counter
  uint8_t power_on_delay;                   // Delay relay power on to reduce power surge (SetOption47)
#ifdef USE_PWM_DIMMER
  uint8_t restore_powered_off_led_counter;  // Seconds before powered-off LED (LEDLink) is restored
  uint8_t pwm_dimmer_led_bri;               // Adjusted brightness LED level
#endif  // USE_PWM_DIMMER

  String mqtt_data;                         // Buffer filled by Response functions
  char version[16];                         // Composed version string like 255.255.255.255
  char image_name[33];                      // Code image and/or commit
  char hostname[33];                        // Composed Wifi hostname
  char serial_in_buffer[INPUT_BUFFER_SIZE];  // Receive buffer
  char mqtt_client[99];                     // Composed MQTT Clientname
  char mqtt_topic[TOPSZ];                   // Composed MQTT topic

#ifdef PIO_FRAMEWORK_ARDUINO_MMU_CACHE16_IRAM48_SECHEAP_SHARED
  char* log_buffer = nullptr;               // Log buffer in IRAM
#else
  char log_buffer[LOG_BUFFER_SIZE];         // Log buffer in DRAM
#endif  // PIO_FRAMEWORK_ARDUINO_MMU_CACHE16_IRAM48_SECHEAP_SHARED

#ifdef USE_BERRY
  bool berry_deferred_ready = false;        // is there an deferred Berry function to be called at next millisecond
  bool berry_fast_loop_enabled = false;     // is Berry fast loop enabled, i.e. control is passed at each loop iteration
#endif  // USE_BERRY
} InfinitekGlobal = { 0 };

TSettings* Settings = nullptr;

LList<char*> backlog;                       // Command backlog implemented with InfinitekLList
#define BACKLOG_EMPTY (backlog.isEmpty())

/*********************************************************************************************\
 * Main
\*********************************************************************************************/

#ifdef ESP32
// IDF5.3 fix esp_gpio_reserve used in init PSRAM. Needed by Infinitek.ino esp_gpio_revoke
#include "esp_private/esp_gpio_reserve.h"
#endif  // ESP32

void setup(void) {
#ifdef ESP32
#ifdef CONFIG_IDF_TARGET_ESP32

#ifdef DISABLE_ESP32_BROWNOUT
  DisableBrownout();      // Workaround possible weak LDO resulting in brownout detection during Wifi connection
#endif  // DISABLE_ESP32_BROWNOUT

#ifndef FIRMWARE_SAFEBOOT
#ifndef DISABLE_PSRAMCHECK
#ifndef CORE32SOLO1
  // restore GPIO5/18 or 16/17 if no PSRAM is found which may be used by Ethernet among others
  if (!FoundPSRAM()) {
    // test if the CPU is not pico
    uint32_t pkg_version = bootloader_common_get_chip_ver_pkg();
    if (pkg_version <= 3) {         // D0WD, S0WD, D2WD
      gpio_reset_pin((gpio_num_t)CONFIG_D0WD_PSRAM_CS_IO);
      gpio_reset_pin((gpio_num_t)CONFIG_D0WD_PSRAM_CLK_IO);
      // IDF5.3 fix esp_gpio_reserve used in init PSRAM
      esp_gpio_revoke(BIT64(CONFIG_D0WD_PSRAM_CS_IO) | BIT64(CONFIG_D0WD_PSRAM_CLK_IO));
    }
  }
#endif  // CORE32SOLO1
#endif  // DISABLE_PSRAMCHECK
#endif  // FIRMWARE_SAFEBOOT
#endif  // CONFIG_IDF_TARGET_ESP32
#endif  // ESP32

#ifdef USE_ESP32_WDT
  enableLoopWDT();          // enabled WDT Watchdog on Arduino `loop()` - must return before 5s or called `feedLoopWDT();` - included in `yield()`
#endif // USE_ESP32_WDT

  RtcPreInit();
  SettingsInit();

#ifdef USE_EMERGENCY_RESET
  EmergencyReset();
#endif  // USE_EMERGENCY_RESET

  InfinitekGlobal.baudrate = APP_BAUDRATE;
  InfinitekGlobal.seriallog_timer = SERIALLOG_TIMER;
  InfinitekGlobal.temperature_celsius = NAN;
  InfinitekGlobal.blinks = 201;
    // --- Power & Energy Monitoring Sensors ---
  #ifdef USE_HLW8012
    hlw8012.begin();
  #endif
  #ifdef USE_BL0937
    bl0937.begin();
  #endif
  #ifdef USE_INA219
    ina219.begin();
  #endif
  #ifdef USE_INA3221
    ina3221.begin();
  #endif
  #ifdef USE_PZEM004T
    pzem.begin(Serial);
  #endif

    // --- Motion & Presence Sensors ---
  #ifdef USE_PIR
    pinMode(PIR_PIN, INPUT);
  #endif
  #ifdef USE_RADAR
    radar.begin();
  #endif
  #ifdef USE_ULTRASONIC
    ultrasonic.begin();
  #endif

    // --- Light & Color Sensors ---
  #ifdef USE_BH1750
    bh1750.begin();
  #endif
  #ifdef USE_TSL2561
    tsl2561.begin();
  #endif
  #ifdef USE_TSL2591
    tsl2591.begin();
  #endif
  #ifdef USE_APDS9960
    apds9960.begin();
  #endif

    // --- Switches & Buttons ---
  #ifdef USE_SWITCH
    pinMode(SWITCH_PIN, INPUT_PULLUP);
  #endif
  #ifdef USE_TOUCH
    touch.begin();
  #endif

    // --- Virtual Sensors ---
    // No hardware init needed

    // --- Advanced / Special Sensors ---
  #ifdef USE_GPS
    gps.begin();
  #endif
  #ifdef USE_RFID
    rfid.begin();
  #endif
  #ifdef USE_CAMERA
    camera.begin();
  #endif
    // --- Sensor Initialization ---
  #if USE_DHT11 || USE_DHT22
    dht.begin();
  #endif
  #if USE_BME280
    bme280.begin(0x76); // Default I2C address
  #endif
  #if USE_BMP280
    bmp280.begin(0x76); // Default I2C address
  #endif
  #if USE_BME680
    bme680.begin(0x76); // Default I2C address
  #endif
  #if USE_SHT3X
    sht3x.begin();
  #endif
  #if USE_SHT4X
    sht4x.begin();
  #endif
  #if USE_CCS811
    ccs811.begin();
  #endif
  #if USE_SGP30
    sgp30.begin();
  #endif
  #if USE_SCD30
    scd30.begin();
  #endif
  #if USE_MHZ19
    mhz19.begin(Serial);
  #endif
  #if USE_PMS5003
    pms.passiveMode();
  #endif
  InfinitekGlobal.wifi_state_flag = WIFI_RESTART;
  InfinitekGlobal.tele_period = 9999;
  InfinitekGlobal.active_device = 1;
  InfinitekGlobal.global_state.data = 0xF;  // Init global state (wifi_down, mqtt_down) to solve possible network issues
  InfinitekGlobal.maxlog_level = LOG_LEVEL_DEBUG_MORE;
  InfinitekGlobal.seriallog_level = (SERIAL_LOG_LEVEL > LOG_LEVEL_INFO) ? SERIAL_LOG_LEVEL : LOG_LEVEL_INFO;  // Allow specific serial messages until config loaded and allow more logging than INFO
  InfinitekGlobal.power_latching = 0x80000000;

  RtcRebootLoad();
  if (!RtcRebootValid()) {
    RtcReboot.fast_reboot_count = 0;
  }
#ifdef FIRMWARE_MINIMAL
  RtcReboot.fast_reboot_count = 0;    // Disable fast reboot and quick power cycle detection
#else  // No FIRMWARE_MINIMAL
  if (ResetReason() == REASON_DEEP_SLEEP_AWAKE) {
    RtcReboot.fast_reboot_count = 0;  // Disable fast reboot and quick power cycle detection
  } else {
    RtcReboot.fast_reboot_count++;
  }
#endif  // FIRMWARE_MINIMAL
  RtcRebootSave();

  if (RtcSettingsLoad(0)) {
    uint32_t baudrate = (RtcSettings.baudrate / 300) * 300;  // Make it a valid baudrate
    if (baudrate) { InfinitekGlobal.baudrate = baudrate; }
  }

  // Init settings and logging preparing for AddLog use
#ifdef PIO_FRAMEWORK_ARDUINO_MMU_CACHE16_IRAM48_SECHEAP_SHARED
  ESP.setIramHeap();
  Settings = (TSettings*)calloc(1, sizeof(TSettings));          // Allocate in "new" 16k heap space
  InfinitekGlobal.log_buffer = (char*)malloc(LOG_BUFFER_SIZE);    // Allocate in "new" 16k heap space
  ESP.resetHeap();
  if (InfinitekGlobal.log_buffer == nullptr) {
    InfinitekGlobal.log_buffer = (char*)malloc(LOG_BUFFER_SIZE);  // Allocate in "old" heap space as fallback
  }
  if (InfinitekGlobal.log_buffer != nullptr) {
    InfinitekGlobal.log_buffer[0] = '\0';
  }
#endif  // PIO_FRAMEWORK_ARDUINO_MMU_CACHE16_IRAM48_SECHEAP_SHARED
  if (Settings == nullptr) {
    Settings = (TSettings*)calloc(1, sizeof(TSettings));
  }

#ifdef ESP32
#if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4
#ifdef USE_USB_CDC_CONSOLE

  bool is_connected_to_USB = false;
  TasConsole.setRxBufferSize(INPUT_BUFFER_SIZE);
  TasConsole.begin(115200);           // always start CDC to test plugged cable
#if SOC_USB_SERIAL_JTAG_SUPPORTED  // Not S2
  for (uint32_t i = 0; i < 5; i++) {  // wait up to 250 ms - maybe a shorter time is enough
      is_connected_to_USB = HWCDCSerial.isPlugged();
      if (is_connected_to_USB) { break; }
      delay(50);
  }
#else
  is_connected_to_USB = true;      // S2
#endif  // SOC_USB_SERIAL_JTAG_SUPPORTED

  if (is_connected_to_USB) {
    // TasConsole is already running
#if !ARDUINO_USB_MODE
    USB.begin();                 // This needs a serial console with DTR/DSR support
#endif  // No ARDUINO_USB_MODE
    TasConsole.println();
    AddLog(LOG_LEVEL_INFO, PSTR("CMD: Using USB CDC"));
  } else {
#if SOC_USB_SERIAL_JTAG_SUPPORTED  // Not S2
    HWCDCSerial.~HWCDC();       // not needed, deinit CDC
#endif  // SOC_USB_SERIAL_JTAG_SUPPORTED
    // Init command serial console preparing for AddLog use
    Serial.begin(InfinitekGlobal.baudrate);
    Serial.println();
    TasConsole = Serial;        // Fallback
    tasconsole_serial = true;
    AddLog(LOG_LEVEL_INFO, PSTR("CMD: Fall back to serial port, no SOF packet detected on USB port"));
  }
#else   // No USE_USB_CDC_CONSOLE
  // Init command serial console preparing for AddLog use
  Serial.begin(InfinitekGlobal.baudrate);
  Serial.println();
//  Serial.setRxBufferSize(INPUT_BUFFER_SIZE);  // Default is 256 chars
  TasConsole = Serial;
#endif  // USE_USB_CDC_CONSOLE
#else   // No ESP32C3, S2 or S3
  // Init command serial console preparing for AddLog use
  Serial.begin(InfinitekGlobal.baudrate);
  Serial.println();
//  Serial.setRxBufferSize(INPUT_BUFFER_SIZE);  // Default is 256 chars
  TasConsole = Serial;
#endif  // ESP32C3, S2 or S3

#else   // No ESP32
  // Init command serial console preparing for AddLog use
  Serial.begin(InfinitekGlobal.baudrate);
  Serial.println();
//  Serial.setRxBufferSize(INPUT_BUFFER_SIZE);  // Default is 256 chars
  TasConsole = Serial;
#endif  // ESP32

  // Ready for AddLog use

//  AddLog(LOG_LEVEL_INFO, PSTR("ADR: Settings %p, Log %p"), Settings, InfinitekGlobal.log_buffer);
#ifdef ESP32
  AddLog(LOG_LEVEL_INFO, PSTR("HDW: %s %s"), GetDeviceHardwareRevision().c_str(),
            FoundPSRAM() ? (CanUsePSRAM() ? "(PSRAM)" : "(PSRAM disabled)") : "" );
  // AddLog(LOG_LEVEL_DEBUG, PSTR("HDW: FoundPSRAM=%i CanUsePSRAM=%i"), FoundPSRAM(), CanUsePSRAM());
#if !defined(HAS_PSRAM_FIX)
  if (FoundPSRAM() && !CanUsePSRAM()) {
    AddLog(LOG_LEVEL_INFO, PSTR("HDW: PSRAM is disabled, requires specific compilation on this hardware (see doc)"));
  }
#endif  // HAS_PSRAM_FIX
#else   // ESP8266
  AddLog(LOG_LEVEL_INFO, PSTR("HDW: %s"), GetDeviceHardware().c_str());
#endif  // ESP32

#ifdef USE_UFILESYS
  UfsInit();  // xdrv_50_filesystem.ino
#endif  // USE_UFILESYS

  SettingsLoad();
  SettingsDelta();
  SettingsMinimum();                           // Set life-saving parameters if out-of-range due to reconfig Settings Area

  OsWatchInit();

  InfinitekGlobal.seriallog_level = Settings->seriallog_level;
  InfinitekGlobal.syslog_level = Settings->syslog_level;

  InfinitekGlobal.module_changed = (Settings->module != Settings->last_module);
  if (InfinitekGlobal.module_changed) {
    Settings->baudrate = APP_BAUDRATE / 300;
    Settings->serial_config = TS_SERIAL_8N1;
  }
  SetSerialInitBegin();                        // Reset serial interface if current baudrate and/or config is different from requested settings

  if (1 == RtcReboot.fast_reboot_count) {      // Allow setting override only when all is well
    UpdateQuickPowerCycle(true);
  }

  if (ResetReason() != REASON_DEEP_SLEEP_AWAKE) {
#ifdef ESP8266
    Settings->flag4.network_wifi = 1;           // Make sure we're in control
#endif  // ESP8266
#ifdef ESP32
    if (!Settings->flag4.network_ethernet) {
      Settings->flag4.network_wifi = 1;         // Make sure we're in control
    }
#endif  // ESP32
  }

  InfinitekGlobal.stop_flash_rotate = Settings->flag.stop_flash_rotate;  // SetOption12 - Switch between dynamic or fixed slot flash save location
  InfinitekGlobal.save_data_counter = Settings->save_data;
  InfinitekGlobal.sleep = Settings->sleep;
#ifndef USE_EMULATION
  Settings->flag2.emulation = 0;
#else   // No USE_EMULATION
#ifndef USE_EMULATION_WEMO
  if (EMUL_WEMO == Settings->flag2.emulation) { Settings->flag2.emulation = 0; }
#endif  // USE_EMULATION_WEMO
#ifndef USE_EMULATION_HUE
  if (EMUL_HUE == Settings->flag2.emulation) { Settings->flag2.emulation = 0; }
#endif  // USE_EMULATION_HUE
#endif  // USE_EMULATION

//  AddLog(LOG_LEVEL_INFO, PSTR("DBG: InfinitekGlobal size %d, data %100_H"), sizeof(InfinitekGlobal), (uint8_t*)&InfinitekGlobal);

  if (Settings->param[P_BOOT_LOOP_OFFSET]) {         // SetOption36
    // Disable functionality as possible cause of fast restart within BOOT_LOOP_TIME seconds (Exception, WDT or restarts)
    if (RtcReboot.fast_reboot_count > Settings->param[P_BOOT_LOOP_OFFSET]) {       // Restart twice
      Settings->flag3.user_esp8285_enable = 0;       // SetOption51 - Enable ESP8285 user GPIO's - Disable ESP8285 Generic GPIOs interfering with flash SPI
      if (RtcReboot.fast_reboot_count > Settings->param[P_BOOT_LOOP_OFFSET] +1) {  // Restart 3 times
        for (uint32_t i = 0; i < MAX_RULE_SETS; i++) {
          if (bitRead(Settings->rule_stop, i)) {
            bitWrite(Settings->rule_enabled, i, 0);  // Disable rules causing boot loop
          }
        }
        Settings->flag4.network_wifi = 1;            // Enable wifi if disabled
      }
      if (RtcReboot.fast_reboot_count > Settings->param[P_BOOT_LOOP_OFFSET] +2) {  // Restarted 4 times
        Settings->rule_enabled = 0;                  // Disable all rules
        Settings->flag3.shutter_mode = 0;            // disable shutter support
        InfinitekGlobal.no_autoexec = true;
      }
      if (RtcReboot.fast_reboot_count > Settings->param[P_BOOT_LOOP_OFFSET] +3) {  // Restarted 5 times
        for (uint32_t i = 0; i < nitems(Settings->my_gp.io); i++) {
          Settings->my_gp.io[i] = GPIO_NONE;         // Reset user defined GPIO disabling sensors
        }
      }
      if (RtcReboot.fast_reboot_count > Settings->param[P_BOOT_LOOP_OFFSET] +4) {  // Restarted 6 times
        Settings->module = Settings->fallback_module;  // Reset module to fallback module
//        Settings->last_module = Settings->fallback_module;
      }
      AddLog(LOG_LEVEL_INFO, PSTR("FRC: " D_LOG_SOME_SETTINGS_RESET " (%d)"), RtcReboot.fast_reboot_count);
#ifdef ESP32
#ifndef FIRMWARE_MINIMAL
      if (RtcReboot.fast_reboot_count > Settings->param[P_BOOT_LOOP_OFFSET] +8) {  // Restarted 10 times
        if (EspPrepSwitchPartition(0)) {             // Switch to safeboot
          RtcReboot.fast_reboot_count = 0;           // Reset for next user restart
          RtcRebootSave();
          EspRestart();                              // Restart in safeboot mode
        }
      }
#endif  // FIRMWARE_MINIMAL
#endif  // ESP32
    }
  }

  memcpy_P(InfinitekGlobal.version, VERSION_MARKER, 1);  // Dummy for compiler saving VERSION_MARKER

  snprintf_P(InfinitekGlobal.version, sizeof(InfinitekGlobal.version), PSTR("%d.%d.%d"), Infinitek_VERSION >> 24 & 0xff, Infinitek_VERSION >> 16 & 0xff, Infinitek_VERSION >> 8 & 0xff);  // Release version 6.3.0
  if (Infinitek_VERSION & 0xff) {  // Development or patched version 6.3.0.10
    snprintf_P(InfinitekGlobal.version, sizeof(InfinitekGlobal.version), PSTR("%s.%d"), InfinitekGlobal.version, Infinitek_VERSION & 0xff);
  }

  // Thehackbox inserts "release" or "commit number" before compiling using sed -i -e 's/PSTR("(%s)")/PSTR("(85cff52-%s)")/g' Infinitek.ino
  // Github inserts "release" or "commit number" before compiling using sed -i -e 's/Infinitek_SHA_SHORT/Infinitek_SHA_SHORT 85cff52-/g' Infinitek_version.h
  snprintf_P(InfinitekGlobal.image_name, sizeof(InfinitekGlobal.image_name), PSTR("(" STR(Infinitek_SHA_SHORT) "%s)"), PSTR(CODE_IMAGE_STR));  // Results in (85cff52-Infinitek) or (release-Infinitek)

  Format(InfinitekGlobal.mqtt_client, SettingsText(SET_MQTT_CLIENT), sizeof(InfinitekGlobal.mqtt_client));
  Format(InfinitekGlobal.mqtt_topic, SettingsText(SET_MQTT_TOPIC), sizeof(InfinitekGlobal.mqtt_topic));
  if (strchr(SettingsText(SET_HOSTNAME), '%') != nullptr) {
    SettingsUpdateText(SET_HOSTNAME, WIFI_HOSTNAME);
    snprintf_P(InfinitekGlobal.hostname, sizeof(InfinitekGlobal.hostname)-1, SettingsText(SET_HOSTNAME), InfinitekGlobal.mqtt_topic, ESP_getChipId() & 0x1FFF);
  } else {
    snprintf_P(InfinitekGlobal.hostname, sizeof(InfinitekGlobal.hostname)-1, SettingsText(SET_HOSTNAME));
  }
  char *s = InfinitekGlobal.hostname;
  while (*s) {
    if (!(isalnum(*s) || ('.' == *s))) { *s = '-'; }                 // Valid hostname chars are A..Z, a..z, 0..9, . and -
    if ((s == InfinitekGlobal.hostname) && ('-' == *s)) { *s = 'x'; }  // First char cannot be a dash so replace by an x
    s++;
  }
  snprintf_P(InfinitekGlobal.mqtt_topic, sizeof(InfinitekGlobal.mqtt_topic), ResolveToken(InfinitekGlobal.mqtt_topic).c_str());

  RtcInit();
  GpioInit();                    // FUNC_SETUP_RING1 -> FUNC_SETUP_RING2 -> FUNC_MODULE_INIT -> FUNC_LED_LINK
  ButtonInit();                  // FUNC_ADD_BUTTON
  SwitchInit();                  // FUNC_ADD_SWITCH
#ifdef ROTARY_V1
  RotaryInit();
#endif  // ROTARY_V1
#ifdef USE_BERRY
  if (!InfinitekGlobal.no_autoexec) {
    BerryInit();                 // Load preinit.be
  }
#endif  // USE_BERRY

  XdrvXsnsCall(FUNC_PRE_INIT);   // FUNC_PRE_INIT

  InfinitekGlobal.init_state = INIT_GPIOS;

  SetPowerOnState();             // FUNC_SET_POWER -> FUNC_SET_DEVICE_POWER
  WifiConnect();

    // Protocol selection logic
    #if defined(USE_MQTT) && USE_MQTT
      AddLog(LOG_LEVEL_INFO, PSTR("Protocol: MQTT enabled"));
      // Place MQTT initialization code here
    #endif

    #if defined(USE_MATTER) && USE_MATTER
      AddLog(LOG_LEVEL_INFO, PSTR("Protocol: Matter enabled"));
      // Place Matter initialization code here
    #endif

    #if defined(USE_MQTT) && defined(USE_MATTER)
      #if USE_MQTT && USE_MATTER
        AddLog(LOG_LEVEL_WARN, PSTR("Warning: Both MQTT and Matter are enabled. This may cause conflicts."));
        AddLog(LOG_LEVEL_WARN, PSTR("Consider using only one protocol at a time for optimal performance."));
      #endif
    #endif

  AddLog(LOG_LEVEL_INFO, PSTR(D_PROJECT " %s - %s " D_VERSION " %s%s-" ARDUINO_CORE_RELEASE "(%s)"),
    PSTR(PROJECT), SettingsText(SET_DEVICENAME), InfinitekGlobal.version, InfinitekGlobal.image_name, GetBuildDateAndTime().c_str());
#ifdef FIRMWARE_MINIMAL
  AddLog(LOG_LEVEL_INFO, PSTR(D_WARNING_MINIMAL_VERSION));
#endif  // FIRMWARE_MINIMAL

#ifdef ESP8266
#ifdef USE_ARDUINO_OTA
  ArduinoOTAInit();
#endif  // USE_ARDUINO_OTA
#endif  // ESP8266

  XdrvXsnsCall(FUNC_INIT);       // FUNC_INIT
#ifdef USE_SCRIPT
  if (bitRead(Settings->rule_enabled, 0)) Run_Scripter(">BS",3,0);
#endif  // USE_SCRIPT

  InfinitekGlobal.rules_flag.system_init = 1;
}

void BacklogLoop(void) {
  if (TimeReached(InfinitekGlobal.backlog_timer)) {
    if (!BACKLOG_EMPTY && !InfinitekGlobal.backlog_mutex) {
      InfinitekGlobal.backlog_mutex = true;
      bool nodelay = false;
      do {
        char* cmd = *backlog.head();
        backlog.removeHead();
/*
        // This adds 32 bytes
        char* cmd = *backlog.removeHead();
*/
        if (!strncasecmp_P(cmd, PSTR(D_CMND_NODELAY), strlen(D_CMND_NODELAY))) {
          free(cmd);
          nodelay = true;
        } else {
          InfinitekGlobal.no_mqtt_response = InfinitekGlobal.backlog_no_mqtt_response;
          ExecuteCommand(cmd, SRC_BACKLOG);
          free(cmd);
          if (nodelay || InfinitekGlobal.backlog_nodelay) {
            InfinitekGlobal.backlog_timer = millis();  // Reset backlog_timer which has been set by ExecuteCommand (CommandHandler)
          }
          break;
        }
      } while (!BACKLOG_EMPTY);
      InfinitekGlobal.backlog_mutex = false;
    }
    if (BACKLOG_EMPTY) {
      InfinitekGlobal.backlog_nodelay = false;
    }
  }
}

void SleepSkip(void) {
  InfinitekGlobal.skip_sleep = 250;     // Skip sleep for 250 loops;
}

void SleepDelay(uint32_t mseconds) {
  if (!InfinitekGlobal.backlog_nodelay && mseconds) {
    uint32_t wait = millis() + mseconds;
    while (!InfinitekGlobal.skip_sleep &&  // We need to service imminent interrupts ASAP
           !TimeReached(wait) &&
           !Serial.available()) {     // We need to service serial buffer ASAP as otherwise we get uart buffer overrun
      XdrvXsnsCall(FUNC_SLEEP_LOOP);  // Main purpose is reacting ASAP on serial data availability or interrupt handling (ADE7880)
      if (InfinitekGlobal.skip_sleep) { break; }
      delay(1);
    }
  } else {
    delay(0);
  }
}

void Scheduler(void) {
  XdrvXsnsCall(FUNC_LOOP);

// check LEAmDNS.h
// MDNS.update() needs to be called in main loop
#ifdef ESP8266                     // Not needed with esp32 mdns
#ifdef USE_DISCOVERY
#ifdef USE_WEBSERVER
#ifdef WEBSERVER_ADVERTISE
  MdnsUpdate();
#endif  // WEBSERVER_ADVERTISE
#endif  // USE_WEBSERVER
#endif  // USE_DISCOVERY
#endif  // ESP8266

  OsWatchLoop();
  ButtonLoop();
  SwitchLoop();
#ifdef USE_DEVICE_GROUPS
  DeviceGroupsLoop();
#endif  // USE_DEVICE_GROUPS
  BacklogLoop();

  static uint32_t state_50msecond = 0;             // State 50msecond timer
  if (TimeReached(state_50msecond)) {
    SetNextTimeInterval(state_50msecond, 50);
    LoopTimedCmnd();
#ifdef ROTARY_V1
    RotaryHandler();
#endif  // ROTARY_V1
    XdrvXsnsCall(FUNC_EVERY_50_MSECOND);
  }

  static uint32_t state_100msecond = 0;            // State 100msecond timer
  if (TimeReached(state_100msecond)) {
    SetNextTimeInterval(state_100msecond, 100);
    Every100mSeconds();
    XdrvXsnsCall(FUNC_EVERY_100_MSECOND);
  }

  static uint32_t state_250msecond = 0;            // State 250msecond timer
  if (TimeReached(state_250msecond)) {
    SetNextTimeInterval(state_250msecond, 250);
    Every250mSeconds();
    XdrvXsnsCall(FUNC_EVERY_250_MSECOND);
  }

  static uint32_t state_second = 0;                // State second timer
  if (TimeReached(state_second)) {
    SetNextTimeInterval(state_second, 1000);
    PerformEverySecond();
    XdrvCall(FUNC_ACTIVE);
    XdrvXsnsCall(FUNC_EVERY_SECOND);
  }

  if (!InfinitekGlobal.serial_local) { SerialInput(); }
#ifdef ESP32
  if (!tasconsole_serial) { TasConsoleInput(); }
#endif  // ESP32

#ifdef ESP8266
#ifdef USE_ARDUINO_OTA
  ArduinoOtaLoop();
#endif  // USE_ARDUINO_OTA
#endif  // ESP8266

#ifndef SYSLOG_UPDATE_SECOND
  SyslogAsync(false);
#endif  // SYSLOG_UPDATE_SECOND
}

void loop(void) {
  uint32_t my_sleep = millis();

  Scheduler();

  uint32_t my_activity = millis() - my_sleep;

  if (InfinitekGlobal.skip_sleep) {
    InfinitekGlobal.skip_sleep--;                    // Temporarily skip sleep to handle imminent interrupts outside interrupt handler
  } else {
    if (Settings->flag3.sleep_normal) {            // SetOption60 - Enable normal sleep instead of dynamic sleep
      //  yield();                                 // yield == delay(0), delay contains yield, auto yield in loop
      SleepDelay(InfinitekGlobal.sleep);             // https://github.com/esp8266/Arduino/issues/2021
    } else {
      if (my_activity < (uint32_t)InfinitekGlobal.sleep) {
        SleepDelay((uint32_t)InfinitekGlobal.sleep - my_activity);  // Provide time for background tasks like wifi
      } else {
        if (InfinitekGlobal.global_state.network_down) {
          SleepDelay(my_activity /2);              // If wifi down and my_activity > setoption36 then force loop delay to 1/2 of my_activity period
        }
      }
    }
  }
  if (!my_activity) { my_activity++; }             // We cannot divide by 0
  uint32_t loop_delay = InfinitekGlobal.sleep;
  if (!loop_delay) { loop_delay++; }               // We cannot divide by 0
  uint32_t loops_per_second = 1000 / loop_delay;   // We need to keep track of this many loops per second
  uint32_t this_cycle_ratio = 100 * my_activity / loop_delay;
  InfinitekGlobal.loop_load_avg = InfinitekGlobal.loop_load_avg - (InfinitekGlobal.loop_load_avg / loops_per_second) + (this_cycle_ratio / loops_per_second); // Take away one loop average away and add the new one
    // --- Sensor Reading and Publishing ---
  #if USE_DHT11 || USE_DHT22
    float dhtTemp = dht.readTemperature();
    float dhtHum = dht.readHumidity();
  #endif
  #if USE_BME280
    float bmeTemp = bme280.readTemperature();
    float bmeHum = bme280.readHumidity();
    float bmePres = bme280.readPressure();
  #endif
  #if USE_BMP280
    float bmpTemp = bmp280.readTemperature();
    float bmpPres = bmp280.readPressure();
  #endif
  #if USE_BME680
    float bme680Temp = bme680.temperature;
    float bme680Hum = bme680.humidity;
    float bme680Gas = bme680.gas_resistance;
  #endif
  #if USE_SHT3X
    float sht3xTemp = sht3x.getTemperature();
    float sht3xHum = sht3x.getHumidity();
  #endif
  #if USE_SHT4X
    float sht4xTemp = sht4x.getTemperature();
    float sht4xHum = sht4x.getHumidity();
  #endif
  #if USE_CCS811
    uint16_t ccs811CO2 = ccs811.geteCO2();
    uint16_t ccs811TVOC = ccs811.getTVOC();
  #endif
  #if USE_SGP30
    uint16_t sgp30CO2 = sgp30.eCO2;
    uint16_t sgp30TVOC = sgp30.TVOC;
  #endif
  #if USE_SCD30
    float scd30CO2 = scd30.getCO2();
    float scd30Temp = scd30.getTemperature();
    float scd30Hum = scd30.getHumidity();
  #endif
  #if USE_MHZ19
    int mhz19CO2 = mhz19.getCO2();
  #endif
  #if USE_PMS5003
    PMS::DATA pmsData;
    if (pms.readUntil(pmsData)) {
      int pmsPM25 = pmsData.PM_AE_UG_2_5;
      int pmsPM10 = pmsData.PM_AE_UG_10_0;
    }
  #endif
    // --- Power & Energy Monitoring ---
  #if USE_HLW8012
    float hlwPower = hlw8012.getPower();
    float hlwVoltage = hlw8012.getVoltage();
    float hlwCurrent = hlw8012.getCurrent();
  #endif
  #if USE_BL0937
    float blPower = bl0937.getPower();
    float blVoltage = bl0937.getVoltage();
    float blCurrent = bl0937.getCurrent();
  #endif
  #if USE_INA219
    float ina219Voltage = ina219.getBusVoltage_V();
    float ina219Current = ina219.getCurrent_mA();
  #endif
  #if USE_INA3221
    float ina3221Voltage = ina3221.getBusVoltage_V(1);
    float ina3221Current = ina3221.getCurrent_mA(1);
  #endif
  #if USE_PZEM004T
    float pzemVoltage = pzem.voltage();
    float pzemCurrent = pzem.current();
    float pzemPower = pzem.power();
  #endif

    // --- Motion & Presence ---
  #if USE_PIR
    bool pirDetected = digitalRead(PIR_PIN);
  #endif
  #if USE_RADAR
    bool radarDetected = radar.isPresence();
  #endif
  #if USE_ULTRASONIC
    float distance = ultrasonic.readDistanceCM();
  #endif

    // --- Light & Color ---
  #if USE_BH1750
    float lux = bh1750.readLightLevel();
  #endif
  #if USE_TSL2561
    float tslLux = tsl2561.getLux();
  #endif
  #if USE_TSL2591
    float tsl2591Lux = tsl2591.getLux();
  #endif
  #if USE_APDS9960
    int color = apds9960.readColor();
  #endif

    // --- Switches & Buttons ---
  #if USE_SWITCH
    bool switchState = digitalRead(SWITCH_PIN);
  #endif
  #if USE_TOUCH
    bool touchState = touch.isTouched();
  #endif

    // --- Virtual Sensors ---
    int wifiRSSI = WiFi.RSSI();
    unsigned long uptime = millis() / 1000;
    unsigned long heap = ESP.getFreeHeap();

    // --- Advanced / Special Sensors ---
  #if USE_GPS
    String gpsLocation = gps.getLocation();
  #endif
  #if USE_RFID
    String rfidTag = rfid.readTag();
  #endif
  #if USE_CAMERA
    camera.capture();
  #endif

    // --- MQTT Publishing Examples ---
  #ifdef USE_MQTT
    // Power sensor example
    #if USE_HLW8012
      char hlwPayload[64];
      snprintf(hlwPayload, sizeof(hlwPayload), "{\"power\":%.2f,\"voltage\":%.2f,\"current\":%.2f}", hlwPower, hlwVoltage, hlwCurrent);
      MqttPublishPayload("infinitek/sensor/hlw8012", hlwPayload);
    #endif
    // Motion sensor example
    #if USE_PIR
      char pirPayload[32];
      snprintf(pirPayload, sizeof(pirPayload), "{\"motion\":%d}", pirDetected);
      MqttPublishPayload("infinitek/sensor/pir", pirPayload);
    #endif
    // Light sensor example
    #if USE_BH1750
      char luxPayload[32];
      snprintf(luxPayload, sizeof(luxPayload), "{\"lux\":%.2f}", lux);
      MqttPublishPayload("infinitek/sensor/bh1750", luxPayload);
    #endif
    // Switch example
    #if USE_SWITCH
      char switchPayload[32];
      snprintf(switchPayload, sizeof(switchPayload), "{\"switch\":%d}", switchState);
      MqttPublishPayload("infinitek/sensor/switch", switchPayload);
    #endif
    // Virtual sensor example
      char sysPayload[64];
      snprintf(sysPayload, sizeof(sysPayload), "{\"rssi\":%d,\"uptime\":%lu,\"heap\":%lu}", wifiRSSI, uptime, heap);
      MqttPublishPayload("infinitek/sensor/system", sysPayload);
  #endif
    #if USE_MQTT
        // Example: Publish DHT sensor data to MQTT topic
    #if defined(USE_DHT) || defined(USE_DHT11) || defined(USE_DHT22)
          char dhtPayload[64];
          snprintf(dhtPayload, sizeof(dhtPayload), "{\"temperature\":%.2f,\"humidity\":%.2f}", dhtTemp, dhtHum);
          MqttPublishPayload("infinitek/sensor/dht", dhtPayload);
        #endif
        // Add similar blocks for other sensors as needed
    #endif
    #if USE_MATTER
      // Example: Publish sensor data to Matter endpoint
      // Replace with actual Matter SDK publish function
      // matter_publish("infinitek/sensor/dht", dhtTemp, dhtHum);
    #endif

    // --- Publish sensor data via MQTT/Matter ---
  #if USE_MQTT
    // Publish sensor data to MQTT topic
    if (InfinitekGlobal.mqtt_enabled) {
      MqttPublishSensorData();
    }
  #endif
  #if USE_MATTER
    // Publish sensor data to Matter endpoint
    if (InfinitekGlobal.matter_enabled) {
      MatterPublishSensorData();
    }
  #endif
}
