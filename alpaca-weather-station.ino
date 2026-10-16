#include <ArduinoRS485.h>
#include <DFRobot_MLX90614.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <aWOT.h>
#include <api/deprecated-avr-comp/avr/dtostrf.h>
#include <cmath>
#include <malloc.h>  // for mallinfo
#include <strings.h> // for strncasecmp
#include <FlashIAPBlockDevice.h>
#include "FlashIAPLimits.h"
#include <TDBStore.h>
#include "opta_info.h"
#include <drivers/Watchdog.h>
#include <drivers/ResetReason.h>

OptaBoardInfo *info;
OptaBoardInfo *boardInfo();

using namespace mbed;

const unsigned long WATCHDOG_TIMEOUT = 20000; // 20 seconds watchdog timeout

// Ethernet.begin waits this long for a link, must stay well below WATCHDOG_TIMEOUT
const unsigned long ETHERNET_TIMEOUT = 10000;

// loop() only kicks the watchdog while the sensor thread completes a cycle in
// this time, so a stuck sensor thread resets the board and the relay goes unsafe
const unsigned long SENSOR_HEARTBEAT_TIMEOUT = 15000;

// observing conditions older than this are reported as an error, not as a value
const unsigned long STALE_DATA_TIMEOUT = 60000;

// Debug flag - set to true for Serial debugging. Serial writes block while a
// host has the USB port open but does not read it, so keep this off in use.
const bool DEBUG = false;

// For storing the safety limits
// Get limits of the In Application Program (IAP) flash, ie. the internal MCU flash.
auto iapLimits{getFlashIAPLimits()};

// Create a block device on the available space of the FlashIAP
FlashIAPBlockDevice blockDevice(iapLimits.start_address, iapLimits.available_size);

// Create a key-value store on the Flash IAP block device
TDBStore store(&blockDevice);

// set when the store cannot be used, limits then only live in RAM
const char *storeError = NULL;

// struct for weather limits
struct WeatherLimits
{
  float AT_min = 0;  // Air temperature low limit C
  float AT_max = 50; // Air temperature high limit C
  float AH_min = 0;  // Air humidity low limit %RH
  float AH_max = 80; // Air humidity high limit %RH
  // float AP_min; // Barometric pressure low limit Pa
  // float AP_max; // Barometric pressure high limit Pa
  float LX_max = 10000; // Light intensity high limit Lux
  float SM_max = 20;    // Gust wind speed high limit m/s (default), km/h, mph, knots
  float SA_max = 15;    // Average wind speed high limit m/s (default), km/h, mph, knots
  // float RA_max; // Accumulated rainfall high limit mm (default), in
  // float RD_max; // Duration of rainfall high limit s
  // float RI_max; // Rainfall intensity high limit mm/h (default), in/h
  // float Rp_max; // Maximum rainfall intensity high limit mm/h (default), in/h
  float ST_min = -80; // Sky temperature low limit C
  float ST_max = -30; // Sky temperature high limit C
  // float RST_min = -80;             // Relative sky temperature low limit C
  float RST_max = -20;            // Relative sky temperature high limit C
  int safety_false_duration = 10; // Time in seconds to keep isSafe false if not safe
};

// Current weather limits, to be updated during runtime or submitted via the web interface
WeatherLimits currentLimits;

// An example key name for the stats on the store
const char limitsKey[]{"limits"};

// Rolling median filter for outlier detection
struct RollingMedian {
  static const int WINDOW_SIZE = 5;
  float values[WINDOW_SIZE];
  int count = 0;
  int index = 0;

  void add(float value) {
    values[index] = value;
    index = (index + 1) % WINDOW_SIZE;
    if (count < WINDOW_SIZE) count++;
  }

  float getMedian() {
    if (count == 0) return 0;

    // Copy values to sort
    float sorted[WINDOW_SIZE];
    for (int i = 0; i < count; i++) {
      sorted[i] = values[i];
    }

    // Simple bubble sort (fine for 5 elements)
    for (int i = 0; i < count - 1; i++) {
      for (int j = 0; j < count - i - 1; j++) {
        if (sorted[j] > sorted[j + 1]) {
          float temp = sorted[j];
          sorted[j] = sorted[j + 1];
          sorted[j + 1] = temp;
        }
      }
    }

    // Return median
    if (count % 2 == 0) {
      return (sorted[count/2 - 1] + sorted[count/2]) / 2.0;
    } else {
      return sorted[count/2];
    }
  }

  bool isFull() {
    return count == WINDOW_SIZE;
  }

  void reset() {
    count = 0;
    index = 0;
  }
};

// S700 Weather Station values, in the order of WS_KEYS
enum WeatherStationValue
{
  WS_AT,   // AT Air temperature C
  WS_AH,   // AH Air humidity %RH
  WS_AP,   // AP Barometric pressure Pa
  WS_LX,   // LX Light intensity Lux
  WS_DN,   // DN Minimum wind direction deg
  WS_DM,   // Dm Maximum wind direction deg
  WS_DA,   // DA Average wind direction deg
  WS_SN,   // SN Minimum wind speed m/s (default), km/h, mph, knots
  WS_SM,   // SM Maximum wind speed m/s (default), km/h, mph, knots
  WS_SA,   // SA Average wind speed m/s (default), km/h, mph, knots
  WS_RA,   // RA Accumulated rainfall mm (default), in
  WS_RD,   // RD Duration of rainfall s
  WS_RI,   // RI Rainfall intensity mm/h (default), in/h
  WS_RP,   // Rp Maximum rainfall intensity mm/h (default), in/h
  WS_HT,   // HT Heating temperature C
  WS_TILT, // TILT Fall detection, 0 when upright
  WS_COUNT
};

// S700 reply keys, compared without case because the manual and the station differ (Dm/DM, Rp/RP)
const char *const WS_KEYS[WS_COUNT] = {"AT", "AH", "AP", "LX", "DN", "DM", "DA", "SN", "SM",
                                       "SA", "RA", "RD", "RI", "RP", "HT", "TILT"};

// S700 Weather Station values, filtered except TILT
float WSValues[WS_COUNT];

// Create rolling median filters for all weather station values, TILT is not filtered (binary status)
RollingMedian wsFilters[WS_TILT];

// Create rolling median filters for sky temperature sensors
RollingMedian skyTempFilter;
RollingMedian mlx1SkyFilter;
RollingMedian mlx2SkyFilter;
RollingMedian mlx1AmbientFilter;
RollingMedian mlx2AmbientFilter;

// Create rolling median filter for rain sensor analog reading
RollingMedian rainSensorFilter;

// Track consecutive read failures
int consecutiveReadFailures = 0;
const int MAX_CONSECUTIVE_FAILURES = 3;

// RS485
constexpr auto baudrate{9600};
// Calculate preDelay and postDelay in microseconds for stable RS-485 transmission
constexpr auto bitduration{1.f / baudrate};
constexpr auto wordlen{9.6f}; // OR 10.0f depending on the channel configuration
constexpr auto preDelayBR{bitduration * wordlen * 3.5f * 1e6};
constexpr auto postDelayBR{bitduration * wordlen * 3.5f * 1e6};

// Thread for reading sensors. The default 4 kB stack has little margin.
static rtos::Thread SensorReadThread(osPriorityNormal, 8 * 1024);

// State shared between the sensor thread and the web server (loop) thread.
// Each value is one 32-bit write, so a reader never sees half of a value.
volatile bool isSafe = false;
volatile uint32_t sensorHeartbeat = 0;  // millis() at the end of the last sensor cycle
volatile uint32_t lastWeatherUpdate = 0; // millis() of the last valid S700 reading
volatile uint32_t lastSkyUpdate = 0;     // millis() of the last valid MLX90614 reading
volatile bool weatherDataSeen = false;
volatile bool skyDataSeen = false;
const char *volatile sensorError = NULL; // why the last cycle failed, points to a string literal

// Safety pins
constexpr int SAFETY_RELAY = D0;
constexpr int SAFETY_RELAY_LED = LED_D0;

// Rain sensor
volatile bool rainSensorSafe = true;
constexpr int rainSensorSafePin = A7;
constexpr int rainSensorSignalHighPin = A6;

// SkyTemperature sensor
DFRobot_MLX90614_I2C mlx1(0x5A, &Wire, 11, 12); // I2C address is 0x5A
DFRobot_MLX90614_I2C mlx2(0x5B, &Wire, 11, 12); // I2C address is 0x5B
float relativeSkyTemperature;                   // Relative sky temperature in Celsius
float skyTemperature;                           // Sky temperature in Celsius
float mlx1SkyTemperature;                       // Sky temperature in Celsius
float mlx2SkyTemperature;                       // Sky temperature in Celsius
float mlx1AmbientTemperature;                   // Ambient temperature in Celsius
float mlx2AmbientTemperature;                   // Ambient temperature in Celsius

// ObservingConditions sensor properties
enum SensorId
{
  SENSOR_DEWPOINT,
  SENSOR_HUMIDITY,
  SENSOR_PRESSURE,
  SENSOR_RAINRATE,
  SENSOR_SKYBRIGHTNESS,
  SENSOR_SKYTEMPERATURE,
  SENSOR_TEMPERATURE,
  SENSOR_WINDDIRECTION,
  SENSOR_WINDGUST,
  SENSOR_WINDSPEED,
  SENSOR_CLOUDCOVER, // not implemented
  SENSOR_SKYQUALITY, // not implemented
  SENSOR_STARFWHM,   // not implemented
  SENSOR_COUNT
};

struct Sensor
{
  const char *name;        // ASCOM sensor name, the Alpaca member is the same name in lower case
  const char *description; // NULL if the sensor is not implemented
  bool fromSkySensors;     // true for the MLX90614 sensors, false for the S700
};

const Sensor SENSORS[SENSOR_COUNT] = {
    {"DewPoint", "Calculated from SenseCAP S700 air temperature and humidity", false},
    {"Humidity", "SenseCAP S700 relative humidity", false},
    {"Pressure", "SenseCAP S700 barometric pressure", false},
    {"RainRate", "SenseCAP S700 rain intensity and Kemo M152K rain sensor", false},
    {"SkyBrightness", "SenseCAP S700 light intensity", false},
    {"SkyTemperature", "Two MLX90614 infrared sensors, warmest reading", true},
    {"Temperature", "SenseCAP S700 air temperature", false},
    {"WindDirection", "SenseCAP S700 average wind direction", false},
    {"WindGust", "SenseCAP S700 maximum wind speed", false},
    {"WindSpeed", "SenseCAP S700 average wind speed", false},
    {"CloudCover", NULL, false},
    {"SkyQuality", NULL, false},
    {"StarFWHM", NULL, false},
};

// declared early for the prototypes the Arduino builder adds
struct AlpacaReply;

// ASCOM connected state, one per device. While disconnected, sensor values
// return NotConnected. IsSafe does not depend on it, so a client that
// disconnects does not make the state unsafe for other clients. They start
// connected, so a client does not see a disconnected device after a
// watchdog reset, and scripts that never connect still work.
bool safetyMonitorConnected = true;
bool conditionsConnected = true;

// Alpaca error numbers
constexpr int32_t ERROR_NOT_IMPLEMENTED = 0x400;
constexpr int32_t ERROR_INVALID_VALUE = 0x401;
constexpr int32_t ERROR_NOT_CONNECTED = 0x407;
constexpr int32_t ERROR_ACTION_NOT_IMPLEMENTED = 0x40C;
constexpr int32_t ERROR_DRIVER = 0x500;

const char *DRIVER_VERSION = "0.1.0";
const char *SAFETY_MONITOR_NAME = "Safety monitor by PPP";
const char *CONDITIONS_NAME = "Weather station by PPP";
const unsigned int ALPACA_PORT = 80;
const unsigned int DISCOVERY_PORT = 32227;

// incremented on every Alpaca response, only used by the loop thread
uint32_t serverTransactionID = 0;

// why the board last restarted, read once in setup
reset_reason_t resetReason = RESET_REASON_UNKNOWN;

// the media access control (ethernet hardware) address
byte mac[6];
// the IP address
byte ip[] = {192, 168, 1, 12};

// unique IDs for the management API, filled from the MAC address in setup
char safetyMonitorUniqueID[40];
char conditionsUniqueID[40];

// Create an Ethernet server
EthernetServer server(ALPACA_PORT);
EthernetUDP discoveryUdp;
Application app;

void debugPrint(const char *message)
{
  if (DEBUG)
  {
    Serial.println(message);
  }
}

// ISR for fully closed sensor // update 22-02-2025 - pins don't like 12V?
// void rainSensorSafeISR()
// {
//   rainSensorSafe = digitalRead(rainSensorSafePin) == LOW;
//   Serial.println("Rain detection change");
// }

void resetFilters()
{
  for (int i = 0; i < WS_TILT; i++)
  {
    wsFilters[i].reset();
  }
  skyTempFilter.reset();
  mlx1SkyFilter.reset();
  mlx2SkyFilter.reset();
  mlx1AmbientFilter.reset();
  mlx2AmbientFilter.reset();
  rainSensorFilter.reset();
}

// Read all sensors and sets safety flag - runs in a separate thread
void readSensors()
{
  auto time_since_not_safe = rtos::Kernel::Clock::now();
  const int maxAttempts = 3;

  while (true)
  {
    digitalWrite(LED_BUILTIN, HIGH);

    // read weather station data with retry logic
    bool weather_station_data_received = false;
    for (int attempt = 0; attempt < maxAttempts && !weather_station_data_received; attempt++)
    {
      weather_station_data_received = readWeatherStation();
      if (!weather_station_data_received && attempt < maxAttempts - 1)
      {
        debugPrint("Weather station read failed, retrying");
        rtos::ThisThread::sleep_for(100); // Small delay between retries
      }
    }

    // Track consecutive failures for weather station
    if (!weather_station_data_received)
    {
      consecutiveReadFailures++;
      debugPrint("Weather station read failed");
    }
    else
    {
      consecutiveReadFailures = 0; // Reset on success
      lastWeatherUpdate = millis();
      weatherDataSeen = true;
    }

    // read sky temperature with retry and validation
    bool sky_temp_data_valid = false;
    float mlx1Sky = 0, mlx2Sky = 0, mlx1Amb = 0, mlx2Amb = 0;

    for (int attempt = 0; attempt < maxAttempts && !sky_temp_data_valid; attempt++)
    {
      mlx1Sky = mlx1.getObjectTempCelsius();
      mlx2Sky = mlx2.getObjectTempCelsius();
      mlx1Amb = mlx1.getAmbientTempCelsius();
      mlx2Amb = mlx2.getAmbientTempCelsius();

      // Basic sanity check on readings (-80 to 85°C for sky and ambient)
      if (mlx1Sky > -80 && mlx1Sky < 85 && mlx2Sky > -80 && mlx2Sky < 85 &&
          mlx1Amb > -80 && mlx1Amb < 85 && mlx2Amb > -80 && mlx2Amb < 85)
      {
        sky_temp_data_valid = true;

        // Add raw readings to filters
        mlx1SkyFilter.add(mlx1Sky);
        mlx2SkyFilter.add(mlx2Sky);
        mlx1AmbientFilter.add(mlx1Amb);
        mlx2AmbientFilter.add(mlx2Amb);

        // Get filtered values
        mlx1SkyTemperature = mlx1SkyFilter.getMedian();
        mlx2SkyTemperature = mlx2SkyFilter.getMedian();
        mlx1AmbientTemperature = mlx1AmbientFilter.getMedian();
        mlx2AmbientTemperature = mlx2AmbientFilter.getMedian();

        // Calculate sky temperature as max of both filtered sensors
        float maxSkyTemp = std::max(mlx1SkyTemperature, mlx2SkyTemperature);
        skyTempFilter.add(maxSkyTemp);
        skyTemperature = skyTempFilter.getMedian();
        relativeSkyTemperature = skyTemperature - WSValues[WS_AT];
        lastSkyUpdate = millis();
        skyDataSeen = true;
      }
      else if (attempt < maxAttempts - 1)
      {
        debugPrint("Sky temp invalid, retrying");
        rtos::ThisThread::sleep_for(100);
      }
    }

    if (!sky_temp_data_valid)
    {
      consecutiveReadFailures++;
      debugPrint("Sky temp failure");
    }

    // work out the new state in locals, so the web server never sees a
    // half-finished result
    bool safe = false;
    const char *error = NULL;

    // If too many consecutive failures, mark unsafe and reset filters
    if (consecutiveReadFailures >= MAX_CONSECUTIVE_FAILURES)
    {
      debugPrint("Too many consecutive sensor failures - marking UNSAFE");
      error = "Sensor communication failed";
      resetFilters();
    }
    else
    {
      // Normal safety check
      safe = weather_station_data_received && sky_temp_data_valid && safetyCheck();
      if (!weather_station_data_received)
      {
        error = "No data received from weather station";
      }
      else if (!sky_temp_data_valid)
      {
        error = "No valid reading from sky temperature sensors";
      }
    }

    // if not safe, keep isSafe false for configured duration, but continue to read sensors
    if (!safe)
    {
      time_since_not_safe = rtos::Kernel::Clock::now();
    }
    safe = safe && (rtos::Kernel::Clock::now() - time_since_not_safe > std::chrono::seconds(currentLimits.safety_false_duration));

    if (safe)
    {
      digitalWrite(SAFETY_RELAY, HIGH);
      digitalWrite(SAFETY_RELAY_LED, HIGH);
      consecutiveReadFailures = 0; // Reset on safe condition restored
    }
    else
    {
      digitalWrite(SAFETY_RELAY, LOW);
      digitalWrite(SAFETY_RELAY_LED, LOW);
    }

    isSafe = safe;
    sensorError = error;
    sensorHeartbeat = millis();

    // the built-in LED flashes while the weather station answers, and stays on when it does not
    rtos::ThisThread::sleep_for(500);
    if (weather_station_data_received)
    {
      digitalWrite(LED_BUILTIN, LOW);
    }
    rtos::ThisThread::sleep_for(500);
  }
}

// Read data from the S700 weather station
bool readWeatherStation()
{
  const unsigned long timeout = 500; // 500 ms timeout
  const int maxChars = 256;          // Maximum characters to read
  char buffer[maxChars + 1];         // +1 for null terminator
  int charCount = 0;

  // drop bytes from a late reply or from line noise, so they do not mix with this reply
  while (RS485.available())
  {
    RS485.read();
  }

  auto startTime = rtos::Kernel::Clock::now();

  RS485.noReceive();
  RS485.beginTransmission();
  RS485.println("0XA;G0?");
  RS485.flush();
  RS485.endTransmission();
  RS485.receive();

  while (rtos::Kernel::Clock::now() - startTime < std::chrono::milliseconds(timeout))
  {
    if (RS485.available())
    {
      char c = RS485.read();
      buffer[charCount++] = c;
      // stop at the end of the line, or when the buffer is full (line noise)
      if (c == '\n' || charCount >= maxChars)
      {
        break;
      }
    }
  }

  buffer[charCount] = '\0'; // Null-terminate the string

  // DEV ^ uncomment below to simulate reading data from the S700 weather station
  // strcpy(buffer, "0XA;AT=23.6;AH=56.4;AP=100819.1;LX=93.0;DN=0.0;DM=0.0;DA=0.0;SN=0.0;SM=0.0;SA=0.0;RA=1.4;RD=60.0;RI=0.0;RP=0.0;HT=-38.4;TILT=0\r\n");
  // charCount = strlen(buffer);

  if (charCount == 0)
  {
    debugPrint("No data received within timeout");
    return false;
  }

  if (!parseWeatherStation(buffer))
  {
    debugPrint("Invalid response received");
    return false;
  }
  return true;
}

// Parse a reply such as "0XA;AT=23.6;AH=56.4;...;TILT=0\r\n" and update WSValues.
// The reply is changed in place. Returns false unless every value is present.
bool parseWeatherStation(char *reply)
{
  size_t length = strlen(reply);
  if (strncmp(reply, "0XA;", 4) != 0 || length < 6 || strcmp(reply + length - 2, "\r\n") != 0)
  {
    return false;
  }
  reply[length - 2] = '\0';

  // Temporary storage for new readings
  float values[WS_COUNT];
  uint32_t found = 0;

  char *save;
  for (char *item = strtok_r(reply + 4, ";", &save); item != NULL; item = strtok_r(NULL, ";", &save))
  {
    char *equals = strchr(item, '=');
    if (equals == NULL)
    {
      return false;
    }
    *equals = '\0';

    for (int i = 0; i < WS_COUNT; i++)
    {
      if (strcasecmp(item, WS_KEYS[i]) == 0)
      {
        char *end;
        values[i] = strtof(equals + 1, &end);
        if (end == equals + 1 || !std::isfinite(values[i]))
        {
          return false;
        }
        found |= 1u << i;
        break;
      }
    }
  }

  if (found != (1u << WS_COUNT) - 1)
  {
    return false;
  }

  // Add all raw readings to their filters and keep the filtered (median) values
  for (int i = 0; i < WS_TILT; i++)
  {
    wsFilters[i].add(values[i]);
    WSValues[i] = wsFilters[i].getMedian();
  }

  // TILT is not filtered (binary status)
  WSValues[WS_TILT] = values[WS_TILT];

  return true;
}

// Safety check of weather station data and connected sensors
bool safetyCheck()
{
  bool safe = true;

  // rainSensorSafe - use filtered value to avoid noise
  int rainSensorReading = analogRead(rainSensorSafePin);
  rainSensorFilter.add(rainSensorReading);
  float filteredRainSensor = rainSensorFilter.getMedian();

  if (filteredRainSensor > 10)
  {
    debugPrint("Rain sensor active");
    rainSensorSafe = false;
    safe = false;
  }
  else
  {
    rainSensorSafe = true;
  }

  // the station has fallen over, its readings cannot be trusted
  if (WSValues[WS_TILT] != 0)
  {
    debugPrint("Weather station tilted");
    safe = false;
  }

  // check if any values are outside of the safety limits
  // temperature
  if (WSValues[WS_AT] < currentLimits.AT_min || WSValues[WS_AT] > currentLimits.AT_max)
  {
    debugPrint("Temperature out of range");
    safe = false;
  }
  // humidity
  if (WSValues[WS_AH] < currentLimits.AH_min || WSValues[WS_AH] > currentLimits.AH_max)
  {
    debugPrint("Humidity out of range");
    safe = false;
  }
  // pressure
  // if (WSValues[WS_AP] < currentLimits.AP_min || WSValues[WS_AP] > currentLimits.AP_max) {
  //   debugPrint("Pressure out of range");
  //   safe = false;
  // }

  // light intensity
  if (WSValues[WS_LX] > currentLimits.LX_max)
  {
    debugPrint("Light intensity out of range");
    safe = false;
  }

  // gust wind speed
  if (WSValues[WS_SM] > currentLimits.SM_max)
  {
    debugPrint("Wind speed out of range");
    safe = false;
  }

  // average wind speed
  if (WSValues[WS_SA] > currentLimits.SA_max)
  {
    debugPrint("Wind speed out of range");
    safe = false;
  }

  // these require user resetting (see manual)
  // // accumulated rainfall
  // if (WSValues[WS_RA] > currentLimits.RA_max) {
  //   debugPrint("Rainfall out of range");
  //   safe = false;
  // }

  // // duration of rainfall
  // if (WSValues[WS_RD] > currentLimits.RD_max) {
  //   debugPrint("Rainfall duration out of range");
  //   safe = false;
  // }

  // rainfall intensity
  if (WSValues[WS_RI] > 0)
  {
    debugPrint("Rainfall intensity out of range");
    safe = false;
  }

  // maximum rainfall intensity
  if (WSValues[WS_RP] > 0)
  {
    debugPrint("Rainfall intensity out of range");
    safe = false;
  }

  // sky temperature
  if (skyTemperature < currentLimits.ST_min || skyTemperature > currentLimits.ST_max)
  {
    debugPrint("Sky temperature out of range");
    safe = false;
  }

  // relative sky temperature
  if (relativeSkyTemperature > currentLimits.RST_max)
  {
    debugPrint("Relative sky temperature out of range");
    safe = false;
  }

  return safe;
}

// Alpaca parameter names are case insensitive. GET parameters are in the
// query string, PUT parameters are in the form encoded body.
void read_params(Request &req, char *params, int length)
{
  params[0] = '\0';

  if (req.method() == Request::GET)
  {
    strncpy(params, req.query(), length - 1);
    params[length - 1] = '\0';
  }
  else
  {
    int toRead = min(req.left(), length - 1);
    int count = req.readBytes(params, toRead);
    params[count] = '\0';
  }
}

bool find_param(const char *params, const char *name, char *value, int valueLength)
{
  size_t nameLength = strlen(name);
  const char *position = params;

  while (*position)
  {
    const char *end = strchr(position, '&');
    if (end == NULL)
    {
      end = position + strlen(position);
    }

    const char *equals = (const char *)memchr(position, '=', end - position);
    if (equals != NULL && (size_t)(equals - position) == nameLength && strncasecmp(position, name, nameLength) == 0)
    {
      int count = min((int)(end - equals - 1), valueLength - 1);
      memcpy(value, equals + 1, count);
      value[count] = '\0';
      return true;
    }

    position = *end ? end + 1 : end;
  }

  return false;
}

// Returns true if the parameter is "true" or "false", and sets result
bool find_bool_param(const char *params, const char *name, bool &result)
{
  char value[8];
  if (!find_param(params, name, value, sizeof(value)))
  {
    return false;
  }
  if (strcasecmp(value, "true") == 0)
  {
    result = true;
    return true;
  }
  if (strcasecmp(value, "false") == 0)
  {
    result = false;
    return true;
  }
  return false;
}

// Returns true if the parameter is a number, and sets result
bool find_float_param(const char *params, const char *name, float &result)
{
  char value[24];
  if (!find_param(params, name, value, sizeof(value)) || value[0] == '\0')
  {
    return false;
  }
  char *end;
  result = strtof(value, &end);
  return *end == '\0' && std::isfinite(result);
}

// Format a float for JSON with 2 decimals
void format_float(char *buffer, float value)
{
  dtostrf(value, 1, 2, buffer);
}

// Reply to one Alpaca request
struct AlpacaReply
{
  enum Type
  {
    NONE,  // method without a return value
    BOOL,
    INT,
    FLOAT,
    STRING,
    JSON   // value printed as is, for arrays and objects
  };

  Type type = NONE;
  bool boolValue = false;
  int32_t intValue = 0;
  float floatValue = 0;
  const char *text = "";
  int32_t errorNumber = 0;
  const char *errorMessage = "";
  bool badRequest = false;

  void setBool(bool value)
  {
    type = BOOL;
    boolValue = value;
  }

  void setInt(int32_t value)
  {
    type = INT;
    intValue = value;
  }

  void setFloat(float value)
  {
    type = FLOAT;
    floatValue = value;
  }

  void setString(const char *value)
  {
    type = STRING;
    text = value;
  }

  void setJson(const char *value)
  {
    type = JSON;
    text = value;
  }

  void setError(int32_t number, const char *message)
  {
    errorNumber = number;
    errorMessage = message;
  }

  void setBadRequest(const char *message)
  {
    badRequest = true;
    errorMessage = message;
  }
};

void send_reply(Response &res, const char *params, AlpacaReply &reply)
{
  if (reply.badRequest)
  {
    res.status(400);
    res.set("Content-Type", "text/plain");
    res.print(reply.errorMessage);
    res.end();
    return;
  }

  char buffer[16];
  uint32_t clientTransactionID = 0;
  if (find_param(params, "ClientTransactionID", buffer, sizeof(buffer)))
  {
    clientTransactionID = strtoul(buffer, NULL, 10);
  }

  res.set("Content-Type", "application/json");
  res.print("{\"ClientTransactionID\":");
  res.print(clientTransactionID);
  // must be 1 or more, also after the counter wraps
  if (++serverTransactionID == 0)
  {
    serverTransactionID = 1;
  }
  res.print(",\"ServerTransactionID\":");
  res.print(serverTransactionID);
  res.print(",\"ErrorNumber\":");
  res.print(reply.errorNumber);
  res.print(",\"ErrorMessage\":\"");
  res.print(reply.errorMessage);
  res.print("\"");

  // a value is only sent when there is no error
  if (reply.errorNumber == 0 && reply.type != AlpacaReply::NONE)
  {
    res.print(",\"Value\":");
    switch (reply.type)
    {
    case AlpacaReply::BOOL:
      res.print(reply.boolValue ? "true" : "false");
      break;
    case AlpacaReply::INT:
      res.print(reply.intValue);
      break;
    case AlpacaReply::FLOAT:
      res.print(reply.floatValue, 2);
      break;
    case AlpacaReply::STRING:
      res.print("\"");
      res.print(reply.text);
      res.print("\"");
      break;
    default:
      res.print(reply.text);
      break;
    }
  }

  res.print("}");
  res.end();
}

// Members common to all device types. Returns false if the member is unknown.
bool common_member(const char *member, bool isPut, const char *params, bool &connected, const char *name,
                   const char *description, const char *deviceState, AlpacaReply &reply)
{
  if (isPut)
  {
    if (strcmp(member, "connected") == 0)
    {
      bool value;
      if (!find_bool_param(params, "Connected", value))
      {
        reply.setBadRequest("Connected parameter missing or invalid");
      }
      else
      {
        connected = value;
      }
    }
    else if (strcmp(member, "connect") == 0)
    {
      connected = true;
    }
    else if (strcmp(member, "disconnect") == 0)
    {
      connected = false;
    }
    else if (strcmp(member, "action") == 0)
    {
      reply.setError(ERROR_ACTION_NOT_IMPLEMENTED, "No actions are supported");
    }
    else if (strcmp(member, "commandblind") == 0 || strcmp(member, "commandbool") == 0 || strcmp(member, "commandstring") == 0)
    {
      reply.setError(ERROR_NOT_IMPLEMENTED, "Commands are not implemented");
    }
    else
    {
      return false;
    }
    return true;
  }

  if (strcmp(member, "connected") == 0)
  {
    reply.setBool(connected);
  }
  else if (strcmp(member, "connecting") == 0)
  {
    reply.setBool(false);
  }
  else if (strcmp(member, "name") == 0)
  {
    reply.setString(name);
  }
  else if (strcmp(member, "description") == 0)
  {
    reply.setString(description);
  }
  else if (strcmp(member, "driverinfo") == 0)
  {
    reply.setString("Arduino Opta weather station and safety monitor");
  }
  else if (strcmp(member, "driverversion") == 0)
  {
    reply.setString(DRIVER_VERSION);
  }
  else if (strcmp(member, "supportedactions") == 0)
  {
    reply.setJson("[]");
  }
  else if (strcmp(member, "devicestate") == 0)
  {
    reply.setJson(deviceState);
  }
  else
  {
    return false;
  }
  return true;
}

// Handle the safety monitor endpoint requests
void safetyMonitorEndPoint(Request &req, Response &res)
{
  char member[32];
  char params[256];
  req.route("member", member, sizeof(member));
  read_params(req, params, sizeof(params));
  bool isPut = req.method() == Request::PUT;

  // independent of the connected state, which all clients share
  bool safe = isSafe;

  char deviceState[48];
  snprintf(deviceState, sizeof(deviceState), "[{\"Name\":\"IsSafe\",\"Value\":%s}]", safe ? "true" : "false");

  AlpacaReply reply;

  if (common_member(member, isPut, params, safetyMonitorConnected, SAFETY_MONITOR_NAME,
                    "Weather based safety monitor", deviceState, reply))
  {
    // handled
  }
  else if (!isPut && strcmp(member, "interfaceversion") == 0)
  {
    reply.setInt(3);
  }
  else if (!isPut && strcmp(member, "issafe") == 0)
  {
    reply.setBool(safe);
  }
  else
  {
    reply.setBadRequest("Unknown safety monitor member");
  }

  send_reply(res, params, reply);
}

// Find a sensor by its ASCOM name or Alpaca member name, -1 if unknown
int find_sensor(const char *name)
{
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    if (strcasecmp(name, SENSORS[i].name) == 0)
    {
      return i;
    }
  }
  return -1;
}

// Dew point in C, NAN if the humidity is not valid
float dew_point(float temperature, float humidity)
{
  if (humidity <= 0)
  {
    return NAN;
  }
  // M1 from Górnicki, K., Winiczenko, R., Kaleta, A. and Choińska, A., 2017. Evaluation of models for the dew point temperature determination. Technical Sciences, 20(3), pp.241-257.
  float B = (log(humidity / 100) + ((17.269 * temperature) / (243.3 + temperature))) / 17.269;
  return (243.3 * B) / (1 - B);
}

// Current value of an implemented sensor, NAN if it cannot be calculated
float sensor_value(int id)
{
  switch (id)
  {
  case SENSOR_DEWPOINT:
    return dew_point(WSValues[WS_AT], WSValues[WS_AH]);
  case SENSOR_HUMIDITY:
    return WSValues[WS_AH];
  case SENSOR_PRESSURE:
    return WSValues[WS_AP] / 100; // Pa to hPa
  case SENSOR_RAINRATE:
  {
    float rate = WSValues[WS_RI];
    if (rate > 500)
    {
      rate = 500; // rainrate error value
    }
    if (!rainSensorSafe)
    {
      rate += 0.01; // add 0.01 mm/h if rain sensor is active since rainrate on the S700 is not that sensitive
    }
    return rate;
  }
  case SENSOR_SKYBRIGHTNESS:
    return WSValues[WS_LX];
  case SENSOR_SKYTEMPERATURE:
    return skyTemperature;
  case SENSOR_TEMPERATURE:
    return WSValues[WS_AT];
  case SENSOR_WINDDIRECTION:
    // ASCOM requires 0 when there is no wind
    return WSValues[WS_SA] > 0 ? WSValues[WS_DA] : 0;
  case SENSOR_WINDGUST:
    return WSValues[WS_SM];
  case SENSOR_WINDSPEED:
    return WSValues[WS_SA];
  }
  return NAN;
}

// Milliseconds since the last valid reading of a source, or -1 if there is none yet
long data_age(bool fromSkySensors)
{
  if (fromSkySensors ? !skyDataSeen : !weatherDataSeen)
  {
    return -1;
  }
  return millis() - (fromSkySensors ? lastSkyUpdate : lastWeatherUpdate);
}

bool data_fresh(bool fromSkySensors)
{
  long age = data_age(fromSkySensors);
  return age >= 0 && age < (long)STALE_DATA_TIMEOUT;
}

const char *stale_message(bool fromSkySensors)
{
  return fromSkySensors ? "No recent reading from the sky temperature sensors"
                        : "No recent reading from the weather station";
}

// DeviceState for ObservingConditions: the implemented sensors with a recent value
void conditions_state(char *state, size_t size)
{
  size_t used = snprintf(state, size, "[");
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    if (SENSORS[i].description == NULL || !data_fresh(SENSORS[i].fromSkySensors))
    {
      continue;
    }
    float value = sensor_value(i);
    if (!std::isfinite(value))
    {
      continue;
    }
    char number[24];
    format_float(number, value);
    used += snprintf(state + used, size - used, "%s{\"Name\":\"%s\",\"Value\":%s}",
                     used > 1 ? "," : "", SENSORS[i].name, number);
    if (used >= size)
    {
      break;
    }
  }
  if (used < size - 1)
  {
    strcat(state, "]");
  }
}

// Handle the observing conditions endpoint requests
void conditionsEndPoint(Request &req, Response &res)
{
  char member[32];
  char params[256];
  req.route("member", member, sizeof(member));
  read_params(req, params, sizeof(params));
  bool isPut = req.method() == Request::PUT;

  char deviceState[640] = "[]";
  if (!isPut && strcmp(member, "devicestate") == 0)
  {
    conditions_state(deviceState, sizeof(deviceState));
  }

  // for the members that return a JSON object
  char json[200];

  AlpacaReply reply;
  int sensor = find_sensor(member);

  if (common_member(member, isPut, params, conditionsConnected, CONDITIONS_NAME,
                    "SenseCAP S700 weather station with MLX90614 sky temperature sensors", deviceState, reply))
  {
    // handled
  }
  else if (!conditionsConnected && strcmp(member, "interfaceversion") != 0 && strcmp(member, "sensordescription") != 0)
  {
    // sensor values and the other operational members need a connection
    reply.setError(ERROR_NOT_CONNECTED, "Observing conditions device is not connected");
  }
  else if (isPut)
  {
    if (strcmp(member, "averageperiod") == 0)
    {
      // values are filtered by a rolling median, but not averaged over a time period
      float period;
      if (!find_float_param(params, "AveragePeriod", period))
      {
        reply.setBadRequest("AveragePeriod parameter missing or invalid");
      }
      else if (period != 0)
      {
        reply.setError(ERROR_INVALID_VALUE, "Only an average period of 0 is supported");
      }
    }
    else if (strcmp(member, "refresh") == 0)
    {
      // ASCOM requires MethodNotImplemented when refresh does nothing.
      // The sensors are read about every second anyway.
      reply.setError(ERROR_NOT_IMPLEMENTED, "Refresh is not implemented, sensors are read every second");
    }
    else
    {
      reply.setBadRequest("Unknown observing conditions member");
    }
  }
  else if (strcmp(member, "interfaceversion") == 0)
  {
    reply.setInt(2);
  }
  else if (strcmp(member, "averageperiod") == 0)
  {
    reply.setFloat(0);
  }
  else if (sensor >= 0)
  {
    const Sensor &s = SENSORS[sensor];
    float value = sensor_value(sensor);
    if (s.description == NULL)
    {
      reply.setError(ERROR_NOT_IMPLEMENTED, "This sensor is not available");
    }
    else if (!data_fresh(s.fromSkySensors))
    {
      reply.setError(ERROR_DRIVER, stale_message(s.fromSkySensors));
    }
    else if (!std::isfinite(value))
    {
      reply.setError(ERROR_DRIVER, "Value cannot be calculated from the current readings");
    }
    else
    {
      reply.setFloat(value);
    }
  }
  else if (strcmp(member, "sensordescription") == 0 || strcmp(member, "timesincelastupdate") == 0)
  {
    char name[24];
    bool isDescription = strcmp(member, "sensordescription") == 0;
    if (!find_param(params, "SensorName", name, sizeof(name)))
    {
      reply.setBadRequest("SensorName parameter missing");
    }
    else if (!isDescription && name[0] == '\0')
    {
      // an empty name asks for the most recent update of any sensor
      long weatherAge = data_age(false);
      long skyAge = data_age(true);
      long age = weatherAge < 0 ? skyAge : (skyAge < 0 ? weatherAge : min(weatherAge, skyAge));
      // ASCOM: a negative value means no valid reading has been received yet
      reply.setFloat(age < 0 ? -1 : age / 1000.0);
    }
    else
    {
      int id = find_sensor(name);
      if (id < 0)
      {
        reply.setError(ERROR_INVALID_VALUE, "Unknown sensor name");
      }
      else if (SENSORS[id].description == NULL)
      {
        reply.setError(ERROR_NOT_IMPLEMENTED, "This sensor is not available");
      }
      else if (isDescription)
      {
        reply.setString(SENSORS[id].description);
      }
      else
      {
        long age = data_age(SENSORS[id].fromSkySensors);
        reply.setFloat(age < 0 ? -1 : age / 1000.0);
      }
    }
  }
  // not part of the ASCOM interface, kept for existing scripts
  else if (strcmp(member, "relativeskytemperature") == 0)
  {
    if (!data_fresh(true) || !data_fresh(false))
    {
      reply.setError(ERROR_DRIVER, stale_message(!data_fresh(true)));
    }
    else
    {
      reply.setFloat(relativeSkyTemperature);
    }
  }
  else if (strcmp(member, "rawmlx") == 0)
  {
    if (!data_fresh(true))
    {
      reply.setError(ERROR_DRIVER, stale_message(true));
    }
    else
    {
      char sky1[16], ambient1[16], sky2[16], ambient2[16];
      format_float(sky1, mlx1SkyTemperature);
      format_float(ambient1, mlx1AmbientTemperature);
      format_float(sky2, mlx2SkyTemperature);
      format_float(ambient2, mlx2AmbientTemperature);
      snprintf(json, sizeof(json),
               "{\"mlx1\": {\"object\": %s, \"ambient\": %s}, \"mlx2\": {\"object\": %s, \"ambient\": %s}}",
               sky1, ambient1, sky2, ambient2);
      reply.setJson(json);
    }
  }
  else
  {
    reply.setBadRequest("Unknown observing conditions member");
  }

  send_reply(res, params, reply);
}

// Management API
void apiVersions(Request &req, Response &res)
{
  char params[256];
  read_params(req, params, sizeof(params));
  AlpacaReply reply;
  reply.setJson("[1]");
  send_reply(res, params, reply);
}

void managementDescription(Request &req, Response &res)
{
  char params[256];
  read_params(req, params, sizeof(params));
  char description[160];
  snprintf(description, sizeof(description),
           "{\"ServerName\":\"Alpaca weather station\",\"Manufacturer\":\"PPP\",\"ManufacturerVersion\":\"%s\",\"Location\":\"Observatory\"}",
           DRIVER_VERSION);
  AlpacaReply reply;
  reply.setJson(description);
  send_reply(res, params, reply);
}

void configuredDevices(Request &req, Response &res)
{
  char params[256];
  read_params(req, params, sizeof(params));
  char devices[320];
  snprintf(devices, sizeof(devices),
           "[{\"DeviceName\":\"%s\",\"DeviceType\":\"SafetyMonitor\",\"DeviceNumber\":0,\"UniqueID\":\"%s\"},"
           "{\"DeviceName\":\"%s\",\"DeviceType\":\"ObservingConditions\",\"DeviceNumber\":0,\"UniqueID\":\"%s\"}]",
           SAFETY_MONITOR_NAME, safetyMonitorUniqueID, CONDITIONS_NAME, conditionsUniqueID);
  AlpacaReply reply;
  reply.setJson(devices);
  send_reply(res, params, reply);
}

// Runs last, for any request no route has answered
void badRequest(Request &req, Response &res)
{
  res.status(400);
  res.set("Content-Type", "text/plain");
  res.print("Unknown endpoint");
  res.end();
}

// Answer Alpaca discovery broadcasts with the HTTP port
void handle_discovery()
{
  int size = discoveryUdp.parsePacket();
  if (size <= 0)
  {
    return;
  }

  char packet[32] = {0};
  discoveryUdp.read(packet, min(size, (int)sizeof(packet) - 1));

  if (strncmp(packet, "alpacadiscovery", 15) == 0)
  {
    char reply[24];
    snprintf(reply, sizeof(reply), "{\"AlpacaPort\":%u}", ALPACA_PORT);
    discoveryUdp.beginPacket(discoveryUdp.remoteIP(), discoveryUdp.remotePort());
    discoveryUdp.write((const uint8_t *)reply, strlen(reply));
    discoveryUdp.endPacket();
  }
}

const char *reset_reason_name(reset_reason_t reason)
{
  switch (reason)
  {
  case RESET_REASON_POWER_ON:
    return "power on";
  case RESET_REASON_PIN_RESET:
    return "reset pin";
  case RESET_REASON_BROWN_OUT:
    return "brown-out (low supply voltage)";
  case RESET_REASON_SOFTWARE:
    return "software";
  case RESET_REASON_WATCHDOG:
    return "watchdog";
  case RESET_REASON_LOCKUP:
    return "core lockup";
  case RESET_REASON_MULTIPLE:
    return "multiple causes";
  default:
    return "unknown";
  }
}

// Display the form for setting the safety limits.
// Printed in parts, so no large String is built on the heap.
void setupPage(Request &req, Response &res)
{
  res.set("Content-Type", "text/html; charset=utf-8");

  res.print(R"~(
  <!DOCTYPE html>
  <html lang="en">
  <head>
      <meta charset="UTF-8">
      <meta name="viewport" content="width=device-width, initial-scale=1.0">
      <title>Safety limits</title>
      <style>
          body {
              font-family: system-ui;
              margin: 0;
              padding: 0;
              display: flex;
              justify-content: center;
              align-items: center;
              min-height: 100vh;
              background-color: #121212;
              color: #fff;
          }
          .container {
              padding: 20px;
              border-radius: 8px;
              box-shadow: 0 0 10px rgba(0, 0, 0, 0.1);
              width: 300px;
              background-color: #1e1e1e;
          }
          h2 {
              margin-top: 0;
              font-size: large;
              font-weight: 400;
              color: #999999;
          }
          .status {
              font-size: small;
              color: #999999;
              margin-bottom: 25px;
          }
          .form-group {
              margin-bottom: 25px;
              padding: 10px 0px;
          }
          .title {
              display: flex;
              justify-content: space-between;
              margin-bottom: 7px;
          }
          .property {
              font-size: large;
          }
          .units {
              font-size: small;
              color: #999999;
          }
          .form-group label {
              display: block;
              margin-bottom: 5px;
              font-size: small;
              color: #999999;
          }
          .form-group input {
              width: 100%;
              padding: 8px;
              box-sizing: border-box;
          }
          .form-group input[type="submit"] {
              border: none;
              cursor: pointer;
              padding: 10px;
              border-radius: 4px;
              transition: background-color 0.3s, color 0.3s;
              background-color: #93d4ff;
              color: #000;
          }
          .form-group input[type="submit"]:hover {
              opacity: 0.9;
          }
          .side-by-side {
              display: flex;
              justify-content: space-between;
          }
          .side-by-side > div {
              flex: 1;
          }
          .side-by-side > div:first-child {
              margin-right: 10px;
          }
      </style>
  </head>
  <body>
      <div class="container">
          <h2>Safety limits 🧯</h2>
          <div class="status">)~");

  // status, to see why the board restarted and whether the sensors answer
  const char *error = sensorError;
  struct mallinfo heap = mallinfo();
  res.print(isSafe ? "Safe" : "Not safe");
  if (error != NULL)
  {
    res.print(": ");
    res.print(error);
  }
  res.print("<br>Uptime: ");
  res.print(millis() / 60000);
  res.print(" min<br>Last reset: ");
  res.print(reset_reason_name(resetReason));
  res.print("<br>Heap in use: ");
  res.print(heap.uordblks);
  res.print(" bytes");
  if (storeError != NULL)
  {
    res.print("<br>");
    res.print(storeError);
  }

  res.print(R"~(</div>
          <form action="/submit" method="post" onsubmit="return validateForm()">

              <!-- temperature limits -->
              <div class="form-group">
                  <div class="title">
                      <div class="property">Temperature</div>
                      <div class="units">°C</div>
                  </div>
                  <div class="side-by-side">
                      <div title="Current: )~");
  res.print(currentLimits.AT_min);
  res.print(R"~( °C">
                          <label for="min-temp">Min</label>
                          <input type="number" id="min-temp" name="min-temp" value=)~");
  res.print(currentLimits.AT_min);
  res.print(R"~( required>
                      </div>
                      <div title="Current: )~");
  res.print(currentLimits.AT_max);
  res.print(R"~( °C">
                          <label for="max-temp">Max</label>
                          <input type="number" id="max-temp" name="max-temp" value=)~");
  res.print(currentLimits.AT_max);
  res.print(R"~( required>
                      </div>
                  </div>
              </div>

              <!-- humidity limits -->
              <div class="form-group">
                  <div class="title">
                      <div class="property">Humidity</div>
                      <div class="units">%</div>
                  </div>
                  <div class="side-by-side">
                      <div title="Current: )~");
  res.print(currentLimits.AH_min);
  res.print(R"~( %">
                          <label for="min-rh">Min</label>
                          <input type="number" id="min-rh" name="min-rh" value=)~");
  res.print(currentLimits.AH_min);
  res.print(R"~( min=0 required>
                      </div>
                      <div title="Current: )~");
  res.print(currentLimits.AH_max);
  res.print(R"~( %">
                          <label for="max-rh">Max</label>
                          <input type="number" id="max-rh" name="max-rh" value=)~");
  res.print(currentLimits.AH_max);
  res.print(R"~( max=100 required>
                      </div>
                  </div>
              </div>

              <!-- wind limits -->
              <div class="form-group">
                  <div class="title">
                      <div class="property">Wind</div>
                      <div class="units">m/s</div>
                  </div>
                  <div class="">
                      <div title="Current: )~");
  res.print(currentLimits.SA_max);
  res.print(R"~( m/s">
                          <label for="max-wind">Max</label>
                          <input type="number" id="max-wind" name="max-wind" value=)~");
  res.print(currentLimits.SA_max);
  res.print(R"~( min=0 required>
                      </div>
                  </div>
              </div>

              <!-- gust wind limits -->
              <div class="form-group">
                  <div class="title">
                      <div class="property">Gust wind</div>
                      <div class="units">m/s</div>
                  </div>
                  <div class="">
                      <div title="Current: )~");
  res.print(currentLimits.SM_max);
  res.print(R"~( m/s">
                          <label for="max-gust-wind">Max</label>
                          <input type="number" id="max-gust-wind" name="max-gust-wind" value=)~");
  res.print(currentLimits.SM_max);
  res.print(R"~( min=0 required>
                      </div>
                  </div>
              </div>

              <!-- light limits -->
              <div class="form-group">
                  <div class="title">
                      <div class="property">Light</div>
                      <div class="units">lux</div>
                  </div>
                  <div class="">
                      <div title="Current: )~");
  res.print(currentLimits.LX_max);
  res.print(R"~( lux">
                          <label for="max-light">Max</label>
                          <input type="number" id="max-light" name="max-light" value=)~");
  res.print(currentLimits.LX_max);
  res.print(R"~( min=0 required>
                      </div>
                  </div>
              </div>

              <!-- sky temperature -->
              <div class="form-group">
                  <div class="title">
                      <div class="property">Sky temperature</div>
                      <div class="units">°C</div>
                  </div>
                  <div class="side-by-side">
                      <div title="Current: )~");
  res.print(currentLimits.ST_min);
  res.print(R"~( °C">
                          <label for="min-sky-temp">Min</label>
                          <input type="number" id="min-sky-temp" name="min-sky-temp" value=)~");
  res.print(currentLimits.ST_min);
  res.print(R"~( required>
                      </div>
                      <div title="Current: )~");
  res.print(currentLimits.ST_max);
  res.print(R"~( °C">
                          <label for="max-sky-temp">Max</label>
                          <input type="number" id="max-sky-temp" name="max-sky-temp" value=)~");
  res.print(currentLimits.ST_max);
  res.print(R"~( required>
                      </div>
                  </div>
              </div>

              <!-- relative sky temperature -->
              <div class="form-group">
                  <div class="title">
                      <div class="property">Relative sky temperature</div>
                      <div class="units">°C</div>
                  </div>
                  <div class="">
                      <div title="Current: )~");
  res.print(currentLimits.RST_max);
  res.print(R"~( °C">
                          <label for="max-rel-sky-temp">Max</label>
                          <input type="number" id="max-rel-sky-temp" name="max-rel-sky-temp" value=)~");
  res.print(currentLimits.RST_max);
  res.print(R"~( required>
                      </div>
                  </div>
              </div>

              <!-- safety false duration -->
              <div class="form-group">
                  <div class="title">
                      <div class="property">Safety false duration</div>
                      <div class="units">s</div>
                  </div>
                  <div class="">
                      <div title="Current: )~");
  res.print(currentLimits.safety_false_duration);
  res.print(R"~( s">
                          <label for="safety-false-duration">Min</label>
                          <input type="number" id="safety-false-duration" name="safety-false-duration" value=)~");
  res.print(currentLimits.safety_false_duration);
  res.print(R"~( min=0 required>
                      </div>
                  </div>
              </div>

              <div class="form-group">
                  <input type="submit" value="Submit">
              </div>
          </form>
      </div>
  </body>
  <script>
      // on load, reset the form
      window.onload = function() {
          document.querySelector('form').reset();
      }

      function validateForm() {
          const minTemp = document.getElementById('min-temp').value;
          const maxTemp = document.getElementById('max-temp').value;
          if (parseInt(minTemp) > parseInt(maxTemp)) {
              alert('Min temperature cannot be greater than Max temperature.');
              return false;
          }

          const minRH = document.getElementById('min-rh').value;
          const maxRH = document.getElementById('max-rh').value;
          if (parseInt(minRH) > parseInt(maxRH)) {
              alert('Min humidity cannot be greater than Max humidity.');
              return false;
          }

          const minSkyTemp = document.getElementById('min-sky-temp').value;
          const maxSkyTemp = document.getElementById('max-sky-temp').value;
          if (parseInt(minSkyTemp) > parseInt(maxSkyTemp)) {
              alert('Min sky temperature cannot be greater than Max sky temperature.');
              return false;
          }

          return true;
      }
  </script>
  </html>
  )~");

  res.end();
}

// Submit the safety limits form data
void submit(Request &req, Response &res)
{

  char name[32];
  char value[32];

  while (req.left())
  {
    if (!req.form(name, 32, value, 32))
    {
      res.sendStatus(400);
      res.end();
      return;
    }

    if (strcmp(name, "max-temp") == 0)
    {
      currentLimits.AT_max = atof(value);
    }
    else if (strcmp(name, "min-temp") == 0)
    {
      currentLimits.AT_min = atof(value);
    }
    else if (strcmp(name, "max-rh") == 0)
    {
      currentLimits.AH_max = atof(value);
    }
    else if (strcmp(name, "min-rh") == 0)
    {
      currentLimits.AH_min = atof(value);
    }
    else if (strcmp(name, "max-light") == 0)
    {
      currentLimits.LX_max = atof(value);
    }
    else if (strcmp(name, "max-wind") == 0)
    {
      currentLimits.SA_max = atof(value);
    }
    else if (strcmp(name, "max-gust-wind") == 0)
    {
      currentLimits.SM_max = atof(value);
    }
    else if (strcmp(name, "min-sky-temp") == 0)
    {
      currentLimits.ST_min = atof(value);
    }
    else if (strcmp(name, "max-sky-temp") == 0)
    {
      currentLimits.ST_max = atof(value);
    }
    else if (strcmp(name, "max-rel-sky-temp") == 0)
    {
      currentLimits.RST_max = atof(value);
    }
    else if (strcmp(name, "safety-false-duration") == 0)
    {
      currentLimits.safety_false_duration = atoi(value);
    }
  }

  // Update the stats and save them to the store. The new limits are used
  // even if saving fails, they are then lost at the next restart.
  bool saved = storeError == NULL && setLimits(limitsKey, currentLimits) == MBED_SUCCESS;

  res.set("Content-Type", "text/html; charset=utf-8");

  // updated settings
  res.print(R"~(
  <!DOCTYPE html>
  <html lang="en">
  <head>
      <meta charset="UTF-8">
      <meta name="viewport" content="width=device-width, initial-scale=1.0">
      <title>Submitted safety limits</title>
  </head>
  <body>
  )~");

  res.print("<h1>Updated safety limits</h1>");
  if (!saved)
  {
    res.print("<p>Error: the limits are active, but they were not saved to flash. They are lost at the next restart.</p>");
  }
  res.print("<br>Temperature: ");
  res.print(currentLimits.AT_min);
  res.print(" - ");
  res.print(currentLimits.AT_max);
  res.print(" °C");

  res.print("<br>Humidity: ");
  res.print(currentLimits.AH_min);
  res.print(" - ");
  res.print(currentLimits.AH_max);
  res.print(" %RH");

  res.print("<br>Light: ");
  res.print(currentLimits.LX_max);
  res.print(" lux");

  res.print("<br>Wind: ");
  res.print(currentLimits.SA_max);
  res.print(" m/s");

  res.print("<br>Gust wind: ");
  res.print(currentLimits.SM_max);
  res.print(" m/s");

  res.print("<br>Sky temperature: ");
  res.print(currentLimits.ST_min);
  res.print(" - ");
  res.print(currentLimits.ST_max);
  res.print(" °C");

  res.print("<br>Relative sky temperature: ");
  res.print(currentLimits.RST_max);
  res.print(" °C");

  res.print("<br>Safety false duration: ");
  res.print(currentLimits.safety_false_duration);
  res.print(" s");

  res.print(R"~(
  </body>
  </html>
  )~");

  res.end();
}

// Retrieve WeatherLimits from the key-value store
int getLimits(const char *key, WeatherLimits *limits)
{
  // Read into a copy, so a failed or old-format read leaves the defaults
  WeatherLimits stored;
  size_t actual_size = 0;

  auto result = store.get(key, &stored, sizeof(stored), &actual_size);
  if (result != MBED_SUCCESS)
    return result;

  // saved by a version with a different WeatherLimits struct
  if (actual_size != sizeof(WeatherLimits))
    return MBED_ERROR_INVALID_SIZE;

  *limits = stored;
  return result;
}

// Store a WeatherLimits to the the key-value store
int setLimits(const char *key, WeatherLimits stats)
{
  return store.set(key, reinterpret_cast<uint8_t *>(&stats), sizeof(WeatherLimits), 0);
}

// setup
void setup()
{
  // set the safety relay off (not safe) before anything else
  pinMode(SAFETY_RELAY, OUTPUT);
  pinMode(SAFETY_RELAY_LED, OUTPUT);
  digitalWrite(SAFETY_RELAY, LOW);
  digitalWrite(SAFETY_RELAY_LED, LOW);
  // set pins for LED testing
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);

  // wait for terminal to come up
  delay(1000);

  // start the watchdog
  mbed::Watchdog::get_instance().start(WATCHDOG_TIMEOUT);

  resetReason = mbed::ResetReason::get();
  Serial.print("Last reset: ");
  Serial.println(reset_reason_name(resetReason));

  info = boardInfo();
  if (info->magic == 0xB5)
  {
    for (int i = 0; i < 6; i++)
    {
      mac[i] = info->mac_address[i];
    }
  }

  snprintf(safetyMonitorUniqueID, sizeof(safetyMonitorUniqueID), "%02x%02x%02x%02x-%02x%02x-4000-8000-000000000001",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  snprintf(conditionsUniqueID, sizeof(conditionsUniqueID), "%02x%02x%02x%02x-%02x%02x-4000-8000-000000000002",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  // the default Ethernet.begin waits 60 s for a link, longer than the watchdog
  IPAddress localIP(ip);
  IPAddress gateway(ip[0], ip[1], ip[2], 1);
  IPAddress subnet(255, 255, 255, 0);
  if (Ethernet.begin(mac, localIP, gateway, gateway, subnet, ETHERNET_TIMEOUT))
  {
    Serial.println(Ethernet.localIP());
  }
  else
  {
    // continue, the safety relay must work without the network
    Serial.println("Ethernet has no link, continuing without network");
  }
  mbed::Watchdog::get_instance().kick();

  // Initialize the flash IAP block device and print the memory layout
  blockDevice.init();
  Serial.print("FlashIAP block device size: ");
  Serial.println(blockDevice.size());
  Serial.print("FlashIAP block device read size: ");
  Serial.println(blockDevice.get_read_size());
  Serial.print("FlashIAP block device program size: ");
  Serial.println(blockDevice.get_program_size());
  Serial.print("FlashIAP block device erase size: ");
  Serial.println(blockDevice.get_erase_size());
  // Deinitialize the device
  blockDevice.deinit();

  // Initialize the key-value store. If it fails, continue with the default
  // limits, so the safety relay still works.
  Serial.print("Initializing TDBStore: ");
  auto result = store.init();
  Serial.println(result == MBED_SUCCESS ? "OK" : "Failed");
  if (result != MBED_SUCCESS)
  {
    storeError = "Settings store failed to start, default limits are used";
  }
  else
  {
    // Get previous run stats from the key-value store
    Serial.println("Retrieving Limits");
    result = getLimits(limitsKey, &currentLimits);

    if (result == MBED_SUCCESS)
    {
      Serial.println("Previous limits");
      Serial.print("AT_min: ");
      Serial.println(currentLimits.AT_min);
      Serial.print("AT_max: ");
      Serial.println(currentLimits.AT_max);
      Serial.print("AH_min: ");
      Serial.println(currentLimits.AH_min);
      Serial.print("AH_max: ");
      Serial.println(currentLimits.AH_max);
      Serial.print("LX_max: ");
      Serial.println(currentLimits.LX_max);
      Serial.print("SM_max: ");
      Serial.println(currentLimits.SM_max);
      Serial.print("SA_max: ");
      Serial.println(currentLimits.SA_max);
      Serial.print("Safety false time: ");
      Serial.println(currentLimits.safety_false_duration);
    }
    else if (result == MBED_ERROR_ITEM_NOT_FOUND)
    {
      Serial.println("No previous data was found.");
    }
    else
    {
      Serial.println("Error reading from key-value store, default limits are used.");
    }
  }
  mbed::Watchdog::get_instance().kick();

  // for Kemo M152K rain sensor
  pinMode(rainSensorSafePin, INPUT);
  pinMode(rainSensorSignalHighPin, OUTPUT);
  digitalWrite(rainSensorSignalHighPin, HIGH);
  // attachInterrupt(digitalPinToInterrupt(rainSensorSafePin), rainSensorSafeISR, CHANGE);

  // initialize the SkyTemperature sensors. If one does not answer, continue:
  // the sensor thread then reports invalid readings and keeps the relay unsafe.
  for (int attempt = 0; attempt < 3 && mlx1.begin() != NO_ERR; attempt++)
  {
    Serial.println("Communication with MLX90614 0x5A failed, please check connection");
    delay(1000);
    mbed::Watchdog::get_instance().kick();
  }

  for (int attempt = 0; attempt < 3 && mlx2.begin() != NO_ERR; attempt++)
  {
    Serial.println("Communication with MLX90614 0x5B failed, please check connection");
    delay(1000);
    mbed::Watchdog::get_instance().kick();
  }

  /**
   * adjust SkyTemperature MLX Sensor sleep mode
   * mode select to enter or exit sleep mode, it's enter sleep mode by default
   *      true is to enter sleep mode
   *      false is to exit sleep mode (automatically exit sleep mode after power down and restart)
   */
  mlx1.enterSleepMode();
  delay(50);
  mlx1.enterSleepMode(false);
  delay(200);

  mlx2.enterSleepMode();
  delay(50);
  mlx2.enterSleepMode(false);
  delay(200);
  mbed::Watchdog::get_instance().kick();

  // setup pages, Alpaca clients open /setup/v1/{devicetype}/0/setup
  app.get("/", &setupPage);
  app.get("/setup", &setupPage);
  app.get("/setup/v1/safetymonitor/0/setup", &setupPage);
  app.get("/setup/v1/observingconditions/0/setup", &setupPage);
  app.post("/submit", &submit);

  // device endpoints
  app.get("/api/v1/safetymonitor/0/:member", &safetyMonitorEndPoint);
  app.put("/api/v1/safetymonitor/0/:member", &safetyMonitorEndPoint);
  app.get("/api/v1/observingconditions/0/:member", &conditionsEndPoint);
  app.put("/api/v1/observingconditions/0/:member", &conditionsEndPoint);

  app.get("/management/apiversions", &apiVersions);
  app.get("/management/v1/description", &managementDescription);
  app.get("/management/v1/configureddevices", &configuredDevices);

  // must be registered last
  app.use(&badRequest);

  server.begin();
  discoveryUdp.begin(DISCOVERY_PORT);

  // RS485 init
  RS485.begin(baudrate);
  RS485.setDelays(preDelayBR, postDelayBR);

  // start SensorReadThread thread
  sensorHeartbeat = millis();
  SensorReadThread.start(readSensors);
}

// loop
void loop()
{
  handle_discovery();

  EthernetClient client = server.accept();

  if (client.connected())
  {
    app.process(&client);
    client.stop();
  }

  // only kick the watchdog while the sensor thread is alive, so a stuck
  // sensor thread or a stuck web server both reset the board
  if (millis() - sensorHeartbeat < SENSOR_HEARTBEAT_TIMEOUT)
  {
    mbed::Watchdog::get_instance().kick();
  }
}

// opta manual
// https://docs.arduino.cc/tutorials/opta/user-manual/

// ascii commands
// https://files.seeedstudio.com/products/SenseCAP/SenseCAP_ONE/SenseCAP_ONE_V2_Compact_Weather_Station_User_Guide.pdf

// alpaca commands
// https://ascom-standards.org/api/
// https://ascom-standards.org/newdocs/observingconditions.html
// https://ascom-standards.org/newdocs/safetymonitor.html
// https://ascom-standards.org/alpyca/alpaca.observingconditions.html
// https://ascom-standards.org/alpyca/alpaca.safetymonitor.html

// to use both cores?
// https://docs.arduino.cc/tutorials/giga-r1-wifi/giga-dual-core/

// threads
// https://opta.findernet.com/en/tutorial/multithreading-opta-and-serie-7m

// or somehow fix lag & echo...
// https://www.arduino.cc/reference/en/libraries/arduinors485/

// context info
// https://github.com/lasselukkari/aWOT/blob/master/examples/RequestContext/RequestContext.ino

// other sensors to add
// https://lambermont.dyndns.org/astro/weatherstation/
// https://www.davisinstruments.com/collections/add-on-sensors/products/anemometer-for-vantage-pro2-vantage-pro
// Kemo M152K rain sensor

// Creating a Flash-Optimized Key-Value Store
// https://docs.arduino.cc/tutorials/portenta-h7/flash-optimized-key-value-store/

// arduino-cli compile --fqbn arduino:mbed_opta:opta ./
// arduino-cli upload -p /dev/cu.usbmodem1301 --fqbn arduino:mbed_opta:opta ./


// arduino-cli compile --fqbn arduino-git:mbed:opta ./
// arduino-cli upload -p /dev/cu.usbmodem2101 --fqbn arduino-git:mbed:opta ./


// TODO:
// - would be nice if we could set the IP from the web interface
