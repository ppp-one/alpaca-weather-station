#include <ArduinoRS485.h>
#include <Ethernet.h>
#include <aWOT.h>
#include <cmath>
#include <unordered_map>
#include <string>
#include <FlashIAPBlockDevice.h>
#include "FlashIAPLimits.h"
#include <TDBStore.h>
#include "opta_info.h"

OptaBoardInfo* info;
OptaBoardInfo* boardInfo();

using namespace mbed;

// For storing the safety limits
// Get limits of the In Application Program (IAP) flash, ie. the internal MCU flash.
auto iapLimits { getFlashIAPLimits() };

// Create a block device on the available space of the FlashIAP
FlashIAPBlockDevice blockDevice(iapLimits.start_address, iapLimits.available_size);

// Create a key-value store on the Flash IAP block device
TDBStore store(&blockDevice);

// struct for weather limits
struct WeatherLimits {
  float AT_min = 0; // Air temperature low limit C
  float AT_max = 50; // Air temperature high limit C
  float AH_min = 0; // Air humidity low limit %RH
  float AH_max = 80; // Air humidity high limit %RH
  // float AP_min; // Barometric pressure low limit Pa
  // float AP_max; // Barometric pressure high limit Pa
  float LX_max = 10000; // Light intensity high limit Lux
  float SM_max = 20; // Gust wind speed high limit m/s (default), km/h, mph, knots
  float SA_max = 15; // Average wind speed high limit m/s (default), km/h, mph, knots
  // float RA_max; // Accumulated rainfall high limit mm (default), in
  // float RD_max; // Duration of rainfall high limit s
  // float RI_max; // Rainfall intensity high limit mm/h (default), in/h
  // float Rp_max; // Maximum rainfall intensity high limit mm/h (default), in/h
  int safety_false_duration = 10; // Time in seconds to keep isSafe false if not safe
};

// Current weather limits, to be updated during runtime or submitted via the web interface
WeatherLimits currentLimits;

// An example key name for the stats on the store
const char limitsKey[] { "limits" };

// RS485
constexpr auto baudrate{ 9600 };
// Calculate preDelay and postDelay in microseconds for stable RS-485 transmission
constexpr auto bitduration{ 1.f / baudrate };
constexpr auto wordlen{ 9.6f };  // OR 10.0f depending on the channel configuration
constexpr auto preDelayBR{ bitduration * wordlen * 3.5f * 1e6 };
constexpr auto postDelayBR{ bitduration * wordlen * 3.5f * 1e6 };

// Thread for reading sensors
static rtos::Thread SensorReadThread;

// S700 values
float s700Values[16];

// S700 key
std::unordered_map<std::string, int> s700Key = {
    {"AT", 0}, // AT Air temperature C
    {"AH", 1}, // AH Air humidity %RH
    {"AP", 2}, // AP Barometric pressure Pa
    {"LX", 3}, // LX Light intensity Lux
    {"DN", 4}, // DN Minimum wind direction deg
    {"Dm", 5}, // Dm Maximum wind direction deg
    {"DA", 6}, // DA Average wind direction deg
    {"SN", 7}, // SN Minimum wind speed m/s (default), km/h, mph, knots
    {"SM", 8}, // SM Maximum wind speed m/s (default), km/h, mph, knots
    {"SA", 9}, // SA Average wind speed m/s (default), km/h, mph, knots
    {"RA", 10}, // RA Accumulated rainfall mm (default), in
    {"RD", 11}, // RD Duration of rainfall s
    {"RI", 12}, // RI Rainfall intensity mm/h (default), in/h
    {"Rp", 13}, // Rp Maximum rainfall intensity mm/h (default), in/h
    {"HT", 14}, // HT Heating temperature C
    {"TILT", 15} // TILT Fall detection
};

// isSafe
bool isSafe = false;

// Rain sensor
volatile bool rainSensorSafe = true;
constexpr int rainSensorSafePin = A2;

// Alpaca error handling
int32_t ErrorNumber = 0;
String ErrorMessage = "\"\"";

// the media access control (ethernet hardware) address
byte mac[6];
//the IP address
byte ip[] = { 192, 168, 1, 12 };

// Create an Ethernet server
EthernetServer server(80);
Application app;

// ISR for fully closed sensor
void rainSensorSafeISR() {
  rainSensorSafe = digitalRead(rainSensorSafePin) == LOW;
  Serial.println("Rain detection change");
}

String split(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

// Read all sensors and sets safety flag - runs in a separate thread
void readSensors()
{
  auto time_since_not_safe = rtos::Kernel::Clock::now();
  bool weather_station_data_received = false;
  bool safety_check_received = false;

  while(true) {
    // TODO: wrap in try catch

    digitalWrite(LED_BUILTIN, HIGH);

    // read weather station data
    weather_station_data_received = readWeatherStation();

    // safety check of weather station data
    safety_check_received = safetyCheck();

    // set isSafe
    isSafe = weather_station_data_received && safety_check_received && rainSensorSafe;

    // if not safe, keep isSafe false for 10 s, but continue to read sensors
    if (!isSafe) {
      time_since_not_safe = rtos::Kernel::Clock::now();
    }
    if ((rtos::Kernel::Clock::now() - time_since_not_safe > std::chrono::seconds(currentLimits.safety_false_duration)) && isSafe) {
      isSafe = true;
    } else {
      isSafe = false;
    }

    // testing LED
    if (!isSafe) {
      digitalWrite(LED_D0, HIGH);
    } else {
      digitalWrite(LED_D0, LOW);
    }

    // if no data received
    if (!weather_station_data_received) {
      ErrorNumber = 1;
      ErrorMessage = "\"No data received from weather station\"";
    } else {
      ErrorNumber = 0;
      ErrorMessage = "\"\"";

      rtos::ThisThread::sleep_for(500);
      digitalWrite(LED_BUILTIN, LOW);
      rtos::ThisThread::sleep_for(500);
    }

  }
}

// Read data from the S700 weather station
bool readWeatherStation() {
  const unsigned long timeout = 500; // 500 ms timeout
  const int maxChars = 256; // Maximum characters to read
  char buffer[maxChars + 1]; // +1 for null terminator
  int charCount = 0;
  auto startTime = rtos::Kernel::Clock::now();

  RS485.noReceive();
  RS485.beginTransmission();
  RS485.println("0XA;G0?");
  RS485.flush();
  RS485.endTransmission();
  RS485.receive();

  while (rtos::Kernel::Clock::now() - startTime < std::chrono::milliseconds(timeout)) {
    if (RS485.available()) {
      char c = RS485.read();
      buffer[charCount++] = c;
      if (c == '\n') {
        break;
      }
    }
  }

  buffer[charCount] = '\0'; // Null-terminate the string

  String val = String(buffer);

  if (charCount == 0) {
    // Serial.println("No data received within timeout");
    return false;
  }

  // If the response is valid, parse the values
  if (val.startsWith("0XA;AT") && val.endsWith("TILT=0\r\n")) {
    int valint = 0;
    int found = 0;
    int strIndex[] = {0, -1};
    int maxIndex = val.length()-1;
    char separator = ';';

    for(int i=0; i<=maxIndex; i++){
      if(val.charAt(i)==separator || i==maxIndex){
          found++;
          strIndex[0] = strIndex[1]+1;
          strIndex[1] = (i == maxIndex) ? i+1 : i;
      }

      if (found == 2) {
        auto vi = val.substring(strIndex[0], strIndex[1]);
        found=1;

        if (vi.length() > 2 && valint < 15) {
          // Serial.print("valint: ");
          // Serial.print(valint);
          // Serial.print(" ");
          // Serial.print(vi);
          // Serial.print(" ");
          // Serial.println(vi.substring(3).toFloat());
          

          s700Values[valint] = vi.substring(3).toFloat();
          valint++;
        }
      }
    }
    return true;
  } else {
    Serial.print("Invalid response received: ");
    Serial.println(val);
    return false;
  }
}

// Safety check of weather station data and connected sensors
bool safetyCheck() {
  // check if any values are outside of the safety limits
  // temperature
  if (s700Values[s700Key["AT"]] < currentLimits.AT_min || s700Values[s700Key["AT"]] > currentLimits.AT_max) {
    Serial.println("Temperature out of range");
    return false;
  }
  // humidity
  if (s700Values[s700Key["AH"]] < currentLimits.AH_min || s700Values[s700Key["AH"]] > currentLimits.AH_max) {
    Serial.println("Humidity out of range");
    return false;
  }
  // pressure
  // if (s700Values[s700Key["AP"]] < currentLimits.AP_min || s700Values[s700Key["AP"]] > currentLimits.AP_max) {
  //   Serial.println("Pressure out of range");
  //   return false;
  // }

  // light intensity
  if (s700Values[s700Key["LX"]] > currentLimits.LX_max) {
    Serial.println("Light intensity out of range");
    return false;
  }

  // gust wind speed
  if (s700Values[s700Key["SM"]] > currentLimits.SM_max) {
    Serial.println("Wind speed out of range");
    return false;
  }

  // average wind speed
  if (s700Values[s700Key["SA"]] > currentLimits.SA_max) {
    Serial.println("Wind speed out of range");
    return false;
  }

  // these require user resetting (see manual)
  // // accumulated rainfall
  // if (s700Values[s700Key["RA"]] > currentLimits.RA_max) {
  //   Serial.println("Rainfall out of range");
  //   return false;
  // }

  // // duration of rainfall
  // if (s700Values[s700Key["RD"]] > currentLimits.RD_max) {
  //   Serial.println("Rainfall duration out of range");
  //   return false;
  // }

  // rainfall intensity
  if (s700Values[s700Key["RI"]] > 0) {
    Serial.println("Rainfall intensity out of range");
    return false;
  }

  // maximum rainfall intensity
  if (s700Values[s700Key["Rp"]] > 0) {
    Serial.println("Rainfall intensity out of range");
    return false;
  }

  return true;
}

// Handle the endpoint requests
void endPoint(Request &req, Response &res) {

  res.set("Content-Type", "application/json");
  auto url = split(req.path(), '/', 5);

  uint32_t ClientID = 0;
  uint32_t ClientTransactionID = 0;

  uint32_t ServerID = 0; // TODO: what is this?
  uint32_t ServerTransactionID = 0; // TODO: what is this? increment?

  float Value;
  String ValueString;
  bool ValueBool;

  char buffer[16]; // Adjust the buffer size as needed

  if (req.query("ClientID", buffer, sizeof(buffer))) {
    ClientID = strtoul(buffer, NULL, 10);
  }

  if (req.query("ClientTransactionID", buffer, sizeof(buffer))) {
    ClientTransactionID = strtoul(buffer, NULL, 10);
  }

  if (url == "connected") {
    ValueBool = true;
  } else if (url == "name") {
    ValueString = "Weather Station and safety monitor by PPP"; // TODO: change this, add limits?
  } else if (url == "driverversion") {
    ValueString = "0.0.2";
  } else if (url == "issafe") {
    ValueBool = isSafe;
  } else if (url == "temperature") {
    Value = s700Values[s700Key["AT"]];
  } else if (url == "humidity") {
    Value = s700Values[s700Key["AH"]];
  } else if (url == "dewpoint") {
    // TODO: check this okay
    // source?
    float B = (log(s700Values[1] / 100) + ((17.27 * s700Values[0]) / (237.3 + s700Values[0]))) / 17.27;
    float dewpoint = (237.3 * B) / (1 - B);
    Value = dewpoint;
  } else if (url == "pressure") {
    Value = s700Values[s700Key["AP"]]/100;
  } else if (url == "skybrightness") {
    Value = s700Values[s700Key["LX"]];
  } else if (url == "windspeed") {
    Value = s700Values[s700Key["SA"]];
  } else if (url == "windgust") {
    Value = s700Values[s700Key["SM"]];
  } else if (url == "winddirection") {
    Value = s700Values[s700Key["DA"]];
  } else if (url == "rainrate") {
    Value = s700Values[s700Key["RI"]];
    if (!rainSensorSafe) {
      Value += 0.01; // add 0.01 mm/h if rain sensor is active since rainrate on the S700 is not that sensitive
    }
  } else {
    ErrorNumber = 1;
    ErrorMessage = "\"Invalid path\"";
  }

  res.print("{");
  res.print("\"ClientID\": ");
  res.print(ClientID);
  res.print(", ");
  res.print("\"ClientTransactionID\": ");
  res.print(ClientTransactionID);
  res.print(", ");
  res.print("\"ServerID\": ");
  res.print(ServerID);
  res.print(", ");
  res.print("\"ErrorNumber\": ");
  res.print(ErrorNumber);
  res.print(", ");
  res.print("\"ErrorMessage\": ");
  res.print(ErrorMessage);
  res.print(", ");
  res.print("\"Value\": ");
  if (url == "name" || url == "driverversion") {
    res.print("\"");
    res.print(ValueString);
    res.print("\"");
  } else if (url == "issafe" || url == "connected") {
    res.print(ValueBool ? "true" : "false");
  } else {
    res.print(Value);
  }
  res.print("}");

  // reset error
  ErrorNumber = 0;
  ErrorMessage = "\"\"";

}

// Display the form for setting the safety limits
void index(Request &req, Response &res) {

  res.set("Content-Type", "text/html; charset=utf-8");

  String index_html = R"~(
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
              height: 100vh;
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
          <form action="/submit" method="post" onsubmit="return validateForm()">

              <!-- temperature limits -->
              <div class="form-group">
                  <div class="title">
                      <div class="property">Temperature</div>
                      <div class="units">°C</div>
                  </div>
                  <div class="side-by-side">
                      <div title="Current: )~" + String(currentLimits.AT_min) + R"~( °C">
                          <label for="min-temp">Min</label>
                          <input type="number" id="min-temp" name="min-temp" value=)~" + String(currentLimits.AT_min) + R"~( required>
                      </div>
                      <div title="Current: )~" + String(currentLimits.AT_max) + R"~( °C">
                          <label for="max-temp">Max</label>
                          <input type="number" id="max-temp" name="max-temp" value=)~" + String(currentLimits.AT_max) + R"~( required>
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
                      <div title="Current: )~" + String(currentLimits.AH_min) + R"~( %">
                          <label for="min-rh">Min</label>
                          <input type="number" id="min-rh" name="min-rh" value=)~" + String(currentLimits.AH_min) + R"~( min=0 required>
                      </div>
                      <div title="Current: )~" + String(currentLimits.AH_max) + R"~( %">
                          <label for="max-rh">Max</label>
                          <input type="number" id="max-rh" name="max-rh" value=)~" + String(currentLimits.AH_max) + R"~( max=100 required>
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
                      <div title="Current: )~" + String(currentLimits.SA_max) + R"~( m/s">
                          <label for="max-wind">Max</label>
                          <input type="number" id="max-wind" name="max-wind" value=)~" + String(currentLimits.SA_max) + R"~( min=0 required>
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
                      <div title="Current: )~" + String(currentLimits.SM_max) + R"~( m/s">
                          <label for="max-gust-wind">Max</label>
                          <input type="number" id="max-gust-wind" name="max-gust-wind" value=)~" + String(currentLimits.SM_max) + R"~( min=0 required>
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
                      <div title="Current: )~" + String(currentLimits.LX_max) + R"~( lux">
                          <label for="max-light">Max</label>
                          <input type="number" id="max-light" name="max-light" value=)~" + String(currentLimits.LX_max) + R"~( min=0 required>
                      </div>
                  </div>
              </div>

              <!-- sky temperature -->
              <div class="form-group">
                  <div class="title">
                      <div class="property">Sky temperature (NOT USED)</div>
                      <div class="units">°C</div>
                  </div>
                  <div class="">
                      <div title="Current: null °C">
                          <label for="max-sky-temp">Max</label>
                          <input type="number" id="max-sky-temp" name="max-sky-temp" value=-30 required>
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
                      <div title="Current: )~" + String(currentLimits.safety_false_duration) + R"~( s">
                          <label for="safety-false-duration">Min</label>
                          <input type="number" id="safety-false-duration" name="safety-false-duration" value=)~" + String(currentLimits.safety_false_duration) + R"~( min=0 required>
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

          return true;
      }
  </script>
  </html>
  )~";

  res.print(index_html);
}

// Submit the safety limits form data
void submit(Request &req, Response &res) {

  char name[32];
  char value[32];

  while (req.left()) {
    if (!req.form(name, 32, value, 32)) {
      return res.sendStatus(400);
    }

    if (strcmp(name, "max-temp") == 0) {
      currentLimits.AT_max = atof(value);
    } else if (strcmp(name, "min-temp") == 0) {
      currentLimits.AT_min = atof(value);
    } else if (strcmp(name, "max-rh") == 0) {
      currentLimits.AH_max = atof(value);
    } else if (strcmp(name, "min-rh") == 0) {
      currentLimits.AH_min = atof(value);
    } else if (strcmp(name, "max-light") == 0) {
      currentLimits.LX_max = atof(value);
    } else if (strcmp(name, "max-wind") == 0) {
      currentLimits.SA_max = atof(value);
    } else if (strcmp(name, "max-gust-wind") == 0) {
      currentLimits.SM_max = atof(value);
    } else if (strcmp(name, "safety-false-duration") == 0) {
      currentLimits.safety_false_duration = atoi(value);
    }
  }

  // Update the stats and save them to the store
  auto result = setLimits(limitsKey, currentLimits);
  
  if (result != MBED_SUCCESS) {
    Serial.println("Error while saving to key-value store");
    while (true);
  }
  
  String header = R"~(
  <!DOCTYPE html>
  <html lang="en">
  <head>
      <meta charset="UTF-8">
      <meta name="viewport" content="width=device-width, initial-scale=1.0">
      <title>Submitted safety limits</title>
  </head>
  <body>
  )~";

  String footer = R"~(
  </body>
  </html>
  )~";

  res.set("Content-Type", "text/html; charset=utf-8");

  // updated settings
  res.print(header);

  res.print("<h1>Updated safety limits</h1>");
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

  res.print("<br>Safety false duration: ");
  res.print(currentLimits.safety_false_duration);
  res.print(" s");

  res.print(footer);

}

// Retrieve WeatherLimits from the key-value store
int getLimits(const char* key, WeatherLimits* limits)
{
  // Retrieve key-value info
  TDBStore::info_t info;
  auto result = store.get_info(key, &info);

  if (result == MBED_ERROR_ITEM_NOT_FOUND)
    return result;

  // Allocate space for the value
  uint8_t buffer[info.size] {};
  size_t actual_size;

  // Get the value
  result = store.get(key, buffer, sizeof(buffer), &actual_size);
  if (result != MBED_SUCCESS)
    return result;

  memcpy(limits, buffer, sizeof(WeatherLimits));
  return result;
}

// Store a WeatherLimits to the the key-value store
int setLimits(const char* key, WeatherLimits stats)
{
  return store.set(key, reinterpret_cast<uint8_t*>(&stats), sizeof(WeatherLimits), 0);  
}

// setup
void setup() {
  
  Serial.begin(115200);

  //  Wait for terminal to come up
  delay(1000);

  #if defined(GET_OPTA_OTP_BOARD_INFO)
    info = boardInfo();
    if (info->magic == 0xB5) {
      for (int i = 0; i < 6; i++) {
        mac[i] = info->mac_address[i];
      }
    }
  #endif

  if (Ethernet.begin(mac, ip)) {
    Serial.println(Ethernet.localIP());
  } else{
    Serial.println("Ethernet failed");
  }

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

  // Initialize the key-value store
  Serial.print("Initializing TDBStore: ");
  auto result = store.init();
  Serial.println(result == MBED_SUCCESS ? "OK" : "Failed");
  if (result != MBED_SUCCESS)
    while (true); // Stop the sketch if an error occurs

  // Get previous run stats from the key-value store
  Serial.println("Retrieving Limits");
  result = getLimits(limitsKey, &currentLimits);

  if (result == MBED_SUCCESS) {
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
  } else if (result == MBED_ERROR_ITEM_NOT_FOUND) {
    Serial.println("No previous data was found.");
  } else {
    Serial.println("Error reading from key-value store.");
    while (true);
  }

  // set pins for LEDs
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_D0, OUTPUT);

  // for Kemo M152K rain sensor
  pinMode(rainSensorSafePin, INPUT);
  attachInterrupt(digitalPinToInterrupt(rainSensorSafePin), rainSensorSafeISR, CHANGE);

  // mount the handler to the default router
  // app.use(&fillContext); // middleware
  app.get("/", &index);
  app.post("/submit", &submit);

  app.get("/api/v1/safetymonitor/0/connected", &endPoint);
  app.put("/api/v1/safetymonitor/0/connected", &endPoint);
  app.get("/api/v1/safetymonitor/0/issafe", &endPoint);
  app.get("/api/v1/safetymonitor/0/name", &endPoint);
  app.get("/api/v1/safetymonitor/0/driverversion", &endPoint);

  app.get("/api/v1/observingconditions/0/connected", &endPoint);
  app.put("/api/v1/observingconditions/0/connected", &endPoint);
  app.get("/api/v1/observingconditions/0/name", &endPoint);
  app.get("/api/v1/observingconditions/0/driverversion", &endPoint);

  app.get("/api/v1/observingconditions/0/windspeed", &endPoint);
  app.get("/api/v1/observingconditions/0/windgust", &endPoint);
  app.get("/api/v1/observingconditions/0/winddirection", &endPoint);

  app.get("/api/v1/observingconditions/0/rainrate", &endPoint);

  app.get("/api/v1/observingconditions/0/temperature", &endPoint);
  app.get("/api/v1/observingconditions/0/humidity", &endPoint);
  app.get("/api/v1/observingconditions/0/dewpoint", &endPoint);
  app.get("/api/v1/observingconditions/0/pressure", &endPoint);

  // app.get("/api/v1/observingconditions/0/cloudcover", &endPoint);
  app.get("/api/v1/observingconditions/0/skybrightness", &endPoint);
  // app.get("/api/v1/observingconditions/0/skyquality", &endPoint);
  // app.get("/api/v1/observingconditions/0/skytemperature", &endPoint);
  // app.get("/api/v1/observingconditions/0/starfwhm", &endPoint);

  // app.get("/api/v1/observingconditions/0/averageperiod", &endPoint);
  // app.put("/api/v1/observingconditions/0/averageperiod", &endPoint);
  // app.put("/api/v1/observingconditions/0/refresh", &endPoint);
  // app.get("/api/v1/observingconditions/0/sensordescription", &endPoint); 0XA; MD=?<CR><LF> 0XA; VE=?<CR><LF> 0XA; TP=?<CR><LF> 0XA; S/N=?<CR><LF> 0XA; NA=?<CR><LF>
  // app.get("/api/v1/observingconditions/0/timesincelastupdate", &endPoint);

  // app.finally(&setStatus); // middleware

  server.begin();

  // RS485 init
  RS485.begin(baudrate);
  RS485.setDelays(preDelayBR, postDelayBR);

  // start SensorReadThread thread
  SensorReadThread.start(readSensors);
}

// loop
void loop(){

  EthernetClient client = server.available();

  if (client.connected()) {
    app.process(&client);
    client.stop();
  }

}

// opta manual
// https://docs.arduino.cc/tutorials/opta/user-manual/

// ascii commands
// https://files.seeedstudio.com/products/SenseCAP/SenseCAP_ONE/SenseCAP_ONE_V2_Compact_Weather_Station_User_Guide.pdf

// alpaca commands
// https://ascom-standards.org/api/
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

// TODO:
// - automatically get mac address (see example on opta website)
// - would be nice if we could set the IP from the web interface