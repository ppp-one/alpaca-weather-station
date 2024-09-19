#include <ArduinoRS485.h>
#include <Ethernet.h>
#include <aWOT.h>
#include <cmath>
#include <unordered_map>
#include <string>

static rtos::Thread SensorReadThread;
float vals[16];

// the media access control (ethernet hardware) address
byte mac[] = { 0xA8, 0x61, 0x0A, 0x50, 0x91, 0x69 };
//the IP address
byte ip[] = { 192, 168, 0, 12 };

// RS485
constexpr auto baudrate{ 9600 };
// Calculate preDelay and postDelay in microseconds for stable RS-485 transmission
constexpr auto bitduration{ 1.f / baudrate };
constexpr auto wordlen{ 9.6f };  // OR 10.0f depending on the channel configuration
constexpr auto preDelayBR{ bitduration * wordlen * 3.5f * 1e6 };
constexpr auto postDelayBR{ bitduration * wordlen * 3.5f * 1e6 };

// is_safe
bool is_safe = false;

// safety limits
int safety_false_time = 10; // Time in seconds to keep is_safe false if not safe
float AT_min = 0; // Air temperature low limit C
float AT_max = 30; // Air temperature high limit C
float AH_min = 0; // Air humidity low limit %RH
float AH_max = 80; // Air humidity high limit %RH
// float AP_min = 90000; // Barometric pressure low limit Pa
// float AP_max = 110000; // Barometric pressure high limit Pa
float LX_max = 10000; // Light intensity high limit Lux
float SM_max = 15; // Gust wind speed high limit m/s (default), km/h, mph, knots
float SA_max = 15; // Average wind speed high limit m/s (default), km/h, mph, knots

// These require user resetting (see manual)
// float RA_max = 0; // Accumulated rainfall high limit mm (default), in
// float RD_max = 0; // Duration of rainfall high limit s

// Resets after an hour?
float RI_max = 0; // Rainfall intensity high limit mm/h (default), in/h
float Rp_max = 0; // Maximum rainfall intensity high limit mm/h (default), in/h
// add more later

// S700 array
std::unordered_map<std::string, int> S700_key = {
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

int32_t ErrorNumber = 0;
String ErrorMessage = "\"\"";

EthernetServer server(80);
Application app;

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

void read_sensors()
{
  auto time_since_not_safe = rtos::Kernel::Clock::now();
  bool weather_station_data_received = false;
  bool safety_check_received = false;

  while(true) {
    // TODO: wrap in try catch

    digitalWrite(LED_BUILTIN, HIGH);

    // read weather station data
    weather_station_data_received = read_weather_station();

    // if no data received, try once more
    if (!weather_station_data_received) {
      rtos::ThisThread::sleep_for(1000);
      weather_station_data_received = read_weather_station();
    }

    // safety check of weather station data
    safety_check_received = safety_check();

    // set is_safe
    is_safe = weather_station_data_received && safety_check_received;

    // if not safe, keep is_safe false for 10 s, but continue to read sensors
    if (!is_safe) {
      time_since_not_safe = rtos::Kernel::Clock::now();
    }
    if ((rtos::Kernel::Clock::now() - time_since_not_safe > std::chrono::seconds(safety_false_time)) && is_safe) {
      is_safe = true;
    } else {
      is_safe = false;
    }

    // testing LED
    if (!is_safe) {
      digitalWrite(LED_D0, HIGH);
    } else {
      digitalWrite(LED_D0, LOW);
    }

    rtos::ThisThread::sleep_for(500);
    digitalWrite(LED_BUILTIN, LOW);
    rtos::ThisThread::sleep_for(500);

  }
}

bool read_weather_station() {
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
          Serial.print("valint: ");
          Serial.print(valint);
          Serial.print(" ");
          Serial.print(vi);
          Serial.print(" ");
          Serial.println(vi.substring(3).toFloat());
          

          vals[valint] = vi.substring(3).toFloat();
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

bool safety_check() {
  // check if any values are outside of the safety limits
  // temperature
  if (vals[S700_key["AT"]] < AT_min || vals[S700_key["AT"]] > AT_max) {
    Serial.println("Temperature out of range");
    return false;
  }
  // humidity
  if (vals[S700_key["AH"]] < AH_min || vals[S700_key["AH"]] > AH_max) {
    Serial.println("Humidity out of range");
    return false;
  }
  // pressure
  // if (vals[S700_key["AP"]] < AP_min || vals[S700_key["AP"]] > AP_max) {
  //   Serial.println("Pressure out of range");
  //   return false;
  // }

  // light intensity
  if (vals[S700_key["LX"]] > LX_max) {
    Serial.println("Light intensity out of range");
    return false;
  }

  // gust wind speed
  if (vals[S700_key["SM"]] > SM_max) {
    Serial.println("Wind speed out of range");
    return false;
  }

  // average wind speed
  if (vals[S700_key["SA"]] > SA_max) {
    Serial.println("Wind speed out of range");
    return false;
  }

  // these require user resetting (see manual)
  // // accumulated rainfall
  // if (vals[S700_key["RA"]] > RA_max) {
  //   Serial.println("Rainfall out of range");
  //   return false;
  // }

  // // duration of rainfall
  // if (vals[S700_key["RD"]] > RD_max) {
  //   Serial.println("Rainfall duration out of range");
  //   return false;
  // }

  // rainfall intensity
  if (vals[S700_key["RI"]] > RI_max) {
    Serial.println("Rainfall intensity out of range");
    return false;
  }

  // maximum rainfall intensity
  if (vals[S700_key["Rp"]] > Rp_max) {
    Serial.println("Rainfall intensity out of range");
    return false;
  }

  // check if rainsensor is active, was reading 168 when active? Need to check with voltmeter
  if (analogRead(A1) > 10) { // TODO: should this trigger an interrupt instead?
    Serial.println("Rain detected");
    return false;
  }

  return true;
}

void endPoint(Request &req, Response &res) {

  res.set("Content-Type", "application/json");
  auto url = split(req.path(), '/', 5);

  uint32_t ClientID = 0;
  uint32_t ClientTransactionID = 0;

  uint32_t ServerID = 0; // TODO: what is this?
  uint32_t ServerTransactionID = 0; // TODO: what is this? increment?

  float Value;

  char buffer[16]; // Adjust the buffer size as needed

  if (req.query("ClientID", buffer, sizeof(buffer))) {
    ClientID = strtoul(buffer, NULL, 10);
  }

  if (req.query("ClientTransactionID", buffer, sizeof(buffer))) {
    ClientTransactionID = strtoul(buffer, NULL, 10);
  }

  if (url == "connected") {
    Value = 1;
  } else if (url == "issafe") {
    Value = is_safe;
  } else if (url == "temperature") {
    Value = vals[S700_key["AT"]];
  } else if (url == "humidity") {
    Value = vals[S700_key["AH"]];
  } else if (url == "dewpoint") {
    // TODO: check this okay
    // source?
    float B = (log(vals[1] / 100) + ((17.27 * vals[0]) / (237.3 + vals[0]))) / 17.27;
    float dewpoint = (237.3 * B) / (1 - B);
    Value = dewpoint;
  } else if (url == "pressure") {
    Value = vals[S700_key["AP"]];
  } else if (url == "windspeed") {
    Value = vals[S700_key["SA"]];
  } else if (url == "windgust") {
    Value = vals[S700_key["SM"]];
  } else if (url == "winddirection") {
    Value = vals[S700_key["DA"]];
  } else if (url == "rainrate") {
    Value = vals[S700_key["RI"]];
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
  res.print(Value);
  res.print("}");

  // reset error
  ErrorNumber = 0;
  ErrorMessage = "\"\"";

}

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
                      <div title="Current: )~" + String(AT_min) + R"~( °C">
                          <label for="min-temp">Min</label>
                          <input type="number" id="min-temp" name="min-temp" value=)~" + String(AT_min) + R"~( required>
                      </div>
                      <div title="Current: )~" + String(AT_max) + R"~( °C">
                          <label for="max-temp">Max</label>
                          <input type="number" id="max-temp" name="max-temp" value=)~" + String(AT_max) + R"~( required>
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
                      <div title="Current: )~" + String(AH_min) + R"~( %">
                          <label for="min-rh">Min</label>
                          <input type="number" id="min-rh" name="min-rh" value=)~" + String(AH_min) + R"~( min=0 required>
                      </div>
                      <div title="Current: )~" + String(AH_max) + R"~( %">
                          <label for="max-rh">Max</label>
                          <input type="number" id="max-rh" name="max-rh" value=)~" + String(AH_max) + R"~( max=100 required>
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
                      <div title="Current: )~" + String(SA_max) + R"~( m/s">
                          <label for="max-wind">Max</label>
                          <input type="number" id="max-wind" name="max-wind" value=)~" + String(SA_max) + R"~( min=0 required>
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
                      <div title="Current: )~" + String(SM_max) + R"~( m/s">
                          <label for="max-gust-wind">Max</label>
                          <input type="number" id="max-gust-wind" name="max-gust-wind" value=)~" + String(SM_max) + R"~( min=0 required>
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
                      <div title="Current: )~" + String(LX_max) + R"~( lux">
                          <label for="max-light">Max</label>
                          <input type="number" id="max-light" name="max-light" value=)~" + String(LX_max) + R"~( min=0 required>
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

void submit(Request &req, Response &res) {

  char name[32];
  char value[32];

  while (req.left()) {
    if (!req.form(name, 32, value, 32)) {
      return res.sendStatus(400);
    }

    res.print(name);
    res.print(":");
    res.println(value);

    if (strcmp(name, "max-temp") == 0) {
      AT_max = atof(value);
    } else if (strcmp(name, "min-temp") == 0) {
      AT_min = atof(value);
    } else if (strcmp(name, "max-rh") == 0) {
      AH_max = atof(value);
    } else if (strcmp(name, "min-rh") == 0) {
      AH_min = atof(value);
    } else if (strcmp(name, "max-light") == 0) {
      LX_max = atof(value);
    } else if (strcmp(name, "max-wind") == 0) {
      SA_max = atof(value);
    } else if (strcmp(name, "max-gust-wind") == 0) {
      SM_max = atof(value);
    } 
  }

}

void setup() {
  
  Serial.begin(115200);

  if (Ethernet.begin(mac, ip)) {
    Serial.println(Ethernet.localIP());
  } else{
    Serial.println("Ethernet failed");
  }

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_D0, OUTPUT);

  // for Kemo M152K rain sensor
  // set pin A0 to HIGH
  pinMode(A0, OUTPUT);
  digitalWrite(A0, HIGH);
  // set pin A1 to INPUT
  pinMode(A1, INPUT);

  // mount the handler to the default router
  // app.use(&fillContext); // middleware
  app.get("/", &index);
  app.post("/submit", &submit);

  app.get("/api/v1/safetymonitor/0/connected", &endPoint);
  app.get("/api/v1/safetymonitor/0/issafe", &endPoint);


  app.get("/api/v1/observingconditions/0/connected", &endPoint);
  // app.get("/api/v1/observingconditions/0/name", &endPoint);
  // app.get("/api/v1/observingconditions/0/driverversion", &endPoint);

  app.get("/api/v1/observingconditions/0/windspeed", &endPoint);
  app.get("/api/v1/observingconditions/0/windgust", &endPoint);
  app.get("/api/v1/observingconditions/0/winddirection", &endPoint);

  app.get("/api/v1/observingconditions/0/rainrate", &endPoint);

  app.get("/api/v1/observingconditions/0/temperature", &endPoint);
  app.get("/api/v1/observingconditions/0/humidity", &endPoint);
  app.get("/api/v1/observingconditions/0/dewpoint", &endPoint);
  app.get("/api/v1/observingconditions/0/pressure", &endPoint);

  // app.get("/api/v1/observingconditions/0/cloudcover", &endPoint);
  // app.get("/api/v1/observingconditions/0/skybrightness", &endPoint);
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
  SensorReadThread.start(read_sensors);
}

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

// arduino-cli compile --fqbn arduino:mbed_opta:opta ./   
// arduino-cli upload -p /dev/cu.usbmodem1301 --fqbn arduino:mbed_opta:opta ./

// ADD USER INPUT FOR THRESHOLDS, STORE IN EEPROM? Or something that can be read from cold boot