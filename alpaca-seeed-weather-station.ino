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
float AT_low = 0; // Air temperature low limit C
float AT_high = 30; // Air temperature high limit C
float AH_low = 0; // Air humidity low limit %RH
float AH_high = 80; // Air humidity high limit %RH
float AP_low = 90000; // Barometric pressure low limit Pa
float AP_high = 110000; // Barometric pressure high limit Pa
float LX_high = 10000; // Light intensity high limit Lux
float SM_high = 15; // Gust wind speed high limit m/s (default), km/h, mph, knots
float SA_high = 15; // Average wind speed high limit m/s (default), km/h, mph, knots

// These require user resetting (see manual)
// float RA_high = 0; // Accumulated rainfall high limit mm (default), in
// float RD_high = 0; // Duration of rainfall high limit s

// Resets after an hour?
float RI_high = 0; // Rainfall intensity high limit mm/h (default), in/h
float Rp_high = 0; // Maximum rainfall intensity high limit mm/h (default), in/h
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
  if (vals[S700_key["AT"]] < AT_low || vals[S700_key["AT"]] > AT_high) {
    Serial.println("Temperature out of range");
    return false;
  }
  // humidity
  if (vals[S700_key["AH"]] < AH_low || vals[S700_key["AH"]] > AH_high) {
    Serial.println("Humidity out of range");
    return false;
  }
  // pressure
  if (vals[S700_key["AP"]] < AP_low || vals[S700_key["AP"]] > AP_high) {
    Serial.println("Pressure out of range");
    return false;
  }

  // light intensity
  if (vals[S700_key["LX"]] > LX_high) {
    Serial.println("Light intensity out of range");
    return false;
  }

  // gust wind speed
  if (vals[S700_key["SM"]] > SM_high) {
    Serial.println("Wind speed out of range");
    return false;
  }

  // average wind speed
  if (vals[S700_key["SA"]] > SA_high) {
    Serial.println("Wind speed out of range");
    return false;
  }

  // these require user resetting (see manual)
  // accumulated rainfall
  // if (vals[S700_key["RA"]] > RA_high) {
  //   Serial.println("Rainfall out of range");
  //   return false;
  // }

  // // duration of rainfall
  // if (vals[S700_key["RD"]] > RD_high) {
  //   Serial.println("Rainfall duration out of range");
  //   return false;
  // }

  // rainfall intensity
  if (vals[S700_key["RI"]] > RI_high) {
    Serial.println("Rainfall intensity out of range");
    return false;
  }

  // maximum rainfall intensity
  if (vals[S700_key["Rp"]] > Rp_high) {
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
    Value = vals[1];
  } else if (url == "dewpoint") {
    // TODO: check this okay
    // source?
    float B = (log(vals[1] / 100) + ((17.27 * vals[0]) / (237.3 + vals[0]))) / 17.27;
    float dewpoint = (237.3 * B) / (1 - B);
    Value = dewpoint;
  } else if (url == "pressure") {
    Value = vals[2];
  } else if (url == "windspeed") {
    Value = vals[9];
  } else if (url == "windgust") {
    Value = vals[8];
  } else if (url == "winddirection") {
    Value = vals[6];
  } else if (url == "rainrate") {
    Value = vals[12];
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

  String index_html = R"(
  <!DOCTYPE html>
  <html lang="en">
  <head>
      <meta charset="UTF-8">
      <meta name="viewport" content="width=device-width, initial-scale=1.0">
      <title>Configure safety parameters</title>
  </head>
  <body>
      <h1>Configure safety parameters</h1>
      <form action="/submit" method="post">
          <label for="maxTemperature">Max temperature:</label>
          <input type="number" name="maxTemperature" id="maxTemperature" value=")" + String(AT_high) + R"(" required>
          <br>
          <label for="minTemperature">Min temperature:</label>
          <input type="number" name="minTemperature" id="minTemperature" value=")" + String(AT_low) + R"(" required>
          <br>
          <label for="maxHumidity">Max humidity:</label>
          <input type="number" name="maxHumidity" id="maxHumidity" value=")" + String(AH_high) + R"(" required>
          <br>
          <label for="minHumidity">Min humidity:</label>
          <input type="number" name="minHumidity" id="minHumidity" value=")" + String(AH_low) + R"(" required>
          <br>
          <label for="maxPressure">Max pressure:</label>
          <input type="number" name="maxPressure" id="maxPressure" value=")" + String(AP_high) + R"(" required>
          <br>
          <label for="minPressure">Min pressure:</label>
          <input type="number" name="minPressure" id="minPressure" value=")" + String(AP_low) + R"(" required>
          <br>
          <input type="submit" value="Submit">
      </form>
  </body>
  </html>
  )";

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

    if (strcmp(name, "maxTemperature") == 0) {
      AT_high = atof(value);
    } else if (strcmp(name, "minTemperature") == 0) {
      AT_low = atof(value);
    } else if (strcmp(name, "maxHumidity") == 0) {
      AH_high = atof(value);
    } else if (strcmp(name, "minHumidity") == 0) {
      AH_low = atof(value);
    } else if (strcmp(name, "maxPressure") == 0) {
      AP_high = atof(value);
    } else if (strcmp(name, "minPressure") == 0) {
      AP_low = atof(value);
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