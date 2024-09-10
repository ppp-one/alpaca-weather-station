#include <ArduinoRS485.h>
#include <Ethernet.h>
#include <aWOT.h>
#include <cmath>

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

// isSafe
bool isSafe = false;

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

void S700_array()
{
  char separator = ';';
  while(1) {
    digitalWrite(LED_BUILTIN, HIGH);
    String data = readRS485();

    if (data == "") {
      Serial.println("No data");
      rtos::ThisThread::sleep_for(1000);
      // TODO: consider changing safety to 0 if it happens too often or data stale
      continue;
    }


    int valint = 0;

    int found = 0;
    int strIndex[] = {0, -1};
    int maxIndex = data.length()-1;

    for(int i=0; i<=maxIndex; i++){
      if(data.charAt(i)==separator || i==maxIndex){
          found++;
          strIndex[0] = strIndex[1]+1;
          strIndex[1] = (i == maxIndex) ? i+1 : i;
      }

      if (found == 2) {
        auto vi = data.substring(strIndex[0], strIndex[1]);
        found=1;

        if (vi.length() > 2 && valint < 15) {
          vals[valint] = vi.substring(3).toFloat();
          valint++;
        }
      }
    }

    rtos::ThisThread::sleep_for(250);

    digitalWrite(LED_BUILTIN, LOW);

    // check if rainsensor is active, was reading 168 when active? Need to check with voltmeter
    if (analogRead(A1) > 10) {
      isSafe = false;
    } else {
      isSafe = true;
    }

    rtos::ThisThread::sleep_for(250);

  }
}

String readRS485() {
  String val = "";

  RS485.noReceive();
  RS485.beginTransmission();
  RS485.println("0XA;G0?");
  RS485.endTransmission();
  RS485.flush();
  RS485.receive();

  if (RS485.available()) {
    auto peeked = RS485.peek();
    while (peeked != -1) {
      char c;
      uint8_t b = RS485.read();

      c = (char)b;
      val += c;

      peeked = RS485.peek();
    }
  }

  // Serial.println(val);

  // if it begins with "0XA;AT" and ends with "\r\n", then it's a valid response
  if (val.startsWith("0XA;AT") && val.endsWith("\r\n")) {
    return val;
  } else {
    // Serial.println(val);
    return "";
  }

}

// define a handler function
void endPoint(Request &req, Response &res) {

  res.set("Content-Type", "application/json");
  auto url = split(req.path(), '/', 5);

  uint32_t ClientID = 0;
  uint32_t ClientTransactionID = 0;

  uint32_t ServerID = 0;
  uint32_t ServerTransactionID = 0;

  int32_t ErrorNumber = 0;
  String ErrorMessage = "\"\"";

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
    Value = isSafe;
  } else if (url == "temperature") {
    Value = vals[0];
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

}


void setup() {
  
  Serial.begin(115200);

  if (Ethernet.begin(mac, ip)) {
    Serial.println(Ethernet.localIP());
  } else{
    Serial.println("Ethernet failed");
  }

  pinMode(LED_BUILTIN, OUTPUT);

  // for Kemo M152K rain sensor
  // set pin A0 to HIGH
  pinMode(A0, OUTPUT);
  digitalWrite(A0, HIGH);
  // set pin A1 to INPUT
  pinMode(A1, INPUT);

  // mount the handler to the default router
  // app.use(&fillContext);
  app.get("/", &endPoint);

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

  // app.finally(&setStatus);

  server.begin();

  // RS485 init
  RS485.begin(baudrate);
  RS485.setDelays(preDelayBR, postDelayBR);
  
  // Enable data reception
  // RS485.receive();

  // start SensorReadThread thread
  SensorReadThread.start(S700_array);
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