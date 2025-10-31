# Alpaca Weather Station

Custom Arduino Opta PLC weather station and safety controller for observatory automation that monitors environmental conditions and provides real-time weather data to prevent telescope damage.

## Purpose

This PLC acts as a comprehensive weather monitoring system that:
- Reads weather data from a SenseCAP S700 weather station via RS485
- Monitors sky temperature using infrared sensors
- Detects rain using a dedicated rain sensor
- Checks all conditions against configurable safety limits
- Controls a safety relay to cut power when conditions become unsafe
- Exposes ASCOM Alpaca ObservingConditions and SafetyMonitor APIs
- Provides a web interface for configuring safety limits

## How It Works

### Main Components

The system continuously monitors weather conditions using three hardware interfaces:

1. **S700 Weather Station** (RS485): Provides 16 weather parameters including:
   - Air temperature and humidity
   - Barometric pressure
   - Light intensity
   - Wind speed, gust speed, and direction
   - Rainfall data

2. **MLX90614 IR Sensors** (I2C): Two sensors measure sky temperature
   - Detects cloud cover (clouds are warmer than clear sky)
   - Calculates relative sky temperature vs. ambient

3. **Rain Sensor** (Analog pin A7): Kemo M152K sensor for direct rain detection
   - Provides backup to weather station rain detection
   - More sensitive for immediate response

### Logic Flow

```
┌─────────────────────────────────────────────────────────┐
│                    MAIN LOOP (loop)                     │
│  - Listens for HTTP requests                            │
│  - Serves web interface and API                         │
│  - Kicks watchdog timer                                 │
└─────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────┐
│            SENSOR THREAD (readSensors)                  │
│  Runs continuously in background every ~1 second        │
└─────────────────────────────────────────────────────────┘
                              ↓
        ┌─────────────────────┴─────────────────────┐
        ↓                                           ↓
┌──────────────────┐                    ┌──────────────────────┐
│ Read Weather     │                    │ Read Sky Temp        │
│ Station (RS485)  │                    │ Sensors (I2C)        │
│ - Send "0XA;G0?" │                    │ - MLX1 (0x5A)        │
│ - Parse response │                    │ - MLX2 (0x5B)        │
│ - Store 16 values│                    │ - Calculate relative │
└──────────────────┘                    └──────────────────────┘
        ↓                                           ↓
        └─────────────────────┬─────────────────────┘
                              ↓
                 ┌────────────────────────┐
                 │   SAFETY CHECK         │
                 │  (safetyCheck)         │
                 └────────────────────────┘
                              ↓
        ┌─────────────────────┴─────────────────────┐
        ↓                                           ↓
┌──────────────────┐                    ┌──────────────────────┐
│ Check Limits     │                    │ Check Rain Sensor    │
│ - Temperature    │                    │ - Analog pin > 10    │
│ - Humidity       │                    │ - Sets rainSensorSafe│
│ - Wind speeds    │                    └──────────────────────┘
│ - Light          │                               ↓
│ - Sky temp       │                    ┌──────────────────────┐
│ - Rainfall       │                    │   SET SAFETY FLAG    │
└──────────────────┘                    │  (isSafe = true/false)│
        ↓                                └──────────────────────┘
        └─────────────────────┬─────────────────────┘
                              ↓
                 ┌────────────────────────┐
                 │   CONTROL RELAY        │
                 │  - D0 relay            │
                 │  - LED_D0 indicator    │
                 │  - 10s delay on unsafe │
                 └────────────────────────┘
```

### Safety Logic

The system evaluates safety by checking:

| Parameter | Safe Condition | Default Limits |
|-----------|---------------|----------------|
| Air Temperature | AT_min ≤ temp ≤ AT_max | 0°C to 50°C |
| Humidity | AH_min ≤ RH ≤ AH_max | 0% to 80% |
| Light Intensity | lux ≤ LX_max | ≤ 10,000 lux |
| Wind Speed (avg) | speed ≤ SA_max | ≤ 15 m/s |
| Wind Gust | gust ≤ SM_max | ≤ 20 m/s |
| Sky Temperature | ST_min ≤ temp ≤ ST_max | -80°C to -30°C |
| Relative Sky Temp | temp ≤ RST_max | ≤ -20°C |
| Rain Detection | No active rain sensor | Digital/Analog |
| Rainfall Rate | RI = 0 and Rp = 0 | 0 mm/h |

**When ANY condition fails:**
1. `isSafe` flag is set to `false`
2. Safety relay (D0) switches to LOW (power cut)
3. LED indicator (LED_D0) turns off
4. System maintains unsafe state for configurable duration (default 10s)
5. Even if conditions improve, relay stays off for the full duration

**When ALL conditions pass:**
1. After the safety duration expires, `isSafe` becomes `true`
2. Safety relay switches to HIGH (power restored)
3. LED indicator turns on

### Multi-Threading Architecture

The code uses **RTOS threads** to handle concurrent operations:

- **Main Thread** (`loop`): Handles HTTP requests, web interface, API calls
- **Sensor Thread** (`readSensors`): Continuously reads sensors and evaluates safety
- **Watchdog Timer**: Resets the system if it hangs (20-second timeout)

This ensures the web interface stays responsive even while sensors are being polled.

## Hardware Setup

**Arduino Opta PLC** with:
- **Ethernet connection** (required for network APIs)
- **RS485 interface** for S700 weather station (9600 baud)
- **I2C bus** (pins 11, 12) for two MLX90614 sensors at addresses 0x5A and 0x5B
- **Analog input A7** for Kemo M152K rain sensor
- **Analog output A6** provides HIGH signal to rain sensor
- **Digital output D0** controls safety relay
- **LED D0** indicates safety status
- **Built-in LED** flashes during normal operation

**Sensor Wiring:**
- S700 Weather Station: Connect to RS485 A/B terminals
- MLX90614 sensors: Connect to I2C SDA/SCL (pins 11, 12)
- Rain sensor: Signal to A7, power HIGH on A6

## Web Interface

Access the configuration UI at `http://192.168.1.12/`

The interface provides a dark-themed form to configure all safety limits:
- Temperature range (min/max in °C)
- Humidity range (min/max in %)
- Wind speed maximum (m/s)
- Gust wind maximum (m/s)
- Light intensity maximum (lux)
- Sky temperature range (min/max in °C)
- Relative sky temperature maximum (°C)
- Safety false duration (seconds to maintain unsafe state)

All settings are **persisted to flash memory** and survive power cycles.

## ASCOM Alpaca APIs

### Safety Monitor API
Base URL: `http://192.168.1.12/api/v1/safetymonitor/0/`

| Endpoint | Method | Description | Returns |
|----------|--------|-------------|---------|
| `/connected` | GET/PUT | Check connection status | `{ "Value": true }` |
| `/issafe` | GET | Get overall safety status | `{ "Value": true/false }` |
| `/name` | GET | Device name | `{ "Value": "Weather Station..." }` |
| `/driverversion` | GET | Driver version | `{ "Value": "0.0.2" }` |

### Observing Conditions API
Base URL: `http://192.168.1.12/api/v1/observingconditions/0/`

| Endpoint | Description | Units | Value Source |
|----------|-------------|-------|--------------|
| `/temperature` | Ambient air temperature | °C | S700 AT |
| `/humidity` | Relative humidity | % | S700 AH |
| `/dewpoint` | Calculated dew point | °C | Formula* |
| `/pressure` | Barometric pressure | hPa | S700 AP |
| `/skybrightness` | Light intensity | lux | S700 LX |
| `/windspeed` | Average wind speed | m/s | S700 SA |
| `/windgust` | Peak wind gust | m/s | S700 SM |
| `/winddirection` | Average wind direction | degrees | S700 DA |
| `/rainrate` | Rainfall intensity | mm/h | S700 RI + sensor |
| `/skytemperature` | IR sky temperature | °C | Max(MLX1, MLX2) |
| `/relativeskytemperature` | Sky temp - air temp | °C | Calculated |
| `/rawmlx` | Raw MLX sensor data | JSON | Both sensors |

*Dew point uses formula from Górnicki et al., 2017 (M1 model)

All endpoints return ASCOM-compliant JSON with `ClientID`, `ClientTransactionID`, `ErrorNumber`, `ErrorMessage`, and `Value` fields.

## Compilation & Upload

Requires [arduino-cli](https://arduino.github.io/arduino-cli/) installed with the Arduino Mbed Opta core (custom core: https://github.com/ppp-one/ArduinoCore-mbed, or use the default Arduino Mbed core, however HTTP queries will be slower with the default core).

```bash
# Compile (adjust FQBN to your installed core)
arduino-cli compile --fqbn arduino-git:mbed:opta ./

# Upload to Opta PLC (replace port as needed)
arduino-cli upload -p /dev/cu.usbmodem1301 --fqbn arduino-git:mbed:opta ./
```

## Configuration

Edit the IP address in `alpaca-weather-station.ino`:
```cpp
byte ip[] = {192, 168, 1, 12};  // Change to match your network
```

Default safety limits can be modified in the `WeatherLimits` struct at the top of the file.

## Maintenance & Troubleshooting

### LED Status Indicators
- **Built-in LED**: Flashes every ~1 second when operating normally
- **LED_D0**: ON when safe, OFF when unsafe

### Common Issues

**No data from weather station:**
- Check RS485 wiring (A/B terminals)
- Verify weather station address is `0XA` (check with manual commands)
- Ensure baud rate is 9600
- Serial monitor will show "No data received from weather station"

**MLX sensors not detected:**
- Verify I2C addresses (0x5A and 0x5B)
- Check SDA/SCL wiring to pins 11, 12
- System will print "Communication with device failed" and wait

**Safety relay not triggering:**
- Check if limits are properly configured via web interface
- Verify rain sensor connection to A7
- Use Serial monitor to see which limit is being violated
- Check `safety_false_duration` setting

**Settings not persisting:**
- Flash memory initialization error - check Serial output
- Ensure Flash IAP block device initialized successfully
- Try resetting to defaults and reconfiguring

### Serial Debugging

Connect to Serial port at 9600 baud to see:
- Weather station communication
- Safety check failures (which limit was violated)
- Sensor initialization status
- Flash memory operations
- Network configuration

### Watchdog Timer

The system includes a 20-second watchdog that will reset the PLC if:
- The sensor thread hangs
- Communication stalls
- The main loop stops responding

This ensures the system recovers automatically from crashes.

## Code Structure

### Key Functions

| Function | Purpose | Thread |
|----------|---------|--------|
| `setup()` | Initialize hardware, network, sensors, flash storage | Main |
| `loop()` | Handle HTTP requests, kick watchdog | Main |
| `readSensors()` | Read all sensors every ~1s, evaluate safety | Sensor |
| `readWeatherStation()` | Query S700 via RS485, parse response | Sensor |
| `safetyCheck()` | Compare all readings against limits | Sensor |
| `index()` | Serve HTML form for safety limits | Main |
| `submit()` | Process form submission, save to flash | Main |
| `endPoint()` | Handle ASCOM API requests | Main |
| `getLimits()` | Read safety limits from flash | Both |
| `setLimits()` | Write safety limits to flash | Main |

### Data Flow

1. **Sensor Thread** reads hardware → stores in global variables
2. **Main Thread** reads global variables → sends via HTTP
3. **Flash Storage** persists `currentLimits` struct across reboots
4. **Safety Relay** directly controlled by sensor thread

### Global Variables (Shared Between Threads)

- `isSafe`: Current safety status (read by API, written by sensor thread)
- `WSValues[16]`: Weather station readings array
- `skyTemperature`, `relativeSkyTemperature`: IR sensor values
- `currentLimits`: Active safety thresholds
- `ErrorNumber`, `ErrorMessage`: ASCOM error state

## References

- [ASCOM Alpaca API Specification](https://ascom-standards.org/api/)
- [SenseCAP S700 User Guide](https://files.seeedstudio.com/products/SenseCAP/SenseCAP_ONE/SenseCAP_ONE_V2_Compact_Weather_Station_User_Guide.pdf)
- [Arduino Opta Manual](https://docs.arduino.cc/tutorials/opta/user-manual/)
- [MLX90614 IR Sensor Datasheet](https://www.melexis.com/en/product/MLX90614/)
- [Multithreading on Opta](https://opta.findernet.com/en/tutorial/multithreading-opta-and-serie-7m)

## License

MIT License - see [LICENSE](LICENSE)
