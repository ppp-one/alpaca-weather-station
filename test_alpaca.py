from datetime import datetime

from alpaca.observingconditions import *
from alpaca.safetymonitor import *

WS = ObservingConditions("192.168.0.12", 0)
SM = SafetyMonitor("192.168.0.12", 0)

# WS.Connected = True

# print("Connected: ", WS.Connected)
# print("Name: ", WS.Name)
# print("DriverVersion: ", WS.DriverVersion)
counter = 0
start = datetime.now()
for i in range(10):
    data = {
        "timestamp": datetime.now(),
        "dewpoint": WS.DewPoint,
        "humidity": WS.Humidity,
        "temperature": WS.Temperature,
        "windspeed": WS.WindSpeed,
        "windgust": WS.WindGust,
        "winddirection": WS.WindDirection,
        "pressure": WS.Pressure,
        "rainrate": WS.RainRate,
        "skybrightness": WS.SkyBrightness,
        # "skytemperature": WS.SkyTemperature,
        "safe": SM.IsSafe,
        "counter": counter + 1,
    }
    counter += 1

    print(data)

    # time.sleep(1)

print("Elapsed time per query: ", (datetime.now() - start) / counter)
