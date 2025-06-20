import csv
from datetime import datetime, timedelta

from alpaca.observingconditions import ObservingConditions
from alpaca.safetymonitor import SafetyMonitor
from matplotlib.animation import FuncAnimation

# Initialize devices
WS = ObservingConditions("192.168.1.12", 0)
SM = SafetyMonitor("192.168.1.12", 0)


for i in range(10):
    current_time = datetime.now()

    # Collect data
    new_data = {
        "dewpoint": WS.DewPoint,
        "humidity": WS.Humidity,
        "temperature": WS.Temperature,
        "windspeed": WS.WindSpeed,
        "windgust": WS.WindGust,
        "winddirection": WS.WindDirection,
        "pressure": WS.Pressure,
        "rainrate": WS.RainRate,
        "skybrightness": WS.SkyBrightness,
        "skytemperature": WS.SkyTemperature,
        "safe": int(SM.IsSafe),  # Convert boolean to int for plotting
    }

    end_time = datetime.now()
    time_per_request = (end_time - current_time).total_seconds() / len(new_data.keys())

    print(f"Collected data at {current_time}: {new_data}")
    print(f"Time per request: {time_per_request:.3f} seconds")
