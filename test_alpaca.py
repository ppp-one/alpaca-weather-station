import csv
from datetime import datetime, timedelta

import matplotlib.pyplot as plt
import numpy as np
from alpaca.observingconditions import ObservingConditions
from alpaca.safetymonitor import SafetyMonitor
from matplotlib.animation import FuncAnimation

# Initialize devices
WS = ObservingConditions("192.168.1.12", 0)
SM = SafetyMonitor("192.168.1.12", 0)

# Set up the plot
fig, axs = plt.subplots(10, 1, figsize=(5, 10), sharex=True)
# plt.tight_layout(pad=0.2)

# Define data keys (excluding timestamp)
data_keys = [
    "dewpoint",
    "humidity",
    "temperature",
    "windspeed",
    "windgust",
    "winddirection",
    "pressure",
    "rainrate",
    "skybrightness",
    "safe",
]

# Initialize data storage
data = {key: [] for key in ["timestamp"] + data_keys}

# Set up CSV writer
csv_file = open("data.csv", mode="w", newline="")
csv_writer = csv.DictWriter(csv_file, fieldnames=data.keys())
csv_writer.writeheader()


def update(frame):
    current_time = datetime.now()

    # Collect data
    new_data = {
        "timestamp": current_time,
        "dewpoint": WS.DewPoint,
        "humidity": WS.Humidity,
        "temperature": WS.Temperature,
        "windspeed": WS.WindSpeed,
        "windgust": WS.WindGust,
        "winddirection": WS.WindDirection,
        "pressure": WS.Pressure,
        "rainrate": WS.RainRate,
        "skybrightness": WS.SkyBrightness,
        "safe": int(SM.IsSafe),  # Convert boolean to int for plotting
    }

    # Update data storage
    for key, value in new_data.items():
        data[key].append(value)

    # Write to CSV
    csv_writer.writerow(new_data)

    # Update plots
    for i, (key, ax) in enumerate(zip(data_keys, axs)):
        ax.clear()
        ax.plot(data["timestamp"], data[key], "b-")
        ax.set_title(key)
        ax.set_xlim(
            left=max(current_time - timedelta(minutes=5), data["timestamp"][0]),
            right=current_time,
        )

        # Format y-axis for boolean data
        if key == "safe":
            ax.set_yticks([0, 1])
            ax.set_yticklabels(["False", "True"])

    # Remove old data points (keep last 12000 points)
    if len(data["timestamp"]) > 12000:
        for key in data:
            data[key] = data[key][-12000:]


# Set up the animation
ani = FuncAnimation(fig, update, interval=1000, cache_frame_data=False)

plt.show()

# Close CSV file when done
csv_file.close()
