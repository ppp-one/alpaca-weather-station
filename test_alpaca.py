from datetime import datetime

from alpaca.observingconditions import *

WS = ObservingConditions("192.168.0.12", 0)

# WS.Connected = True
counter = 0
for i in range(10):
    temperture = WS.Temperature
    humidity = WS.Humidity
    counter += 1

    print(datetime.now(), temperture, humidity, counter)

    # time.sleep(1)
