from alpaca.observingconditions import *
from datetime import datetime

WS = ObservingConditions('192.168.0.12', 0)

# WS.Connected = True
counter = 0
for i in range(1000):
    temperture = WS.Temperature
    counter += 1

    print(datetime.now(), temperture, counter)

    # time.sleep(1)