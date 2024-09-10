from alpaca.observingconditions import *

WS = ObservingConditions('192.168.0.12', 0)

# WS.Connected = True

for i in range(10):
    temperture = WS.Temperature

    print(temperture)

    # time.sleep(1)