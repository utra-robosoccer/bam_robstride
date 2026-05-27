import can
import robstride

# After setting up can0 via slcand or similar
with can.Bus(interface='socketcan', channel='can0', bitrate=1000000) as bus:
    client = robstride.Client(bus)
    client.enable(1)  # default motor ID