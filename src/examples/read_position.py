"""Read motor position by enabling the motor to trigger feedback."""

import time
from actuator import CH341TransportWrapper, RobstrideActuator, RobstrideActuatorConfig, RobstrideConfigureRequest

MOTOR_ID = 1
PORT = "/dev/ttyCH341USB0"

transport = CH341TransportWrapper(PORT)
supervisor = RobstrideActuator(
    transports=[transport],
    py_actuators_config=[(MOTOR_ID, RobstrideActuatorConfig(0))],  # 0 = RobStride00
)

# Enable the motor — this sends an Enable CAN frame; motor responds with a feedback frame
cfg = RobstrideConfigureRequest(MOTOR_ID)
cfg.torque_enabled = True
cfg.kp = 0.0  # zero gains so it doesn't move
cfg.kd = 0.5
supervisor.configure_actuator(cfg)

# Wait for the feedback frame to be received by the background task
time.sleep(0.3)

while True:
    state = supervisor.get_actuators_state([MOTOR_ID])
    if state:
        s = state[0]
        print(f"ID={s.actuator_id}  pos={s.position:.3f}°  vel={s.velocity:.3f}°/s  torque={s.torque:.3f}Nm  online={s.online}")
    else:
        # Re-send Enable to get another feedback frame
        supervisor.configure_actuator(cfg)
        time.sleep(0.1)
    time.sleep(0.2)
