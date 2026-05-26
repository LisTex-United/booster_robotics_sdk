import time

from booster_robotics_sdk_python import (
    B1LocoClient,
    ChannelFactory,
    # B1LowCmdPublisher,
    # B1LowStateSubscriber,
    # GetModeResponse,
    GripperMotionParameter,
    GripperControlMode,
    B1HandIndex,
    # LowCmd,
    # MotorCmd,
    RobotMode,
)

CONTROL_FREQ = 50.0

ChannelFactory.Instance().Init(0, "127.0.0.1")

client = B1LocoClient()
client.Init()

ret = client.SwitchHandEndEffectorControlMode(True)
print(f"SwitchHandEndEffectorControlMode(True) returned: {ret}")
time.sleep(0.1)

# # Open to max (~77mm): position=1000
# open_param = GripperMotionParameter(position=1000, force=200, speed=300)
# ret = client.ControlGripper(open_param, GripperControlMode.kPosition, B1HandIndex.kRightHand)

# Close slowly with low force to reduce risk
close_param = GripperMotionParameter(position=0, force=100, speed=50)
ret = 100
for attempt in range(5):
    ret = client.ControlGripper(close_param, GripperControlMode.kPosition, B1HandIndex.kRightHand)
    print(f"ControlGripper returned: {ret}")
    if ret == 0:
        break
    time.sleep(0.2)

if ret == 502:
    print("Note: 502 means state transition failed on robot side.")

if ret not in (0, 100):
    raise SystemExit(f"Abort: ControlGripper failed with {ret}")

# Best-effort: disable end-effector control mode when done.
ret2 = client.SwitchHandEndEffectorControlMode(False)
print(f"SwitchHandEndEffectorControlMode(False) returned: {ret2}")