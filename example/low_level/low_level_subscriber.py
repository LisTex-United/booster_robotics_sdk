from booster_robotics_sdk_python import ChannelFactory, B1LowStateSubscriber
import time


def imu_only_handler(low_state_msg):
    imu = low_state_msg.imu_state
    rpy, gyro, acc = imu.rpy, imu.gyro, imu.acc
    label_w = 6
    col = 14

    def triple(prefix, x0, x1, x2):
        nums = " ".join(f"{x:>{col}.5f}" for x in (x0, x1, x2))
        return f"  {prefix:<{label_w}}{nums}"

    hdr = f"  {'':{label_w}}" + " ".join(f"{name:>{col}}" for name in ("roll", "pitch", "yaw"))
    print("imu")
    print(hdr)
    print(triple("rpy", rpy[0], rpy[1], rpy[2]))
    print(triple("gyro", gyro[0], gyro[1], gyro[2]))
    print(triple("acc", acc[0], acc[1], acc[2]))


def handler(low_state_msg):
    print("Received message:")
    print(f"  serial motor count: {len(low_state_msg.motor_state_serial)}")
    print(f"  parallel motor count: {len(low_state_msg.motor_state_parallel)}")
    imu_state = low_state_msg.imu_state
    print(f"  imu: {imu_state.rpy[0]}, {imu_state.rpy[1]}, {imu_state.rpy[2]}, "
          f"{imu_state.gyro[0]}, {imu_state.gyro[1]}, {imu_state.gyro[2]}, "
          f"{imu_state.acc[0]}, {imu_state.acc[1]}, {imu_state.acc[2]}")
    for i, motor in enumerate(low_state_msg.motor_state_serial):
        print(f"  serial motor {i}: {motor.q}, {motor.dq}, {motor.ddq}, {motor.tau_est}")
    for i, motor in enumerate(low_state_msg.motor_state_parallel):
        print(
            f"  parallel motor {i}: {motor.dq}, {motor.ddq}, {motor.tau_est}")
    print("done")


def main():
    ChannelFactory.Instance().Init(0)
    channel_subscriber = B1LowStateSubscriber(handler)
    channel_subscriber.InitChannel()
    print("listening (imu only)")
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
