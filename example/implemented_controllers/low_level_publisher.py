import time
from booster_robotics_sdk_python import ChannelFactory, B1LowCmdPublisher, LowCmd, LowCmdType, MotorCmd, B1JointCnt, B1JointIndex
from joints_const import JOINT_PARAMETERS, TOTAL_DOF
SLEEP_TIME = 0.2


def main():
    ChannelFactory.Instance().Init(0)
    channel_publisher = B1LowCmdPublisher()
    channel_publisher.InitChannel()
    print(f"The number of joints is: {B1JointCnt} and the total DOF is: {TOTAL_DOF}")
    motor_cmds = [MotorCmd() for _ in range(TOTAL_DOF)]
    x = 0
    while x < 10:
        x+=1
        low_cmd = LowCmd()
        low_cmd.cmd_type = LowCmdType.PARALLEL
        low_cmd.motor_cmd = motor_cmds
        for i in range(B1JointCnt):
            low_cmd.motor_cmd[i].q = 0.0
            low_cmd.motor_cmd[i].dq = 0.0
            low_cmd.motor_cmd[i].tau = 0.0
            low_cmd.motor_cmd[i].kp = 0.0
            low_cmd.motor_cmd[i].kd = 0.0
            low_cmd.motor_cmd[i].weight = 0.0
            if i == B1JointIndex.kLeftElbowYaw.value:
                print(i)
                low_cmd.motor_cmd[i].q = -1.3
                low_cmd.motor_cmd[i].dq = 0.0
                low_cmd.motor_cmd[i].tau = 0.0
                low_cmd.motor_cmd[i].kp = 10.0
                low_cmd.motor_cmd[i].kd = 2.5
                low_cmd.motor_cmd[i].weight = 1.0

        channel_publisher.Write(low_cmd)
        time.sleep(SLEEP_TIME)


if __name__ == "__main__":
    main()
