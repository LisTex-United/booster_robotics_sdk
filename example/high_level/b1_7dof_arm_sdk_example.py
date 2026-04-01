from booster_robotics_sdk_python import (
    B1LocoClient, 
    ChannelFactory, 
    RobotMode, 
    B1LowCmdPublisher,
    LowCmd,
    MotorCmd
)
import sys
import time

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} networkInterface")
        sys.exit(-1)

    ChannelFactory.Instance().Init(0, sys.argv[1])

    arm_sdk_publisher = B1LowCmdPublisher()
    msg = LowCmd()

    client = B1LocoClient()
    client.Init()

    arm_sdk_publisher.InitChannel()

    # Define arm joints (14 joints - 7 DOF for each arm)
    # JointIndexWith7DofArm enum values
    arm_joints = [
        2,   # kLeftShoulderPitch
        3,   # kLeftShoulderRoll
        4,   # kLeftElbowPitch
        5,   # kLeftElbowYaw
        6,   # kLeftWristPitch
        7,   # kLeftWristYaw
        8,   # kLeftHandRoll
        9,   # kRightShoulderPitch
        10,  # kRightShoulderRoll
        11,  # kRightElbowPitch
        12,  # kRightElbowYaw
        13,  # kRightWristPitch
        14,  # kRightWristYaw
        15   # kRightHandRoll
    ]

    # init control params
    weight = 0.0
    weight_rate = 0.2
    
    kp = 55.0
    kd = 1.5
    dq = 0.0
    tau_ff = 0.0
    
    control_dt = 0.02
    max_joint_velocity = 0.5
    
    weight_margin = weight_rate * control_dt
    max_joint_delta = max_joint_velocity * control_dt
    sleep_time = control_dt
    
    # Joint position array (14 joints: 7 DOF per arm)
    # Values are in radians. Mapping to arm_joints array:
    # Index 0:  kLeftShoulderPitch  (Left arm shoulder pitch)
    # Index 1:  kLeftShoulderRoll   (Left arm shoulder roll)
    # Index 2:  kLeftElbowPitch     (Left arm elbow pitch)
    # Index 3:  kLeftElbowYaw       (Left arm elbow yaw)
    # Index 4:  kLeftWristPitch     (Left arm wrist pitch)
    # Index 5:  kLeftWristYaw       (Left arm wrist yaw)
    # Index 6:  kLeftHandRoll       (Left arm hand roll)
    # Index 7:  kRightShoulderPitch (Right arm shoulder pitch)
    # Index 8:  kRightShoulderRoll  (Right arm shoulder roll)
    # Index 9:  kRightElbowPitch    (Right arm elbow pitch)
    # Index 10: kRightElbowYaw      (Right arm elbow yaw)
    # Index 11: kRightWristPitch    (Right arm wrist pitch)
    # Index 12: kRightWristYaw      (Right arm wrist yaw)
    # Index 13: kRightHandRoll      (Right arm hand roll)
    init_pos = [0.5, -1.0, 0.0, -1.4, 0.0, 0.0, 0.0,
                0.5,  1.0, 0.0,  1.4, 0.0, 0.0, 0.0]
    
    target_pos = [0.0, 0.0, 0.0, 0.0, 0, 0, 0,
                  0.0, 0.0, 0.0, 0.0, 0, 0, 0]
    
    # wait for init
    input("Press ENTER to init arms ...")
    
    ret = client.ChangeMode(RobotMode.kCustom)
    
    # set init pos
    print("Initailizing arms ...")
    init_time = 5.0
    init_time_steps = int(init_time / control_dt)
    
    # Initialize motor_cmd vector with 29 motors (kJointCnt7DofArm)
    motor_cmd_list = []
    for i in range(29):
        motor_cmd = MotorCmd()
        motor_cmd_list.append(motor_cmd)
    msg.motor_cmd = motor_cmd_list
    
    # init joints
    for i in range(init_time_steps):
        # increase weight
        weight += weight_margin
        weight = max(0.0, min(weight, 0.5))
        print(weight)
        
        # set control joints
        for j in range(len(init_pos)):
            motor_cmd_list[arm_joints[j]].q = init_pos[j]
            motor_cmd_list[arm_joints[j]].dq = dq
            motor_cmd_list[arm_joints[j]].kp = kp
            motor_cmd_list[arm_joints[j]].kd = kd
            motor_cmd_list[arm_joints[j]].tau = tau_ff
            motor_cmd_list[arm_joints[j]].weight = weight
        
        # Update the message
        msg.motor_cmd = motor_cmd_list
        
        # send dds msg
        arm_sdk_publisher.Write(msg)
        
        # sleep
        time.sleep(sleep_time)
    
    print("Done!")
    
    # wait for control
    input("Press ENTER to start arm ctrl ...")
    
    # start control
    print("Start arm ctrl!")
    period = 10.0
    num_time_steps = int(period / control_dt)
    
    current_jpos_des = init_pos.copy()
    
    # lift arms up
    for i in range(num_time_steps):
        # update jpos des
        for j in range(len(init_pos)):
            delta = target_pos[j] - current_jpos_des[j]
            clamped_delta = max(-max_joint_delta, min(delta, max_joint_delta))
            current_jpos_des[j] += clamped_delta
            
            print(f"Target joint position: {target_pos[j]}")
            print(f"Current joint position: {current_jpos_des[j]}")
        print("----")
        
        # set control joints
        for j in range(len(init_pos)):
            motor_cmd_list[arm_joints[j]].q = current_jpos_des[j]
            motor_cmd_list[arm_joints[j]].dq = dq
            motor_cmd_list[arm_joints[j]].kp = kp
            motor_cmd_list[arm_joints[j]].kd = kd
            motor_cmd_list[arm_joints[j]].tau = tau_ff
        
        # Update the message
        msg.motor_cmd = motor_cmd_list
        
        # send dds msg
        arm_sdk_publisher.Write(msg)
        
        # sleep
        time.sleep(sleep_time)
    
    print("Done!")

if __name__ == "__main__":
    main()
