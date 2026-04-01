#!/usr/bin/env python3
"""Read current head joint values from the robot."""
import sys
import time
from booster_robotics_sdk_python import ChannelFactory, B1LowStateSubscriber

# Head joint indices
HEAD_YAW_INDEX = 0
HEAD_PITCH_INDEX = 1

received = False

def handler(low_state_msg):
    global received
    if received:
        return
    received = True
    
    serial_motors = low_state_msg.motor_state_serial
    print(f"\nSerial motor count: {len(serial_motors)}")
    
    if len(serial_motors) > HEAD_PITCH_INDEX:
        head_yaw = serial_motors[HEAD_YAW_INDEX]
        head_pitch = serial_motors[HEAD_PITCH_INDEX]
        print(f"\n=== HEAD JOINT VALUES ===")
        print(f"Head Yaw   (index {HEAD_YAW_INDEX}):   q = {head_yaw.q:.4f} rad ({head_yaw.q * 57.2958:.2f} deg)")
        print(f"Head Pitch (index {HEAD_PITCH_INDEX}):   q = {head_pitch.q:.4f} rad ({head_pitch.q * 57.2958:.2f} deg)")
    else:
        print("Head joints not found in serial motors!")
    
    # Also print waist if available (index 10 in standard, but may vary)
    parallel_motors = low_state_msg.motor_state_parallel
    print(f"\nParallel motor count: {len(parallel_motors)}")

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <network_interface>")
        print("Example: python3 read_head_state.py eth0")
        sys.exit(1)
    
    network_interface = sys.argv[1]
    print(f"Connecting via {network_interface}...")
    
    ChannelFactory.Instance().Init(0, network_interface)
    channel_subscriber = B1LowStateSubscriber(handler)
    channel_subscriber.InitChannel()
    print("Waiting for robot state message...")
    
    # Wait for one message then exit
    timeout = 5.0
    start = time.time()
    while not received and (time.time() - start) < timeout:
        time.sleep(0.1)
    
    if not received:
        print("Timeout: No robot state received. Is the robot connected?")
        sys.exit(1)

if __name__ == "__main__":
    main()
