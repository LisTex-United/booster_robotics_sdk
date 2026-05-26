import time
import argparse

from booster_robotics_sdk_python import (
    B1LocoClient, 
    ChannelFactory, 
    RobotMode,
    B1LowCmdPublisher,
    LowCmd,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--network_interface",
        type=str,
        required=True,
        help="Network interface (e.g., eth0, enp0s1)",
    )
    args = parser.parse_args()
    
    print("Initializing Booster SDK...")
    
    ChannelFactory.Instance().Init(0, args.network_interface)
    
    publisher = B1LowCmdPublisher()
    msg = LowCmd()
    
    # Create and initialize client
    client = B1LocoClient()
    client.Init()
    
    # Initialize publisher channel AFTER creating LowCmd
    publisher.InitChannel()
    
    print("SDK initialized!")
    print("\nChanging to CUSTOM mode...")
    
    # Change to custom mode
    ret = client.ChangeMode(RobotMode.kCustom)
    print(f"Mode change returned: {ret}")
    
    # Give time for mode to switch
    time.sleep(1.0)

if __name__ == "__main__":
    main()
