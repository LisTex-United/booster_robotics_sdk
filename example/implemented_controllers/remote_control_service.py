from typing import Optional
import evdev
import threading
from dataclasses import dataclass
import time
import termios
import tty
import select
import atexit
import sys


@dataclass
class JoystickConfig:
    max_vx: float = 0.35
    max_vy: float = 0.35
    max_vyaw: float = 0.5
    control_threshold: float = 0.1
    # logitech
    back_button: evdev.ecodes = evdev.ecodes.BTN_TL2
    stop_button: evdev.ecodes = evdev.ecodes.BTN_TR2
    walk_button: evdev.ecodes = evdev.ecodes.BTN_B
    x_axis: evdev.ecodes = evdev.ecodes.ABS_Y
    y_axis: evdev.ecodes = evdev.ecodes.ABS_X
    yaw_axis: evdev.ecodes = evdev.ecodes.ABS_Z
    

    # xiaoji
    # custom_mode_button: evdev.ecodes = evdev.ecodes.BTN_B
    # rl_gait_button: evdev.ecodes = evdev.ecodes.BTN_A
    # x_axis: evdev.ecodes = evdev.ecodes.ABS_Y
    # y_axis: evdev.ecodes = evdev.ecodes.ABS_X
    # yaw_axis: evdev.ecodes = evdev.ecodes.ABS_RX


class RemoteControlService:
    """Service for handling joystick remote control input without display dependencies."""

    def __init__(self, config: Optional[JoystickConfig] = None):
        """Initialize remote control service with optional configuration."""
        self.config = config or JoystickConfig()
        self._lock = threading.Lock()
        self._running = True
        self._joystick_controlling = False
        self.back_button_pressed = False
        self.already_sent_stop = True

        try:
            self._init_joystick()
            self._start_joystick_thread()
        except Exception as e:
            print(f"{e}, the joystick didn't started")
            self._running = False

        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0

    def get_operation_hint(self) -> str:
        if hasattr(self, "joystick") and getattr(self, "joystick") is not None:
            return "Press Start to activate Joystick. Joystick left axis for forward/backward/left/right, right axis for rotation left/right"
        return "Press keyboard 'w'/'s' to increase/decrease vx; Press 'a'/'d' to increase/decrease vy; Press 'q'/'e' to increase/decrease vyaw, press 'Space' to stop."

    def get_walk_operation_hint(self) -> str:
        if hasattr(self, "joystick") and getattr(self, "joystick") is not None:
            return "Press joystick button A to start walk."
        return "Press keyboard 'r' to start walk."

    def _init_joystick(self) -> None:
        """Initialize and validate joystick connection using evdev."""
        try:
            devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
            joystick = None

            for device in devices:
                caps = device.capabilities()
                # print(f"Device {device.name}:")
                # print(f"Capabilities: {device.capabilities(verbose=True)}")

                # Check for both absolute axes and keys
                if evdev.ecodes.EV_ABS in caps and evdev.ecodes.EV_KEY in caps:
                    abs_info = caps.get(evdev.ecodes.EV_ABS, [])
                    # Look for typical gamepad axes
                    axes = [code for (code, info) in abs_info]
                    if all(code in axes for code in [self.config.x_axis, self.config.y_axis, self.config.yaw_axis]):
                        absinfo = {}
                        for code, info in abs_info:
                            absinfo[code] = info
                        self.axis_ranges = {
                            self.config.x_axis: absinfo[self.config.x_axis],
                            self.config.y_axis: absinfo[self.config.y_axis],
                            self.config.yaw_axis: absinfo[self.config.yaw_axis],
                        }
                        print(f"Found suitable joystick: {device.name}")
                        joystick = device
                        break

            if not joystick:
                raise RuntimeError("No suitable joystick found")

            self.joystick = joystick
            print(f"Selected joystick: {joystick.name}")
        except Exception as e:
            raise RuntimeError(f"Failed to initialize joystick: {e}")

    def _start_joystick_thread(self):
        """Start joystick polling thread."""
        self.joystick_runner = threading.Thread(target=self._run_joystick)
        self.joystick_runner.daemon = True
        self.joystick_runner.start()

    def start_walk(self) -> bool:
        """Check if walk button is pressed."""
        if hasattr(self, "joystick") and getattr(self, "joystick") is not None:
            return self.joystick.active_keys() == [self.config.walk_button]
        return self.keyboard_start_walk
    
    def _run_joystick(self):
        """Poll joystick events."""
        while self._running:
            try:
                # read one event
                event = self.joystick.read_one()
                if event:
                    if event.type == evdev.ecodes.EV_KEY:
                        self._handle_button(event.code, event.value)
                        
                    # elif event.type == evdev.ecodes.EV_ABS and self.is_joystick_controlling():
                    #     self._handle_axis(event.code, event.value)
                        
                else:
                    time.sleep(0.01)
            except Exception as e:
                if not self._running:  # If the exception was caused by shutdown, no need to log
                    break
                print(f"Error in joystick polling loop: {e}")
                time.sleep(0.05)

    def _handle_axis(self, code: int, value: int):
        try:
            """Handle axis events."""
            if code == self.config.x_axis:
                self.vx = self._scale(value, self.config.max_vx, self.config.control_threshold, code)
                # print("value x:", self.vx)
            elif code == self.config.y_axis:
                self.vy = self._scale(value, self.config.max_vy, self.config.control_threshold, code)
                # print("value y:", self.vy)
            elif code == self.config.yaw_axis:
                self.vyaw = self._scale(value, self.config.max_vyaw, self.config.control_threshold, code)
                # print("value yaw:", self.vyaw)
        except Exception:
            raise

    def _handle_button(self, code: int, value: int):
        """Handle button events."""
        if code == self.config.stop_button and value == 1:
            with self._lock:
                self.vx = 0.0
                self.vy = 0.0
                self.vyaw = 0.0
                self._joystick_controlling = not self._joystick_controlling
                self.already_sent_stop = False
            
            if self._joystick_controlling:
                print("Emergency Stop: velocities set to zero. Joystick Control Enabled.")
            else:
                print("Joystick Control Disabled.") 
        
        elif code == self.config.back_button and value == 1:
            with self._lock:
                self.back_button_pressed = True

    def is_joystick_controlling(self) -> bool:
        """Check if joystick is currently controlling the robot."""
        with self._lock:
            return self._joystick_controlling
    
    def send_stop(self) -> bool:
        """Check if it has already sent a stop command after pressing the start button."""
        with self._lock:
            if self._joystick_controlling:
                if not self.already_sent_stop:
                    self.already_sent_stop = True
                    return True
                
        return False
    
    def send_back(self) -> bool:
        """Check if the back button has been pressed."""
        with self._lock:
            if self.back_button_pressed:
                self.back_button_pressed = False
                return True
        return False
                    
    def _scale(self, value: float, max: float, threshold: float, axis_code: int) -> float:
        """Scale joystick input to velocity command using actual axis ranges."""
        absinfo = self.axis_ranges[axis_code]
        min_in = absinfo.min
        max_in = absinfo.max

        mapped_value = ((value - min_in) / (max_in - min_in) * 2 - 1) * max

        if abs(mapped_value) < threshold:
            return 0.0
        return -mapped_value

    def get_vx_cmd(self) -> float:
        """Get forward velocity command."""
        with self._lock:
            return self.vx

    def get_vy_cmd(self) -> float:
        """Get lateral velocity command."""
        with self._lock:
            return self.vy

    def get_vyaw_cmd(self) -> float:
        """Get yaw velocity command."""
        with self._lock:
            return self.vyaw

    def close(self):
        """Clean up resources."""
        self._running = False
        # try restore stdin terminal settings if we changed them
        try:
            if getattr(self, "_stdin_tty", False) and getattr(self, "_old_termios", None) is not None:
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._old_termios)
        except Exception:
            pass
        if hasattr(self, "joystick") and getattr(self, "joystick") is not None:
            try:
                self.joystick.close()
            except Exception as e:
                print(f"Error closing joystick: {e}")
        if hasattr(self, "joystick_runner") and getattr(self, "joystick_runner") is not None:
            try:
                self.joystick_runner.join(timeout=1.0)
                if self.joystick_runner.is_alive():
                    print("Joystick thread didn't exit within the time limit")
            except Exception as e:
                print(f"Error waiting for joystick thread to end: {e}")
                
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
