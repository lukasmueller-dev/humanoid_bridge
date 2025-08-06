#!/usr/bin/env python3
"""
Test script to verify terminal restoration after RemoteControlService usage.
This script tests the keyboard input handling and ensures proper cleanup.
"""

import time
import signal
import sys
from utils.remote_control_service import RemoteControlService

def main():
    print("Testing RemoteControlService terminal restoration...")
    print("Use WASD for movement, Q/E for yaw, Space to stop, Ctrl+C to exit")
    
    # Create remote control service
    remote_control = None
    try:
        remote_control = RemoteControlService()
        print(f"Control instructions: {remote_control.get_operation_hint()}")
        
        # Test for 30 seconds or until interrupted
        start_time = time.time()
        while time.time() - start_time < 30:
            vx = remote_control.get_vx_cmd()
            vy = remote_control.get_vy_cmd()
            vyaw = remote_control.get_vyaw_cmd()
            
            if vx != 0 or vy != 0 or vyaw != 0:
                print(f"Commands: vx={vx:.2f}, vy={vy:.2f}, vyaw={vyaw:.2f}")
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if remote_control:
            print("Cleaning up remote control service...")
            remote_control.close()
        print("Test completed. Terminal should be restored.")

if __name__ == "__main__":
    main()
