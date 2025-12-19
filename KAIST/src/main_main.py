#!/usr/bin/env python3
# main_robot.py
# Integrated control: Robot ↔ Vision (Modbus)
from scipy.spatial.transform import Rotation as Rot

from vision_processor.vision_processor_abstract import VisionProcessorAbstract
from modbus_robot_interface import ModbusRobotInterface
from vision_processor.vision_processor_gun import VisionProcessorGun
from vision_processor.vision_processor import VisionProcessor
from main_main_refactored import RSFramesUtils
# from april_gun_function import get_aruco_gun_pose_for_command
import time
import cv2

def process_command(cmd: int, modbus: ModbusRobotInterface, process: VisionProcessorAbstract, frame_num: int, rs_utils: RSFramesUtils):
    """Process a single command from the robot.
    
    Args:
        cmd: Command number (1-4)
        modbus: ModbusRobotInterface instance
    """
    print(f"\n📥 Received command {cmd}, running vision...")

    # Run vision processing
    try:
        if cmd in [0, 1, 2, 3, 4]:
            color_image, intrinsics = rs_utils.get_frames()
            marker_poses = modbus.get_robot_transform_matrices()
            pose_main = process.process_frame(color_image, intrinsics, marker_poses)
            print("Before pose main", pose_main)
            if pose_main is not None and frame_num > 250:
                x, y, z = pose_main["position_object_to_world"].squeeze()
                
                offset = 0.2
                offset_objective = pose_main["position_object_to_world"].copy().squeeze()
                offset_objective -=  offset*pose_main["rotation_ee_obj_to_world"].copy()[:, 2]
                x_back, y_back, z_back = offset_objective
                
                rx, ry, rz = Rot.from_matrix(pose_main["rotation_ee_obj_to_world"]).as_euler("xyz", degrees=True)
                print("pose_main", x,y,z,rx,ry,rz)
                print("pose_back", x_back,y_back,z_back)
                print("object to camera", pose_main['position_object_to_camera'])
                print("rotation_object to camera", pose_main['rotation_object_to_camera'])
                print("camera_position", pose_main['board_position_in_camera'].squeeze())
                print("camera_rotation", pose_main['board_rotation_in_camera'].squeeze())
                pose_main = modbus.pose_to_modbus_data(x,y,z,rx,ry,rz)
                pose_back = modbus.pose_to_modbus_data(x_back,y_back,z_back,rx,ry,rz)
        else:
            print(f"❌ Unknown cmd={cmd}")
            modbus.write_response(0)
            modbus.reset_command()
            return
    except Exception as e:
        print(f"❌ Vision exception: {e}")
        modbus.write_response(0)
        modbus.reset_command()
        return

    # Check if vision succeeded
    if pose_main is None:
        print("❌ Vision failed")
        modbus.write_response(0)
        modbus.reset_command()
        return
    
    # Write pose data
    if frame_num > 250:
        if not modbus.write_pose(pose_back, pose_back):
            print("❌ Failed to write pose")
            modbus.reset_command()
            return

    # Send response based on command type
    if cmd in [1] and frame_num > 250:
        modbus.write_response(1)  # Use registers 301~306
    elif cmd in [2, 4]:
        modbus.write_response(2)  # Use registers 307~312

    # modbus.reset_command()


def main_loop():
    """Main control loop that monitors robot commands and triggers vision processing."""
    
    rs_utils = RSFramesUtils()
    
    # vp = VisionProcessor("./best.pt", "./point_cloud/detected_circles.json")
    vp_gun = VisionProcessorGun(rs_utils.intrinsics)
    
    # Use context manager for automatic cleanup
    with ModbusRobotInterface(ip="192.168.0.29", port=1502, timeout=0.1) as modbus:
        frame_num = 0
        if not modbus.is_connected():
            print("❌ Failed to connect to robot - running in simulation mode")
            print("   Vision will run but Modbus operations will be skipped")
        else:
            print("✅ Main loop started (listening to register 351)")
        
        try:
            while True:
                frame_num += 1
                # Read command from robot
                cmd = modbus.read_command()
                print("cmd: ", cmd)
                
                if cmd is None:
                    print("⚠️  No command received, retrying...")
                    time.sleep(1.0)
                    continue
                
                # if cmd == 0:
                #     print("⏳ Waiting... (CMD=0)")
                #     time.sleep(1.0)
                #     continue
                
                # Process non-zero command
                # if cmd in [0,1,2,3,4]:
                #     process_command(cmd, modbus, vp)
                # elif cmd in [0,1,2,3]:
                print("frame_num", frame_num)
                process_command(cmd, modbus, vp_gun, frame_num, rs_utils)
                
        except KeyboardInterrupt:
            print("\n⚠️  Keyboard interrupt detected")
        except Exception as e:
            print(f"❌ Main loop error: {e}")
        finally:
            print("🛑 Main loop terminated")


if __name__ == "__main__":
    main_loop()