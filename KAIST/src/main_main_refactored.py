#!/usr/bin/env python3
# main_robot.py
# Modular integrated control: Robot ↔ Vision (Modbus)
import sys, os
sys.path.append(os.path.dirname(__file__))

from typing import Dict, Optional, Tuple
from scipy.spatial.transform import Rotation as Rot
import time
import cv2
import pyrealsense2 as rs
import numpy as np
import hydra 

from vision_processor.vision_processor_abstract import VisionProcessorAbstract
from modbus_robot_interface import ModbusRobotInterface
from vision_processor.vision_processor_port import VisionProcessorPort
from vision_processor.vision_processor_gun import VisionProcessorGun
from vision_processor.vision_processor import VisionProcessor


class RSFramesUtils:
    
    def __init__(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 15)
        profile = self.pipeline.start(config)

        self.align = rs.align(rs.stream.color)
        self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.frame_num = 0
        
    def get_frames(self):
        """Capture and align frames from RealSense camera."""
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()

        if not color_frame:
            return None, None
        
        color_image = np.asanyarray(color_frame.get_data())
        
        # Get and adjust intrinsics for rotation
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        self.frame_num += 1
        return color_image, intrinsics

class CommandProcessor:
    """Manages command routing and vision processor selection."""
    
    def __init__(self, modbus: ModbusRobotInterface):
        self.modbus = modbus
        self.processors: Dict[str, VisionProcessorAbstract] = {}
        self.command_map: Dict[int, dict] = {}
        
    def register_processor(self, name: str, processor: VisionProcessorAbstract):
        """Register a vision processor."""
        self.processors[name] = processor
        print(f"✅ Registered processor: {name}")
        
    def register_command(self, cmd: int, processor_name: str, 
                        response_type: int, use_offset: bool = False):
        """Register a command with its processor and parameters.
        
        Args:
            cmd: Command number
            processor_name: Name of the processor to use
            response_type: Response type to send (1, 2, etc.)
            use_offset: Whether to apply offset calculation
        """
        if processor_name not in self.processors:
            raise ValueError(f"Processor '{processor_name}' not registered")
            
        self.command_map[cmd] = {
            'processor': processor_name,
            'response_type': response_type,
            'use_offset': use_offset
        }
        print(f"✅ Registered command {cmd} → {processor_name}")
        
    def process_command(self, cmd: int, frame_util: RSFramesUtils) -> bool:
        """Process a command using the appropriate vision processor.
        
        Args:
            cmd: Command number
            frame_num: Current frame number
            
        Returns:
            bool: True if successful, False otherwise
        """
        # Check if command is registered
        if cmd not in self.command_map:
            print(f"❌ Unknown command: {cmd}")
            self.modbus.write_response(0)
            self.modbus.reset_command()
            return False
            
        config = self.command_map[cmd]
        processor = self.processors[config['processor']]
        
        print(f"\n📥 Command {cmd} → Processor '{config['processor']}'")
        
        # Run vision processing
        try:
            # When starting vision process, write 0 on address 352
            self.modbus.write_response(0)
            pose_result = self._run_vision(processor, frame_util)
            
            if pose_result is None:
                print("❌ Vision failed")
                self.modbus.write_response(0)
                self.modbus.reset_command()
                return False
                
        except Exception as e:
            print(f"❌ Vision exception: {e}")
            self.modbus.write_response(0)
            self.modbus.reset_command()
            return False
        
        # Calculate poses
        pose_main, pose_back = self._calculate_poses(
            pose_result, 
            use_offset=config['use_offset']
        )
        
        # Write pose data (after warmup frames)
        # If confidence_score converges -> write pose in address 301 ~ 306(pose_back) - 352(1), 307 ~ 312(pose_main), 352(2)
        # If is_converged: response -> 1
        if pose_result["is_converged"]:
            if not self.modbus.write_pose(pose_back, pose_back):
                print("❌ Failed to write pose")
                self.modbus.reset_command()
                return False
                
            # Send response
            self.modbus.write_response(config['response_type'])
            print(f"Write {config['response_type']} in address 352")
            
        return True
    
    def _run_vision(self, processor: VisionProcessorAbstract, 
                   frame_utils: RSFramesUtils) -> Optional[dict]:
        """Run vision processing pipeline."""
        color_image, intrinsics = frame_utils.get_frames()
        marker_poses = self.modbus.get_robot_transform_matrices()
        if frame_utils.frame_num > 25:
            pose_main = processor.process_frame(color_image, intrinsics, marker_poses)
        
        if pose_main is not None:
            self._print_pose_info(pose_main)
            
        return pose_main
    
    def _calculate_poses(self, pose_result: dict, 
                        use_offset: bool) -> Tuple[list, list]:
        """Calculate main and offset poses.
        
        Returns:
            Tuple of (pose_main, pose_back) in Modbus format
        """
        x, y, z = pose_result["position_object_to_world"].squeeze()
        rot = pose_result["rotation_ee_obj_to_world"].copy()
        # rot[:, 2] = -rot[:, 2]
        # rot[:, 1] = -rot[:, 1]
        rx, ry, rz = Rot.from_matrix(
            rot#pose_result["rotation_ee_obj_to_world"]
        ).as_euler("xyz", degrees=True)
        
        pose_main = self.modbus.pose_to_modbus_data(x, y, z, rx, ry, rz)
        
        if use_offset:
            # offset = np.array([0, 0, 0.2])
            # TODO: Add offset back
            offset = 0.2 # np.array([0, 0, 0.2])
            offset_objective = pose_result["position_object_to_world"].copy().squeeze()
            # normal_vec = pose_result["rotation_ee_obj_to_world"].copy()[:,2]
            offset_rotation = pose_result["rotation_object_to_world"].copy()[:, 2]
            # offset_rotation[0] = -offset_rotation[0]
            offset_values = offset * offset_rotation
            offset_objective -= offset_values
            
            # offset_objective += pose_result["rotation_ee_obj_to_world"].copy() @ offset 
            x_back, y_back, z_back = offset_objective
            pose_back = self.modbus.pose_to_modbus_data(x_back, y_back, z_back, rx, ry, rz)
            print(f"📍 Offset pose: ({x_back:.3f}, {y_back:.3f}, {z_back:.3f})")
        else:
            pose_back = pose_main
            
        return pose_main, pose_back
    
    def _print_pose_info(self, pose_result: dict):
        """Print detailed pose information."""
        x, y, z = pose_result["position_object_to_world"].squeeze()
        rx, ry, rz = Rot.from_matrix(
            pose_result["rotation_ee_obj_to_world"]
        ).as_euler("xyz", degrees=True)
        
        print(f"📍 Main pose: ({x:.3f}, {y:.3f}, {z:.3f}) | ({rx:.1f}°, {ry:.1f}°, {rz:.1f}°)")
        print(f"   Object→Camera: {pose_result['position_object_to_camera'].squeeze()}")
        print(f"   Board in Camera: {pose_result['board_position_in_camera'].squeeze()}")


def setup_command_processor(modbus: ModbusRobotInterface, intrinsics) -> CommandProcessor:
    """Initialize and configure the command processor with vision processors.
    
    This is where you define which commands use which processors.
    """
    cp = CommandProcessor(modbus)
    
    # Initialize vision processors
    vp_standard = VisionProcessor("./best.pt", "./circle_detection_weights.pt", "./point_cloud/detected_circles.json")
    vp_gun = VisionProcessorGun(intrinsics)
    vp_port = VisionProcessorPort(intrinsics)
    
    # Register processors
    cp.register_processor('standard', vp_standard)
    cp.register_processor('gun', vp_gun)
    cp.register_processor('port', vp_port)
    
    response_type_pose_back = 1
    response_type_pose_main = 2
    response_type_select = response_type_pose_back
    temp = "gun"
    
    # response_type 1: pose_back, response_type 2: pose_main
    # Register commands with their processor mappings
    # Command 0: Gun processor, no offset
    cp.register_command(0, temp, response_type=response_type_select, use_offset=True)
    
    # Command 1: Grab the gun from port ('gun')
    cp.register_command(1, temp, response_type=response_type_select, use_offset=True)
    
    # Command 2: Put the gun to port ('port')
    cp.register_command(2, temp, response_type=response_type_select, use_offset=True)
    
    # Command 3: Grab the gun from charging port of car ('gun')
    cp.register_command(3, temp, response_type=response_type_select, use_offset=True)
    
    # Command 4: Put the gun to charging port of car ('standard')
    cp.register_command(4, temp, response_type=response_type_select, use_offset=True)

    return cp

def main_loop():
    """Main control loop that monitors robot commands and triggers vision processing."""
    
    with ModbusRobotInterface(ip="192.168.0.29", port=1502, timeout=0.1) as modbus:
        
        rs_utils = RSFramesUtils()
        
        if not modbus.is_connected():
            print("❌ Failed to connect to robot - running in simulation mode")
            print("   Vision will run but Modbus operations will be skipped")
        else:
            print("✅ Main loop started (listening to register 351)")
        
        # Setup command processor with all vision processors
        command_processor = setup_command_processor(modbus, rs_utils.intrinsics)
        
        frame_num = 0
        
        try:
            while True:
                
                # Read command from robot
                cmd = modbus.read_command()
                print(f"📡 Command: {cmd}")
                
                if cmd is None:
                    print("⚠️  No command received, retrying...")
                    time.sleep(1.0)
                    continue
                
                # Process command using appropriate processor
                command_processor.process_command(cmd, rs_utils)
                
        except KeyboardInterrupt:
            print("\n⚠️  Keyboard interrupt detected")
        except Exception as e:
            print(f"❌ Main loop error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            print("🛑 Main loop terminated")


if __name__ == "__main__":
    main_loop()