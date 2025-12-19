from abc import ABC, abstractmethod


class VisionProcessorAbstract(ABC):
    @abstractmethod
    def process_frame(self, color_image, intrinsics, marker_poses):
        """Process a single frame for object detection and pose estimation.
        Input: ...
        Returns: current_pose"""

        pass