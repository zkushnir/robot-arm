"""RealSense camera wrapper for robot vision"""
import numpy as np
import pyrealsense2 as rs


class RealSenseCamera:
    """Wrapper for Intel RealSense camera"""

    def __init__(self, width=640, height=480, fps=30):
        self.width = width
        self.height = height
        self.fps = fps

        self.pipeline = None
        self.config = None
        self.align = None

    def start(self):
        """Initialize and start the camera"""
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Enable color and depth streams
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)

        # Start streaming
        profile = self.pipeline.start(self.config)

        # Create alignment object (align depth to color)
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        # Get camera intrinsics
        color_stream = profile.get_stream(rs.stream.color)
        self.intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

        print(f"RealSense camera started: {self.width}x{self.height} @ {self.fps}fps")

    def stop(self):
        """Stop the camera"""
        if self.pipeline:
            self.pipeline.stop()
            print("RealSense camera stopped")

    def get_frames(self):
        """
        Get aligned color and depth frames

        Returns:
            color_image: numpy array (H, W, 3) BGR image
            depth_image: numpy array (H, W) depth in mm
            depth_colormap: numpy array (H, W, 3) colorized depth for visualization
        """
        # Wait for frames
        frames = self.pipeline.wait_for_frames()

        # Align depth to color
        aligned_frames = self.align.process(frames)

        # Get aligned frames
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return None, None, None

        # Convert to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Create colormap for depth visualization
        depth_colormap = rs.colorizer().colorize(depth_frame)
        depth_colormap = np.asanyarray(depth_colormap.get_data())

        return color_image, depth_image, depth_colormap

    def get_3d_point(self, pixel_x, pixel_y, depth_image):
        """
        Convert 2D pixel + depth to 3D point in camera frame

        Args:
            pixel_x: X coordinate in image
            pixel_y: Y coordinate in image
            depth_image: Depth image from get_frames()

        Returns:
            (x, y, z) in meters in camera coordinate frame
        """
        depth_mm = depth_image[pixel_y, pixel_x]
        depth_m = depth_mm / 1000.0  # Convert mm to meters

        # Deproject pixel to 3D point
        point_3d = rs.rs2_deproject_pixel_to_point(
            self.intrinsics, [pixel_x, pixel_y], depth_m
        )

        return point_3d  # Returns (x, y, z) in meters

    def __enter__(self):
        """Context manager support"""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager support"""
        self.stop()
