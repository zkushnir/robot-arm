"""Test RealSense camera and block detection"""
import cv2
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from armstack.vision.realsense_camera import RealSenseCamera
from armstack.vision.block_detector import BlockDetector


def main():
    print("Starting RealSense camera test...")
    print("Press 'q' to quit")
    print("Press 'r/g/b/y' to change target color (red/green/blue/yellow)")
    print("Press 'd' to toggle debug view")
    print("Press 's' to save current frame")

    target_color = 'red'
    debug_mode = True

    try:
        with RealSenseCamera(width=640, height=480, fps=30) as camera:
            detector = BlockDetector()

            while True:
                # Get frames
                color_image, depth_image, depth_colormap = camera.get_frames()

                if color_image is None:
                    continue

                # Detect blocks
                if debug_mode:
                    blocks, debug_image = detector.detect_blocks(
                        color_image, target_color=target_color, debug=True
                    )
                    display_image = debug_image
                else:
                    blocks = detector.detect_blocks(color_image, target_color=target_color)
                    display_image = color_image

                # Print block info
                if blocks:
                    print(f"\rFound {len(blocks)} {target_color} block(s)   ", end='')
                    for i, block in enumerate(blocks[:3]):  # Show top 3
                        cx, cy = block['center']
                        if depth_image is not None:
                            depth_mm = depth_image[cy, cx]
                            point_3d = camera.get_3d_point(cx, cy, depth_image)
                            x, y, z = point_3d
                            print(f"  Block {i+1}: ({cx}, {cy}) -> 3D: ({x*1000:.1f}, {y*1000:.1f}, {z*1000:.1f})mm", end='')
                else:
                    print(f"\rNo {target_color} blocks found   ", end='')

                # Display
                cv2.imshow('RealSense - Color', display_image)
                if depth_colormap is not None:
                    cv2.imshow('RealSense - Depth', depth_colormap)

                # Handle keyboard
                key = cv2.waitKey(1) & 0xFF

                if key == ord('q'):
                    break
                elif key == ord('r'):
                    target_color = 'red'
                    print("\nSwitched to RED detection")
                elif key == ord('g'):
                    target_color = 'green'
                    print("\nSwitched to GREEN detection")
                elif key == ord('b'):
                    target_color = 'blue'
                    print("\nSwitched to BLUE detection")
                elif key == ord('y'):
                    target_color = 'yellow'
                    print("\nSwitched to YELLOW detection")
                elif key == ord('d'):
                    debug_mode = not debug_mode
                    print(f"\nDebug mode: {debug_mode}")
                elif key == ord('s'):
                    filename = f"capture_{target_color}.png"
                    cv2.imwrite(filename, display_image)
                    print(f"\nSaved {filename}")

    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()
        print("\nCamera test complete")


if __name__ == "__main__":
    main()
