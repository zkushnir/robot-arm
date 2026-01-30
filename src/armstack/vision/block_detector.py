"""Block detection using color and shape"""
import cv2
import numpy as np


class BlockDetector:
    """Detect colored blocks in images"""

    def __init__(self):
        # Default HSV color ranges (you'll tune these for your block)
        self.color_ranges = {
            'red': {
                'lower1': np.array([0, 100, 100]),
                'upper1': np.array([10, 255, 255]),
                'lower2': np.array([160, 100, 100]),
                'upper2': np.array([180, 255, 255])
            },
            'blue': {
                'lower': np.array([100, 100, 100]),
                'upper': np.array([130, 255, 255])
            },
            'green': {
                'lower': np.array([40, 100, 100]),
                'upper': np.array([80, 255, 255])
            },
            'yellow': {
                'lower': np.array([20, 100, 100]),
                'upper': np.array([40, 255, 255])
            }
        }

        self.min_area = 500  # Minimum contour area in pixels
        self.max_area = 50000  # Maximum contour area

    def detect_blocks(self, color_image, target_color='red', debug=False):
        """
        Detect blocks of a specific color

        Args:
            color_image: BGR image from camera
            target_color: 'red', 'blue', 'green', or 'yellow'
            debug: If True, return debug visualization

        Returns:
            blocks: List of dicts with keys: 'center', 'area', 'contour', 'bbox'
            debug_image: Visualization image (if debug=True)
        """
        # Convert to HSV
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # Create mask based on color
        if target_color not in self.color_ranges:
            raise ValueError(f"Unknown color: {target_color}")

        color_range = self.color_ranges[target_color]

        # Handle red which wraps around HSV hue
        if target_color == 'red':
            mask1 = cv2.inRange(hsv, color_range['lower1'], color_range['upper1'])
            mask2 = cv2.inRange(hsv, color_range['lower2'], color_range['upper2'])
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv, color_range['lower'], color_range['upper'])

        # Morphological operations to clean up mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter and extract block info
        blocks = []
        for contour in contours:
            area = cv2.contourArea(contour)

            # Filter by area
            if area < self.min_area or area > self.max_area:
                continue

            # Get bounding box
            x, y, w, h = cv2.boundingRect(contour)

            # Get center
            M = cv2.moments(contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
            else:
                cx, cy = x + w // 2, y + h // 2

            blocks.append({
                'center': (cx, cy),
                'area': area,
                'contour': contour,
                'bbox': (x, y, w, h)
            })

        # Sort by area (largest first)
        blocks.sort(key=lambda b: b['area'], reverse=True)

        if debug:
            debug_image = self._create_debug_image(color_image, mask, blocks, target_color)
            return blocks, debug_image

        return blocks

    def _create_debug_image(self, color_image, mask, blocks, target_color):
        """Create visualization for debugging"""
        debug_img = color_image.copy()

        # Draw contours and info
        for i, block in enumerate(blocks):
            cx, cy = block['center']
            x, y, w, h = block['bbox']

            # Draw bounding box
            cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Draw center
            cv2.circle(debug_img, (cx, cy), 5, (0, 0, 255), -1)

            # Draw contour
            cv2.drawContours(debug_img, [block['contour']], -1, (255, 0, 0), 2)

            # Add text
            text = f"{target_color} #{i+1}: {block['area']:.0f}px"
            cv2.putText(debug_img, text, (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Show mask in corner
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        h, w = mask_colored.shape[:2]
        small_mask = cv2.resize(mask_colored, (w // 4, h // 4))
        debug_img[0:small_mask.shape[0], 0:small_mask.shape[1]] = small_mask

        return debug_img

    def set_color_range(self, color_name, lower, upper, lower2=None, upper2=None):
        """
        Set custom HSV color range

        Args:
            color_name: Name for this color
            lower: Lower HSV bound (H, S, V)
            upper: Upper HSV bound (H, S, V)
            lower2, upper2: Optional second range (for red wrapping)
        """
        self.color_ranges[color_name] = {
            'lower': np.array(lower),
            'upper': np.array(upper)
        }

        if lower2 is not None and upper2 is not None:
            self.color_ranges[color_name]['lower2'] = np.array(lower2)
            self.color_ranges[color_name]['upper2'] = np.array(upper2)
