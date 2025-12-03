#!/usr/bin/env python3
"""
Generate Test Scenes with ArUco Markers
Creates synthetic test images with ArUco markers for weed detection testing
"""

import cv2
import numpy as np
import os

def create_aruco_marker(marker_id, size=200, dict_type=cv2.aruco.DICT_4X4_50):
    """
    Generate an ArUco marker image
    
    Args:
        marker_id: ID of the marker to generate
        size: Size of the marker in pixels
        dict_type: ArUco dictionary type
        
    Returns:
        numpy array with the marker image
    """
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size)
    return marker_img


def create_grass_texture(width, height):
    """
    Create a realistic grass texture background
    
    Args:
        width: Image width
        height: Image height
        
    Returns:
        numpy array with grass texture
    """
    # Create base green color with variations
    base_green = np.random.randint(40, 100, (height, width, 3), dtype=np.uint8)
    base_green[:, :, 1] = np.random.randint(80, 140, (height, width))  # More green
    base_green[:, :, 2] = np.random.randint(20, 60, (height, width))   # Less red
    
    # Add grass-like texture with Gaussian noise
    noise = np.random.normal(0, 15, (height, width, 3))
    grass = np.clip(base_green + noise, 0, 255).astype(np.uint8)
    
    # Add some dark spots to simulate soil/shadows
    num_spots = np.random.randint(20, 40)
    for _ in range(num_spots):
        x = np.random.randint(0, width)
        y = np.random.randint(0, height)
        radius = np.random.randint(5, 20)
        cv2.circle(grass, (x, y), radius, (30, 40, 20), -1)
    
    # Blur slightly for more realistic look
    grass = cv2.GaussianBlur(grass, (5, 5), 0)
    
    return grass


def add_weeds(image):
    """
    Add simple weed-like shapes to the image
    
    Args:
        image: Background image to add weeds to
        
    Returns:
        Image with added weeds
    """
    height, width = image.shape[:2]
    num_weeds = np.random.randint(5, 12)
    
    for _ in range(num_weeds):
        x = np.random.randint(50, width - 50)
        y = np.random.randint(50, height - 50)
        
        # Random weed color (darker or lighter green, brownish)
        weed_type = np.random.choice(['dark_green', 'light_green', 'brown'])
        if weed_type == 'dark_green':
            color = (20, 60, 30)
        elif weed_type == 'light_green':
            color = (60, 120, 40)
        else:
            color = (40, 70, 80)
        
        # Draw irregular weed shape
        num_points = np.random.randint(4, 8)
        points = []
        for i in range(num_points):
            angle = 2 * np.pi * i / num_points + np.random.uniform(-0.3, 0.3)
            radius = np.random.randint(10, 30)
            px = int(x + radius * np.cos(angle))
            py = int(y + radius * np.sin(angle))
            points.append([px, py])
        
        pts = np.array(points, dtype=np.int32)
        cv2.fillPoly(image, [pts], color)
        cv2.polylines(image, [pts], True, (10, 40, 20), 2)
    
    return image


def create_test_scene_1():
    """
    Create test scene 1: Multiple ArUco markers at different positions
    """
    print("Generating test_scene_1_aruco.jpg...")
    
    width, height = 1280, 720
    scene = create_grass_texture(width, height)
    scene = add_weeds(scene)
    
    # Marker configurations: (id, x, y, size, rotation)
    markers_config = [
        (0, 200, 150, 150, 0),
        (1, 650, 180, 120, 15),
        (2, 1000, 200, 140, -10),
        (3, 400, 450, 130, 25),
        (4, 850, 480, 110, -20),
    ]
    
    for marker_id, x, y, size, rotation in markers_config:
        marker = create_aruco_marker(marker_id, size)
        
        # Convert to BGR
        marker_bgr = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        
        # Apply rotation if needed
        if rotation != 0:
            center = (size // 2, size // 2)
            rot_matrix = cv2.getRotationMatrix2D(center, rotation, 1.0)
            marker_bgr = cv2.warpAffine(marker_bgr, rot_matrix, (size, size), 
                                       borderMode=cv2.BORDER_CONSTANT, 
                                       borderValue=(255, 255, 255))
        
        # Add white border
        border = 10
        marker_with_border = cv2.copyMakeBorder(marker_bgr, border, border, border, border,
                                                cv2.BORDER_CONSTANT, value=(255, 255, 255))
        
        # Place marker on scene
        mh, mw = marker_with_border.shape[:2]
        y1, y2 = y, y + mh
        x1, x2 = x, x + mw
        
        if y2 <= height and x2 <= width:
            # Blend marker with background
            alpha = 0.95
            scene[y1:y2, x1:x2] = cv2.addWeighted(marker_with_border, alpha, 
                                                  scene[y1:y2, x1:x2], 1-alpha, 0)
    
    # Add some realistic lighting variation
    vignette = np.zeros((height, width), dtype=np.float32)
    cv2.circle(vignette, (width//2, height//2), int(width*0.8), 1.0, -1)
    vignette = cv2.GaussianBlur(vignette, (0, 0), width/4)
    vignette = vignette[:, :, np.newaxis]
    scene = (scene * (0.7 + 0.3 * vignette)).astype(np.uint8)
    
    return scene


def create_test_scene_2():
    """
    Create test scene 2: Markers at various distances and orientations
    """
    print("Generating test_scene_2_aruco.jpg...")
    
    width, height = 1280, 720
    scene = create_grass_texture(width, height)
    scene = add_weeds(scene)
    
    # Marker configurations: closer markers (larger), farther markers (smaller)
    markers_config = [
        (5, 100, 100, 180, 30),    # Large, close
        (6, 500, 120, 160, -15),   # Medium-large
        (7, 900, 140, 100, 45),    # Medium, far
        (8, 300, 380, 140, -25),   # Medium
        (9, 700, 400, 90, 10),     # Small, far
        (10, 1050, 450, 120, -30), # Medium
    ]
    
    for marker_id, x, y, size, rotation in markers_config:
        marker = create_aruco_marker(marker_id, size)
        marker_bgr = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        
        # Apply rotation
        if rotation != 0:
            center = (size // 2, size // 2)
            rot_matrix = cv2.getRotationMatrix2D(center, rotation, 1.0)
            marker_bgr = cv2.warpAffine(marker_bgr, rot_matrix, (size, size), 
                                       borderMode=cv2.BORDER_CONSTANT, 
                                       borderValue=(255, 255, 255))
        
        # Add white border
        border = 8
        marker_with_border = cv2.copyMakeBorder(marker_bgr, border, border, border, border,
                                                cv2.BORDER_CONSTANT, value=(255, 255, 255))
        
        # Add slight shadow for realism
        shadow = np.zeros_like(marker_with_border)
        shadow_offset = 5
        mh, mw = marker_with_border.shape[:2]
        
        # Place marker on scene
        y1, y2 = y, y + mh
        x1, x2 = x, x + mw
        
        if y2 <= height and x2 <= width:
            # Add shadow first
            sy1, sy2 = y1 + shadow_offset, y2 + shadow_offset
            sx1, sx2 = x1 + shadow_offset, x2 + shadow_offset
            if sy2 <= height and sx2 <= width:
                scene[sy1:sy2, sx1:sx2] = cv2.addWeighted(scene[sy1:sy2, sx1:sx2], 0.7,
                                                          shadow, 0.3, 0)
            
            # Then place marker
            alpha = 0.95
            scene[y1:y2, x1:x2] = cv2.addWeighted(marker_with_border, alpha, 
                                                  scene[y1:y2, x1:x2], 1-alpha, 0)
    
    # Add lens distortion effect
    k1, k2, p1, p2 = -0.02, 0.001, 0, 0
    camera_matrix = np.array([[width, 0, width/2],
                             [0, height, height/2],
                             [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.array([k1, k2, p1, p2], dtype=np.float32)
    
    return scene


def main():
    """Generate both test scenes"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Generate scene 1
    scene1 = create_test_scene_1()
    output_path1 = os.path.join(script_dir, 'test_scene_1_aruco.jpg')
    cv2.imwrite(output_path1, scene1)
    print(f"✓ Saved: {output_path1}")
    print(f"  Contains markers: ID 0, 1, 2, 3, 4")
    
    # Generate scene 2
    scene2 = create_test_scene_2()
    output_path2 = os.path.join(script_dir, 'test_scene_2_aruco.jpg')
    cv2.imwrite(output_path2, scene2)
    print(f"✓ Saved: {output_path2}")
    print(f"  Contains markers: ID 5, 6, 7, 8, 9, 10")
    
    print("\n✅ Both test scenes generated successfully!")
    print("You can test them with:")
    print("  ros2 run weed_detection_node image_publisher test_scene_1_aruco.jpg 1.0")
    print("  ros2 run weed_detection_node image_publisher test_scene_2_aruco.jpg 1.0")


if __name__ == '__main__':
    main()
