#!/usr/bin/env python3
"""
Generate ArUco markers for testing
"""

import cv2
import numpy as np
import os


def generate_single_markers():
    """Generate individual ArUco markers"""
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    
    # Generate 5 individual markers
    for marker_id in range(5):
        # Use the correct OpenCV API for drawing markers
        marker_image = np.zeros((200, 200), dtype=np.uint8)
        cv2.aruco.drawMarker(aruco_dict, marker_id, 200, marker_image, 1)
        
        filename = f'aruco_marker_{marker_id}.png'
        cv2.imwrite(filename, marker_image)
        print(f'Generated: {filename}')


def generate_test_scene():
    """Generate a test scene with multiple ArUco markers"""
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    
    # Create a blank white image (like a table/scene)
    scene_width = 1280
    scene_height = 720
    scene = np.ones((scene_height, scene_width, 3), dtype=np.uint8) * 255
    
    # Generate markers at different positions
    markers_data = [
        (0, 200, 150, 150),   # (id, x, y, size)
        (1, 600, 200, 120),
        (2, 950, 300, 100),
        (3, 300, 450, 130),
        (4, 800, 500, 110),
    ]
    
    for marker_id, x, y, size in markers_data:
        # Generate marker using the correct OpenCV API
        marker = np.zeros((size, size), dtype=np.uint8)
        cv2.aruco.drawMarker(aruco_dict, marker_id, size, marker, 1)
        
        # Convert to 3 channels if needed
        if len(marker.shape) == 2:
            marker = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        
        # Place marker on scene
        y_end = min(y + size, scene_height)
        x_end = min(x + size, scene_width)
        marker_h = y_end - y
        marker_w = x_end - x
        
        if marker_h > 0 and marker_w > 0:
            scene[y:y_end, x:x_end] = marker[:marker_h, :marker_w]
    
    # Add some visual elements (simulating a scene)
    cv2.putText(scene, 'ArUco Test Scene', (450, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 2)
    
    # Save the test scene
    filename = 'test_scene_with_aruco.jpg'
    cv2.imwrite(filename, scene)
    print(f'\nGenerated test scene: {filename}')
    print(f'Scene dimensions: {scene_width}x{scene_height}')
    print(f'Number of markers: {len(markers_data)}')


def main():
    print('ArUco Marker Generator')
    print('=' * 50)
    
    # Generate individual markers
    print('\nGenerating individual markers...')
    generate_single_markers()
    
    # Generate test scene
    print('\nGenerating test scene...')
    generate_test_scene()
    
    print('\n✓ Done! Test markers generated successfully.')
    print('\nYou can now use these images to test the ROS2 node:')
    print('  ros2 run weed_detection_node image_publisher test_scene_with_aruco.jpg')


if __name__ == '__main__':
    main()

