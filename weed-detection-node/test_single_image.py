#!/usr/bin/env python3
"""
Single image test - tests the ArUco detection without ROS2
This directly tests the core functionality
"""

import cv2
import numpy as np
import json

def test_aruco_detection(image_path):
    """Test ArUco detection on a single image"""
    
    print("=" * 60)
    print("Testing ArUco Detection (Standalone)")
    print("=" * 60)
    
    # Load image
    print(f"\n1. Loading image: {image_path}")
    image = cv2.imread(image_path)
    if image is None:
        print(f"❌ Failed to load image: {image_path}")
        return False
    
    print(f"   ✓ Image loaded: {image.shape}")
    
    # Setup ArUco detector
    print("\n2. Setting up ArUco detector...")
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters_create()
    print("   ✓ ArUco detector initialized")
    
    # Detect markers (compatible with OpenCV 4.6+)
    print("\n3. Detecting ArUco markers...")
    corners, ids, rejected = cv2.aruco.detectMarkers(image, aruco_dict, parameters=aruco_params)
    
    if ids is not None and len(ids) > 0:
        print(f"   ✓ Detected {len(ids)} markers!")
        
        # Create annotated image
        annotated_image = image.copy()
        cv2.aruco.drawDetectedMarkers(annotated_image, corners, ids)
        
        # Extract coordinates
        print("\n4. Marker Details:")
        coordinates_data = {'markers': []}
        
        for i, marker_id in enumerate(ids):
            marker_corners = corners[i][0]
            center_x = int(np.mean(marker_corners[:, 0]))
            center_y = int(np.mean(marker_corners[:, 1]))
            
            marker_info = {
                'id': int(marker_id[0]),
                'center': {'x': center_x, 'y': center_y},
                'corners': [
                    {'x': int(corner[0]), 'y': int(corner[1])}
                    for corner in marker_corners
                ]
            }
            
            coordinates_data['markers'].append(marker_info)
            
            print(f"   Marker ID {marker_id[0]}: Center at ({center_x}, {center_y})")
            
            # Draw center point
            cv2.circle(annotated_image, (center_x, center_y), 5, (0, 255, 0), -1)
            cv2.putText(
                annotated_image,
                f'ID: {marker_id[0]}',
                (center_x - 20, center_y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )
        
        # Save annotated image
        output_path = 'test_output_annotated.jpg'
        cv2.imwrite(output_path, annotated_image)
        print(f"\n5. Saved annotated image: {output_path}")
        
        # Save coordinates
        coord_path = 'test_output_coordinates.json'
        with open(coord_path, 'w') as f:
            json.dump(coordinates_data, f, indent=2)
        print(f"   Saved coordinates: {coord_path}")
        
        print("\n" + "=" * 60)
        print("✓ TEST PASSED - ArUco detection working correctly!")
        print("=" * 60)
        
        return True
    else:
        print("   ❌ No markers detected!")
        print("\n" + "=" * 60)
        print("✗ TEST FAILED - No markers found")
        print("=" * 60)
        return False


if __name__ == '__main__':
    import sys
    
    image_path = 'test_scene_with_aruco.jpg'
    if len(sys.argv) > 1:
        image_path = sys.argv[1]
    
    success = test_aruco_detection(image_path)
    sys.exit(0 if success else 1)

