import cv2
import cv2.aruco as aruco
import numpy as np

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

for marker_id in [1, 2]:
    # Create a bigger marker for better detection
    marker_img = aruco.generateImageMarker(aruco_dict, marker_id, 400)

    # Optional: add white border
    marker_img = np.pad(marker_img, pad_width=20, mode='constant', constant_values=255)

    filename = f"aruco_{marker_id}.png"
    cv2.imwrite(filename, marker_img)
    print(f"Saved {filename}")
