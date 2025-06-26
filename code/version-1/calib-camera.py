import cv2
import numpy as np
import glob

# Set the size of the checkerboard (number of inner corners)
checkerboard_size = (8, 5)

# Prepare the points in the 3D world coordinate system
objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)

# Lists to store 3D and 2D points
objpoints = []  # Store 3D points in the world coordinate system
imgpoints = []  # Store 2D points in the image

# Open the CSI camera using a GStreamer pipeline
cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! appsink", cv2.CAP_GSTREAMER)

# Check if the camera is opened
if not cap.isOpened():
    print("Unable to open the camera")
    exit()

image_count = 0  # Track the number of captured images

while True:
    ret, frame = cap.read()  # Capture an image from the camera
    if not ret:
        print("Unable to get an image from the camera")
        break

    # Display the camera frame
    cv2.imshow('Press Space to Capture', frame)

    key = cv2.waitKey(1)

    # Press the space key to capture an image for calibration
    if key & 0xFF == ord(' '):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert the image to grayscale
        
        # Find the inner corners of the checkerboard
        ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

        if ret:
            objpoints.append(objp)  # Save the world coordinate points
            # Refine corner detection
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            imgpoints.append(corners2)  # Save the image coordinate points
            
            # Draw and display the corners
            frame = cv2.drawChessboardCorners(frame, checkerboard_size, corners2, ret)
            print(f"Checkerboard inner corners detected, number of corners: {len(corners2)}")

            # Save the calibration image
            image_path = f'calibration_image_{image_count}.jpg'
            cv2.imwrite(image_path, frame)
            print(f'Image saved to: {image_path}')
            image_count += 1
        else:
            print("Checkerboard inner corners not detected, please try again.")

    # Press 'q' to quit
    if key & 0xFF == ord('q'):
        break

# Release the camera after finishing
cap.release()
cv2.destroyAllWindows()

# Check if enough calibration images have been collected
if len(objpoints) > 0 and len(imgpoints) > 0:
    # Perform camera calibration with the detected points
    ret, camera_matrix, distortion_coefficients, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # Print the calibration results
    print(f"Camera matrix:\n{camera_matrix}")
    print(f"Distortion coefficients:\n{distortion_coefficients}")

    # Save the calibration data
    np.savez("calibration_data.npz", camera_matrix=camera_matrix, distortion_coefficients=distortion_coefficients)
    print("Calibration data saved to 'calibration_data.npz'")
else:
    print("Not enough images collected for calibration.")
