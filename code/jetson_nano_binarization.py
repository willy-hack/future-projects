import numpy as np
import cv2

# Attempt to open the camera
imcap = cv2.VideoCapture('nvarguscamerasrc ! video/x-raw(memory:NVMM), width=720, height=480, format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink', cv2.CAP_GSTREAMER)

if not imcap.isOpened():
    print("Error: Could not open video device.")
    exit()

# Create a window to display the slider
cv2.namedWindow('Threshold Adjustment')

# Callback function for the slider (does nothing)
def nothing(x):
    pass

# Create a slider
cv2.createTrackbar('Threshold', 'Threshold Adjustment', 127, 255, nothing)

while True:
    ret, imageFrame = imcap.read()

    # Check if the frame was read successfully
    if not ret:
        print("Error: Failed to capture image")
        break

    # Convert the frame to a grayscale image
    grayFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2GRAY)

    # Get the threshold value from the slider position
    threshold_value = cv2.getTrackbarPos('Threshold', 'Threshold Adjustment')

    # Apply binary thresholding
    _, binaryFrame = cv2.threshold(grayFrame, threshold_value, 255, cv2.THRESH_BINARY)

    # Display the original image and the binary image
    cv2.imshow("Original Image", imageFrame)
    cv2.imshow("Threshold Adjustment", binaryFrame)

    # Press 'q' to exit
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

imcap.release()  
cv2.destroyAllWindows()  
