import cv2
import numpy as np
import serial as AC
import struct
import time
import Adafruit_BNO055.BNO055 as BNO055
from function import process_roi, detect_color, pd_control
import Jetson.GPIO as GPIO

# Define sensing regions
rois = [
    (370, 150, 640, 200),  # Large right sensing region
    (0, 150, 270, 200),  # Large left sensing region
]
GPIO.setmode(GPIO.BOARD)  # Use pin numbering mode
target_heading = [0] * 4
left_heading = [-90, -180, 90, 0]
right_heading = [90, 180, -90, 0]
turn_side = 0
kp_roi = 0.008  # Default gain for ROI proportion
kd_roi = 0.01  # Default gain for ROI derivative
combined_control_signal = 0
count = 0
PWM = 100
round_number = 0
data_to_send = 0
output_pin = 40
GPIO.setup(output_pin, GPIO.OUT)
# Load calibration data
calibration_data = np.load('calibration_data.npz')
camera_matrix = calibration_data['camera_matrix']
distortion_coefficients = calibration_data['distortion_coefficients']

# Attempt to open the serial port
try:
    ser = AC.Serial('/dev/ttyTHS1', 115200, timeout=1)  # Adjust baud rate as needed
except AC.SerialException as e:
    print(f"Error: Could not open serial port: {e}")
    exit()


def main():
    global turn_side
    global target_heading
    global left_heading
    global right_heading
    global count
    global PWM
    global round_number
    global data_to_send
    current_last = 0  # Initialize current_last variable
    window_name = "Camera Preview"
    binary_window_name = "Binary Preview"
    
    # Open the camera
    cap = cv2.VideoCapture('nvarguscamerasrc ! video/x-raw(memory:NVMM), width=640, height=480, format=(string)NV12, framerate=12/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true sync=false', cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("Error: Could not open video stream.")
        return
    bno = BNO055.BNO055(busnum=1)

    if not bno.begin():
        raise RuntimeError('Failed to initialize BNO055!')

    GPIO.output(output_pin, GPIO.HIGH)
    combined_control_signal = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        # Read sensor data from BNO055
        heading, roll, pitch = bno.read_euler()
        accel_x, accel_y, accel_z = bno.read_linear_acceleration()
        if heading > 180:
            heading -= 360
        # Undistort the image using calibration parameters
        h, w = frame.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, (w, h), 0.12, (w, h))
        undistorted_frame = cv2.undistort(frame, camera_matrix, distortion_coefficients, None, new_camera_matrix)

        # Create a binary copy
        gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)
        _, binary_frame = cv2.threshold(gray, 105, 255, cv2.THRESH_BINARY)
        binary_frame = cv2.cvtColor(binary_frame, cv2.COLOR_GRAY2BGR)

        roi_values = []
        for i, (x1, y1, x2, y2) in enumerate(rois):
            processed_roi, black_pixels = process_roi(undistorted_frame, x1, y1, x2, y2)  # Get the number of black pixels
            roi_values.append(black_pixels)
            binary_frame[y1:y2, x1:x2] = processed_roi

        # Color detection, returning Y coordinates
        color_y_positions = detect_color(undistorted_frame)
        
        # Straight driving
        if turn_side == 0 or turn_side == 2:
            PWM = 100
            if color_y_positions[0] > color_y_positions[1]:     
                if target_heading == [0] * 4:
                    target_heading = right_heading
                if color_y_positions[0] > 290:
                    turn_side = 1
            elif color_y_positions[0] < color_y_positions[1]:
                if target_heading == [0] * 4:
                    target_heading = left_heading
                if color_y_positions[1] > 270:
                    turn_side = 1    
            else:
                turn_side = 0
            # Select PD control signal based on data mode
            if roi_values[0] >= roi_values[1]:
                if roi_values[0] >= 4500:
                    print("right")
                    combined_control_signal = pd_control(4500, roi_values[0], kp_roi, kd_roi)
            else:
                if roi_values[1] >= 4500:
                    print("left")
                    combined_control_signal = -pd_control(4500, roi_values[1], kp_roi, kd_roi)
            start_time = time.time()  # Get current time (seconds)
        
        # Turning
        if turn_side == 1:
            PWM = 100
            # Select PD control signal based on data mode
            if target_heading == left_heading:
                if roi_values[1] >= 5000:
                    combined_control_signal = pd_control(5000, roi_values[1], kp_roi, kd_roi)
                else:
                    combined_control_signal = -80
            else:
                if roi_values[0] >= 5000:
                    combined_control_signal = -pd_control(4000, roi_values[0], kp_roi, kd_roi)
                else:
                    combined_control_signal = 40
            current_time = time.time()
            elapsed_time = current_time - start_time
            if elapsed_time >= 1 and heading < target_heading[count] + 35 and heading > target_heading[count] - 35 and color_y_positions[0] == 0 and color_y_positions[1] == 0:
                turn_side = 2
                if count >= 3:
                    count = 0
                    round_number += 1
                else:
                    count += 1
                combined_control_signal = 0
                    
        # Limit the control signal to a specific range
        if combined_control_signal > 180:
            combined_control_signal = 180
        if combined_control_signal < -180:
            combined_control_signal = -180

        # Prepare data to be sent
        data_to_send = (int(combined_control_signal), int(turn_side), int(PWM))
        # Print data to be sent for debugging purposes
        print("Sent: ", data_to_send)

        # Add packet header "A" to the data packet
        header = b"A"
        send_data_value = struct.pack('3i', *data_to_send)  # Ensure 3 integers are packed
        send_data_value = header + send_data_value

        # Send data with packet header
        ser.write(send_data_value)

        # Display two windows
        cv2.imshow(window_name, undistorted_frame)
        cv2.imshow(binary_window_name, binary_frame)
        
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
