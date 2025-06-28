import cv2 # type: ignore
import numpy as np
import serial as AC # type: ignore
import struct
import time
import Adafruit_BNO055.BNO055 as BNO055 # type: ignore
from function import process_roi, detect_color_final, pd_control, draw_multiple_curves,detect_color_final # type: ignore
import Jetson.GPIO as GPIO # type: ignore

# Define sensing regions
rois = [
    (370, 170, 640, 220),  # Large right sensing region
    (0, 170, 270, 220),  # Large left sensing region
    (275, 250, 355, 330),  # Final region
]
led_pin = 40
GPIO.setmode(GPIO.BOARD)  # Use pin numbering mode
GPIO.setup(led_pin, GPIO.OUT)


#stright_line
start_pts = [(150, 0), (600, 0)]
end_pts = [(0, 639), (639, 639)]
pink_colors = [(255, 0, 255), (255, 0, 255)]  # 兩條粉紅色線
# Define colors, start points, and end points for multiple lines
colors = [(0, 0, 255), (0, 255, 0)]  # Red and Green
start_points = [(0,400), (639, 300)]  # List of start points
end_points = [(300,30), (300, 70)]  # Lst of end pointsx上點 x 小向左y 小向上
slope_values = [0.45, -0.45]  # Slope values
curvature_factors = [0.45, 0.45]  # Curvature values
red =100
green = 140
roi_values0 = 3000
roi_values1 = 3000


target_heading = [0] * 5
left_heading = [0, -90, -180, 90, 0]
right_heading = [0, 90, 180, -90, 0]
red_left_heading = [180, 90, 0, -90, -180]
red_right_heading = [-180, -90, 0, 90, 180]
pink_positions = [0] * 4
color_y_positions = [0] * 4
turn_side = 0
kp_roi = 0.02  # Default gain for ROI proportion
kd_roi = 0.03  # Default gain for ROI derivative
kp_heading = 4  # Default proportional gain for heading
kd_heading = 8  # Default derivative gain for heading
kp_X = 0.4 # Default proportional gain for heading
kd_X = 0.8 # Default derivative gain for heading
combined_control_signal = 0
count = 0
PWM = 0
round_number = 0
turn_side = 8
turn_time = 0
turn_diside = False
pass_block = True
ROI2 = False
ROI34 = False
time1 = True
park_side = 0
stop = True
data_to_send = 0
turn_target = 0
turn_heading = 0
RED_X = 0
GREEN_X = 0
turn_debug = True
ROI_0 = False
ROI_1 = False
start_1 = False
start_2 = False
start_0 = True
start_3 = False


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
    global turn_time
    global turn_diside
    global pass_block
    global color_y_positions
    global pink_positions
    global ROI2
    global ROI34
    global time1
    global park_side
    global stop
    global data_to_send
    global turn_target 
    global turn_heading 
    global turn_debug 
    global turn_right_diside
    global red
    global green
    global roi_values0
    global ROI_0
    global ROI_1
    global start_2
    global start_1 
    global start_0
    global start_3
    window_name = "Camera Preview"
    binary_window_name = "Binary Preview"

    # Initial red and green X difference
    last_red_x_diff = 0
    last_green_x_diff = 0
    last_pink_red_x_diff = 0
    last_pink_green_x_diff = 0

    # Open the camera
    cap = cv2.VideoCapture('nvarguscamerasrc ! video/x-raw(memory:NVMM), width=640, height=480, format=(string)NV12, framerate=11/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true sync=false', cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("Error: Could not open video stream.")
        return
    bno = BNO055.BNO055(busnum=1)

    if not bno.begin():
        raise RuntimeError('Failed to initialize BNO055!')
    combined_control_signal = 0
    start_time = 0
    current_time = 0
    elapsed_time = 0
    time_count = 0
    turn_time = 0
    park_final = 0
    stop = True
    GPIO.output(led_pin, GPIO.HIGH)
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
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, (w, h), 0.1, (w, h))
        undistorted_frame = cv2.undistort(frame, camera_matrix, distortion_coefficients, None, new_camera_matrix)

        # Create a binary copy
        gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)
        _, binary_frame = cv2.threshold(gray, 105, 255, cv2.THRESH_BINARY)
        binary_frame = cv2.cvtColor(binary_frame, cv2.COLOR_GRAY2BGR)

        # Process each sensing region
        roi_values = []
        for x1, y1, x2, y2 in rois:
            processed_roi, black_pixels = process_roi(undistorted_frame, x1, y1, x2, y2)
            roi_values.append(black_pixels)
            binary_frame[y1:y2, x1:x2] = processed_roi

       
        # Perform color detection and pass red_curve_points, obtaining the X difference for red and green
        color_y_positions, pink_positions, red_x_diff, green_x_diff, pink_red_x_diff, pink_green_x_diff = detect_color_final(undistorted_frame, last_red_x_diff, last_green_x_diff, start_points, end_points, slope_values, curvature_factors, colors,start_pts, end_pts)

        # Save the current X difference values for red and green for use in the next frame
        last_red_x_diff = red_x_diff
        last_green_x_diff = green_x_diff

        #turn_side = 4
        #target_heading = left_heading

        #start leaving parking lot
        if turn_side == 8:
            PWM = -45
            if  roi_values[0]> roi_values[1] and start_0:
                ROI_0 = True
                start_1 = True
                start_0 = False
            elif  roi_values[0]< roi_values[1]and start_0:
                ROI_1 = True
                start_1 = True
                start_0 = False
            if  abs(heading) < 60 and start_1 or abs(heading) > 80 and start_1:
                if ROI_1:
                    combined_control_signal = 180
                elif ROI_0:
                    combined_control_signal = -180
            if abs(heading) > 60 and abs(heading) < 80 and start_1:
                start_2 = True
                start_1 = False
            if start_2:
                PWM = 40
                combined_control_signal = 0
            if start_2 and roi_values[2] > 4000:
                start_2 = False
                start_3 = True
            if abs(heading) > 10 and abs(heading) < 170 and start_3:
                if ROI_1:
                    combined_control_signal = -180
                elif ROI_0:
                    combined_control_signal = 180
            if abs(heading) < 10 and start_3  or abs(heading) > 170 and start_3:
                start_2 = False
                start_1 = False
                turn_side = 0

        if turn_side == 0 or turn_side == 2 or turn_side == 6:
            PWM = 50
            if target_heading == [0] * 5:
                if color_y_positions[0] > color_y_positions[1] and  color_y_positions[0] > 270:
                   target_heading = right_heading
                elif color_y_positions[0] < color_y_positions[1] and color_y_positions[1] > 270:
                    target_heading = left_heading
            if color_y_positions[0] > color_y_positions[1] and  color_y_positions[0] > 270 and target_heading == right_heading or target_heading == red_right_heading and color_y_positions[0] > color_y_positions[1] and  color_y_positions[0] > 270:
                turn_side = 1
                if round_number == 2 and 3 == count:
                    if stop:
                        turn_side = 6
                        stop = False
                        current_time = 0
                        elapsed_time = 0
                        start_time = time.time()
                        time_count = 3
                        turn_side = 4
            elif color_y_positions[1] > color_y_positions[0] and  color_y_positions[1] > 270 and target_heading == left_heading or target_heading == red_left_heading and color_y_positions[1] > color_y_positions[0] and  color_y_positions[1] > 270:
                turn_side = 1  
                if round_number == 2 and 3 == count:
                    if stop:
                        turn_side = 6
                        stop = False  
                        current_time = 0
                        elapsed_time = 0 
                        start_time = time.time()
                        time_count = 3
                        turn_side = 4
            else:
                turn_side = 0
            # Select PD control signal based on data mode
            if color_y_positions[2] < color_y_positions[3] and green_x_diff != 0 and color_y_positions[3] > 20:
                print("green")
                if  color_y_positions[3]>200 :
                    turn_diside = False
                combined_control_signal = -pd_control(green, green_x_diff, kp_X, kd_X)
            elif color_y_positions[2] > color_y_positions[3] and red_x_diff != 0 and color_y_positions[2] > 20:
                print("red")
                if  color_y_positions[2]>200 :
                    turn_diside = True
                combined_control_signal = pd_control(red, red_x_diff, kp_X, kd_X)
            elif roi_values[0] >= roi_values0 and heading < target_heading[count] + 15 and heading > target_heading[count] - 15 and roi_values[1] < roi_values[0] and abs(target_heading[count]) != 180 or abs(target_heading[count]) == 180 and roi_values[0] >= roi_values0 and heading < -170 and heading > 170 and roi_values[1] < roi_values[0] :
                print("right")
                combined_control_signal = pd_control(roi_values0, roi_values[0], kp_roi, kd_roi)
            elif roi_values[1] >= roi_values1 and heading < target_heading[count] + 15 and heading > target_heading[count] - 15 and roi_values[0] < roi_values[1] and abs(target_heading[count]) != 180 or abs(target_heading[count]) == 180 and roi_values[1] >= roi_values1 and heading < -170 and heading > 170 and roi_values[0] < roi_values[1] : 
                print("left")
                combined_control_signal = -pd_control(roi_values1, roi_values[1], kp_roi, kd_roi)
            else:
                turn_target = 0
                turn_heading = heading-target_heading[count]
                if target_heading[count] == -180 and heading > 0 or target_heading[count] == 180 and heading < 0: 
                    turn_target = 0
                    turn_heading = heading+target_heading[count]
                elif target_heading[count] == -90 and heading > 90:
                    turn_target = 0
                    turn_heading = -270+heading
                elif target_heading[count] == 90 and heading < -90:
                    turn_target = 0
                    turn_heading = 270-heading 
                
                combined_control_signal = pd_control(turn_target, turn_heading, kp_heading, kd_heading)
                print("heading",turn_target,turn_heading,target_heading[count])
                if combined_control_signal > 150:
                    combined_control_signal = 150
                if combined_control_signal < -150:
                    combined_control_signal = -150
            start_time = time.time() # Get current time (seconds)
            turn_debug = True

        # Turning
        if turn_side == 1:
            PWM = 40
            # Select PD control signal based on data mode
            if color_y_positions[2] < color_y_positions[3] and green_x_diff != 0:
                print("green")
                if  color_y_positions[3]>120 :
                    turn_diside = False
                combined_control_signal = -pd_control(green, green_x_diff, kp_X, kd_X)
            elif color_y_positions[2] > color_y_positions[3] and red_x_diff != 0:
                print("red")
                if  color_y_positions[2]>120 :
                    turn_diside = True
                combined_control_signal = pd_control(red, red_x_diff, kp_X, kd_X)         

            else:
                if target_heading[count] == -180 and target_heading[count+1] == 90 or target_heading[count] == 180 and target_heading[count+1] == -90:  
                    if target_heading[count] == -180 and heading > 0 or target_heading[count] == 180 and heading < 0: 
                        turn_target = target_heading[count+1]+target_heading[count]
                        turn_heading = heading+target_heading[count]
                    else:
                        turn_target = target_heading[count+1]+target_heading[count]
                        turn_heading = heading-target_heading[count]
                elif target_heading[count+1] == -180 and heading > 0 or target_heading[count+1] == 180 and heading < 0: 
                    turn_target = 0
                    turn_heading = heading+target_heading[count+1] 
                else:
                    if target_heading[count] == -180 and heading > 0 or target_heading[count] == 180 and heading < 0: 
                        turn_target = target_heading[count+1]-target_heading[count]
                        turn_heading = heading+target_heading[count]
                    else:
                        turn_target = target_heading[count+1]-target_heading[count]
                        turn_heading = heading-target_heading[count]     
                if target_heading[count+1] == -90 and heading > 90:
                    turn_target = 0
                    turn_heading = -270+heading
                elif target_heading[count+1] == 90 and heading < -90:
                    turn_target = 0
                    turn_heading = 270-heading 
                print("heading_turn",turn_target,turn_heading,count)
                combined_control_signal = pd_control(turn_target, turn_heading, kp_heading, kd_heading)
                
                if turn_debug and target_heading == left_heading or target_heading == red_left_heading and turn_debug:
                    combined_control_signal = -180
                    turn_debug = False
                elif turn_debug and target_heading == right_heading or target_heading == red_right_heading and turn_debug:
                    combined_control_signal = 180
                    turn_debug = False
                # Limit the control signal to a specific range
                if combined_control_signal > 155:
                    combined_control_signal = 155
                if combined_control_signal < -155:
                    combined_control_signal = -155
            current_time = time.time()
            elapsed_time = current_time - start_time
            if elapsed_time >= 2.9 and color_y_positions[0] == 0 and target_heading == right_heading or target_heading == red_right_heading and elapsed_time >= 2.9 and color_y_positions[0] == 0 or elapsed_time >= 2.9 and color_y_positions[1] == 0 and target_heading == left_heading or target_heading == red_left_heading and elapsed_time >= 2.9 and color_y_positions[1] == 0 :
                turn_side = 2
                if count >= 3:
                    count = 0
                    round_number +=1
                    print(turn_diside)
                    if round_number == 2:
                        turn_side = 3
                        time_count = 0
                        if target_heading == right_heading:
                            turn_right_diside = False
                        elif target_heading == left_heading:
                            turn_right_diside = True
                        print("red_rotation")
                        start_time = time.time() # Get current time (seconds)
                else:
                    count += 1
                combined_control_signal = 0

        # Red rotation     
        if turn_side == 3 or turn_side == 7:
            if turn_diside:
                print("red_rotation")
                if target_heading == left_heading:
                    target_heading = red_right_heading
                elif target_heading == right_heading:
                    target_heading = red_left_heading
                if turn_side == 3:
                    if color_y_positions[3] > color_y_positions[2] and green_x_diff != 0  and color_y_positions[3] > color_y_positions[0] and  target_heading == red_right_heading or color_y_positions[3] > color_y_positions[2] and green_x_diff != 0  and color_y_positions[3] > color_y_positions[1] and  target_heading == red_left_heading:
                        print("green")
                        if  color_y_positions[3]>120 :
                            turn_right_diside = True
                        combined_control_signal = -pd_control(green, green_x_diff, kp_X, kd_X)
                    elif color_y_positions[2] > color_y_positions[3] and red_x_diff != 0  and color_y_positions[2] > color_y_positions[0] and  target_heading == red_right_heading or color_y_positions[2] > color_y_positions[3] and red_x_diff != 0  and color_y_positions[2] > color_y_positions[1] and  target_heading == red_left_heading:
                        print("red")
                        if  color_y_positions[2]>120 :
                            turn_right_diside = False
                        
                        combined_control_signal = pd_control(red, red_x_diff, kp_X, kd_X)
                    else:
                        combined_control_signal = pd_control(0, heading, kp_heading, kd_heading)
                        print("heading")
                    if color_y_positions[0] > 270 or color_y_positions[1] > 270:
                        turn_side = 7
                        print("turn")
                        if turn_right_diside:
                            combined_control_signal = 180 
                            PWM = -170
                        else:
                            combined_control_signal = -180
                            PWM = 170
                if abs(heading) < 160 :
                    time_count = 0
                    start_time = time.time()
                if abs(heading) > 160:
                    turn_side = 2
                    time_count = 0
                    count = 0
                    if roi_values[0] >= roi_values0 and heading < target_heading[count] + 15 and heading > target_heading[count] - 15 and roi_values[1] < roi_values[0] and abs(target_heading[count]) != 180 or abs(target_heading[count]) == 180 and roi_values[0] >= roi_values0 and heading < -165 and heading > 165 and roi_values[1] < roi_values[0] :
                        print("right")
                        combined_control_signal = pd_control(roi_values0, roi_values[0], kp_roi, kd_roi)
                    elif roi_values[1] >= roi_values1 and heading < target_heading[count] + 15 and heading > target_heading[count] - 15 and roi_values[0] < roi_values[1] and abs(target_heading[count]) != 180 or abs(target_heading[count]) == 180 and roi_values[1] >= roi_values1 and heading < -165 and heading > 165 and roi_values[0] < roi_values[1] : 
                        print("left")
                        combined_control_signal = -pd_control(roi_values1, roi_values[1], kp_roi, kd_roi)
                    elif heading > 0:
                        combined_control_signal = pd_control(0, heading-180, kp_heading, kd_heading)
                        print("heading_rotation ")
                    elif heading < 0:
                        combined_control_signal = pd_control(0, heading+180, kp_heading, kd_heading)
                        print("heading_rotation ")
                    current_time = time.time()
                    elapsed_time = current_time - start_time
                    time_count = time_count + 1  
                    if elapsed_time > 2 and time_count >= 5:
                        turn_side =  2
                    PWM = 40
            else:
                turn_side = 2
                count = 0
 
        # Parking area turn
        if turn_side == 4:
            if not ROI2:
                PWM = 35
                current_time = time.time()
                elapsed_time = current_time - start_time
                if elapsed_time != 0.8 and time_count > 0 or elapsed_time < 0.9:
                    if target_heading == left_heading or target_heading == red_left_heading:
                        if roi_values[0] > 4000:
                            combined_control_signal = -100
                        else:
                            combined_control_signal = 65
                    else:
                        if roi_values[1] > 4000:
                            combined_control_signal = 65
                        else:
                           combined_control_signal = -80
                    time_count = time_count-1
                    time.sleep(0.01)
                else:
                    if target_heading == left_heading or target_heading == red_left_heading:
                        combined_control_signal = pd_control(target_heading[count], heading, kp_heading, kd_heading)
                    else:
                        combined_control_signal = pd_control(target_heading[count], heading, kp_heading, kd_heading)
                if roi_values[2] >= 4000:
                    ROI2 = True
                    time_count = 0
            if ROI2:
                PWM = -40
                if target_heading == left_heading or target_heading == red_left_heading:
                        combined_control_signal = 180
                else:
                        combined_control_signal = -180
                if heading < target_heading[count+1] + 30 and heading > target_heading[count+1] - 30:
                    combined_control_signal = 0
                    turn_side = 5

        # Parking area
        if turn_side == 5:
            PWM = heading
            if target_heading == left_heading and pink_positions[1] > 100 or target_heading == red_left_heading and pink_positions[1] > 80: 
                combined_control_signal = -pd_control(-40, pink_green_x_diff, kp_X, kd_X+0.5)
                print("pink right")
            elif target_heading == right_heading and pink_positions[1] != 0 or target_heading == red_right_heading and pink_positions[1] != 0:
                combined_control_signal = pd_control(200, pink_red_x_diff, kp_X, kd_X+0.5)
                print("pink left")
            else:
                if target_heading == left_heading or target_heading == red_left_heading:
                    print("right")
                    combined_control_signal = pd_control(3000, roi_values[0], kp_roi, kd_roi)
                else:
                    print("left")
                    combined_control_signal = -pd_control(3000, roi_values[1], kp_roi, kd_roi)
                    
                    

        # Limit the control signal to a specific range
        if combined_control_signal > 180:
            combined_control_signal = 180
        if combined_control_signal < -180:
            combined_control_signal = -180
   
        # Prepare data to be sent
        data_to_send = (int(combined_control_signal), int(turn_side), int(PWM))
        
        # Print data to be sent for debugging purposes
        print(f"Sent: {data_to_send}", heading, target_heading,turn_diside,color_y_positions[3],color_y_positions[2],color_y_positions[1],color_y_positions[0],park_side,count,roi_values[0],roi_values[1],roi_values[2],turn_diside)

        # Add packet header "A" to the data packeta
        header = b"A"
        send_data_value = struct.pack('3i', *data_to_send)  # Pack 3 integers
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

                                                                             
