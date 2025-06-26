from machine import Pin, PWM, I2C, UART, time_pulse_us
import struct
import time

# Set the pins for ultrasonic module 9
TRIG_PIN1 = 8  # Corresponds to the Trig pin of the first ultrasonic module
ECHO_PIN1 = 9  # Corresponds to the Echo pin of the first ultrasonic module
TRIG_PIN2 = 12  # Corresponds to the Trig pin of the second ultrasonic module
ECHO_PIN2 = 13  # Corresponds to the Echo pin of the second ultrasonic module
TRIG_PIN3 = 4  
ECHO_PIN3 = 5

# Initialize ultrasonic sensor pins
trig1 = Pin(TRIG_PIN1, Pin.OUT)
echo1 = Pin(ECHO_PIN1, Pin.IN)
trig2 = Pin(TRIG_PIN2, Pin.OUT)
echo2 = Pin(ECHO_PIN2, Pin.IN)
trig3 = Pin(TRIG_PIN3, Pin.OUT)
echo3 = Pin(ECHO_PIN3, Pin.IN)

servo_pin = PWM(Pin(28), freq=50)  # Initialize the servo pin and set frequency to 50Hz
motor_in1 = Pin(21, Pin.OUT)  # Initialize motor control pins
motor_in2 = Pin(20, Pin.OUT)
button_out = Pin(15, Pin.OUT)  # Initialize button output pin
motor_pwm = PWM(Pin(22), freq=1000)  # Initialize motor PWM with a frequency of 1000Hz
encoder_pin_A = Pin(0, Pin.IN)  # Initialize encoder pin A
encoder_pin_B = Pin(1, Pin.IN)  # Initialize encoder pin B
button = Pin(18, Pin.IN, Pin.PULL_UP)  # Initialize button input with pull-up resistor
data_value = [0] * 3  # Initialize data list
value = [0] * 3
encoder_count = 0  # Initialize encoder count
last_state_A = encoder_pin_A.value()  # Initialize encoder state
jetson_nano_return_last = 0
turn_3 = True
# Configure UART
uart = UART(0, baudrate=115200, tx=Pin(16), rx=Pin(17))

def jetson_nano_return(number):
    global data_value
    HEADER = b"A"  # Define the header
    HEADER_SIZE = len(HEADER)
    DATA_SIZE = 12  # 3 integers, each 4 bytes, total 12 bytes
    TOTAL_SIZE = HEADER_SIZE + DATA_SIZE  # Total size including header

    if uart.any():
        data = uart.read(TOTAL_SIZE)
        
        # Check if the complete data packet is received
        if len(data) == TOTAL_SIZE:
            # Find the header
            header_index = data.find(HEADER)
            if header_index != -1:
                # If the header is found, remove the header and extract data
                start_index = header_index + HEADER_SIZE
                data = data[start_index:] + data[:start_index]
                data_value = struct.unpack('3i', data[:DATA_SIZE])
                return data_value[number]
            else:
                print("Error: Incorrect header received.")
        else:
            print("Error: Incomplete data received.")
    return data_value[number]

def jetson_all():
    global value
    value[0] = jetson_nano_return(0)
    value[1] = jetson_nano_return(1)
    value[2] = jetson_nano_return(2)
    print(value[0], value[1], value[2])

def encoder_interrupt(pin):
    global encoder_count, last_state_A
    state_A = encoder_pin_A.value()
    state_B = encoder_pin_B.value()
    if state_A != last_state_A:
        encoder_count += 1 if state_B != state_A else -1
    last_state_A = state_A

def run_encoder(motor_angle, speed):
    global encoder_count
    encoder_count = 0  # Reset encoder count before running
    while abs(encoder_count) < motor_angle:
        jetson_all()
        combined_control_signal = value[0]
        set_servo_angle(combined_control_signal)
        control_motor(speed)
        print(f"Encoder count: {encoder_count}, combined_control_signal: {combined_control_signal}")
        time.sleep(0.01)
    control_motor(0)

def approach_until(trig, echo, threshold, operator):
    dist = measure_distance(trig, echo)
    while (dist > threshold if operator == '>' else dist < threshold):
        jetson_all()
        dist = measure_distance(trig, echo)
        control_motor(value[2])
        set_servo_angle(value[0])
        time.sleep(0.05)

def run_encoder_Auto(motor_angle, speed, string):
    global encoder_count
    encoder_count = 0  # Reset encoder count before running
    while abs(encoder_count) < motor_angle:
        # Use the provided string as control signal
        combined_control_signal = string
        
        # Adjust the servo using the control signal
        set_servo_angle(combined_control_signal)
        control_motor(speed)
        
        time.sleep(0.01)
    control_motor(0)

# Set up GPIO interrupt for encoder
encoder_pin_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder_interrupt)

# Define motor control function
def control_motor(speed):
    if speed > 0:
        motor_in1.high()
        motor_in2.low()
    elif speed < 0:
        motor_in1.low()
        motor_in2.high()
    else:
        motor_in1.high()
        motor_in2.high()
    motor_pwm.duty_u16(int(abs(speed) * 65535 / 100))  # Set PWM duty cycle

# Set servo angle function
def set_servo_angle(angle):
    min_duty = 1000  # Corresponds to 1ms duty cycle
    max_duty = 2000  # Corresponds to 2ms duty cycle
    duty = int(min_duty + (angle - 50 + 180) * (max_duty - min_duty) / 360)
    duty_u16 = int(duty * 65535 / 20000)
    servo_pin.duty_u16(duty_u16)

# Measure distance using ultrasonic sensor
def measure_distance(trig, echo):
    # Send trigger pulse
    trig.value(0)
    time.sleep_us(2)
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)

    # Read echo pulse width
    duration = time_pulse_us(echo, 1)

    # Calculate distance (speed of sound ~ 343 m/s)
    distance = (duration / 2) * 0.0343

    return distance

try:
    motor_in1.off()
    motor_in2.off()
    set_servo_angle(0)
    button_out.high()
    while button.value() == 1:
        distance1 = measure_distance(trig1, echo1)
        distance2 = measure_distance(trig2, echo2)
        distance3 = measure_distance(trig3, echo3)
        print(jetson_nano_return(0), jetson_nano_return(1), jetson_nano_return(2), distance1,distance2,distance3,abs(encoder_count))
        time.sleep(0.5)
    while value[1] != 5:
        jetson_all()
        control_motor(value[2])
        set_servo_angle(value[0])
        time.sleep(0.05)
        if value[1] == 3 :
            run_encoder_Auto(300, 45,-30)
        if value[1] == 7 :
            run_encoder_Auto(100, 45, value[2])
            while value[1] == 7:
                jetson_all()
                control_motor(40)
                set_servo_angle(value[0])
        if value[1] == 6:
            run_encoder(9000, 50)
            control_motor(-25)
            time.sleep(0.35)
            control_motor(0)
            time.sleep(6)
    distance1 = measure_distance(trig1, echo1)
    distance2 = measure_distance(trig2, echo2)
    distance3 = measure_distance(trig3, echo3)
    jetson_all()
    if distance1 > distance2:  # Right-side parking
        while distance2 > 15:
            jetson_all()
            distance2 = measure_distance(trig2, echo2)
            control_motor(35)
            set_servo_angle(value[0])
        run_encoder(1500, 35)
        while distance2 < 20:
            jetson_all()
            distance2 = measure_distance(trig2, echo2)
            control_motor(35)
            set_servo_angle(value[0])
        run_encoder(1000, 35)
        while distance2 > 20:
            jetson_all()
            distance2 = measure_distance(trig2, echo2)
            control_motor(33)
            set_servo_angle(value[0])
            time.sleep(0.05)
        run_encoder(1000, 35)
        run_encoder(2500, 40)
        # Reverse parking
        while abs(value[2]) <80 or abs(value[2]) >100:
            jetson_all()
            distance2 = measure_distance(trig2, echo2)
            set_servo_angle(180)
            control_motor(-40)  
        set_servo_angle(40)
        distance3 = measure_distance(trig3, echo3)
        while distance3 <23:
            jetson_all()
            distance3 = measure_distance(trig3, echo3)
            control_motor(40)
        distance3 = measure_distance(trig3, echo3)
        while abs(value[2]) > 2 and abs(value[2]) <170 :
            jetson_all()
            distance3 = measure_distance(trig3, echo3)
            set_servo_angle(-180)
            control_motor(-37)
        run_encoder_Auto(3500, 40, -40)   
        control_motor(0)
    else:  # Left-side parking
        while distance1 > 20:
            jetson_all()
            distance1 = measure_distance(trig1, echo1)
            control_motor(35)
            set_servo_angle(value[0])
            time.sleep(0.05)
        run_encoder(1500, 35)
        while distance1 < 21:
            jetson_all()
            distance1 = measure_distance(trig1, echo1)
            control_motor(40)
            set_servo_angle(value[0])
            time.sleep(0.05)
        run_encoder(1000, 35) 
        while distance1 > 20:
            jetson_all()
            distance1 = measure_distance(trig1, echo1)
            control_motor(33)
            set_servo_angle(value[0])
            time.sleep(0.05)
        run_encoder(1650,35)
        run_encoder(500,40)
        control_motor(0)
        while abs(value[2]) <80 or abs(value[2]) >100:
            jetson_all()
            distance2 = measure_distance(trig2, echo2)
            set_servo_angle(-180)
            control_motor(-35)
            time.sleep(0.05)  
        set_servo_angle(-20)
        distance3 = measure_distance(trig3, echo3)
        while distance3 <20:
            jetson_all()
            distance3 = measure_distance(trig3, echo3)
            control_motor(40)
        distance3 = measure_distance(trig3, echo3)
        while abs(value[2]) > 3 and abs(value[2]) <170:
            jetson_all()
            distance3 = measure_distance(trig3, echo3)
            set_servo_angle(180)
            control_motor(-38)
        run_encoder_Auto(3000, 40, -20)   
        control_motor(0)
    control_motor(-25)
    time.sleep(0.35)
    control_motor(0)
    button_out.low()

except KeyboardInterrupt:
    button_out.low()
    motor_in1.off()
    motor_in2.off()
    set_servo_angle(0)
    print("Program interrupted")











