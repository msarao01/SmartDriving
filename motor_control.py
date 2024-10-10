import time
from mqtt import MQTTClient
from machine import Pin, PWM
import network
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect("Tufts_Robot", "")
while wlan.ifconfig()[0] == '0.0.0.0':
    print('.', end=' ')
    time.sleep(1)
# We should have a valid IP now via DHCP
print(wlan.ifconfig())
mqtt_broker = 'broker.hivemq.com'
port = 1883
topic_sub = 'ME35-24/alexmedha'
desired_loc_z = -1
desired_loc_x = 0
found_tag = True
def callback(topic, msg):
    # callback for new mqtt message on topic
    global z_pos, x_pos, found_tag
    # global x_pos
    # decode the message and update position variables
    string = msg.decode()
    if string[1] == ",":       # confirm correct format coming from camera
        other_tag, x, z = string.split(',')
        if int(other_tag) == 1:
            found_tag = True
            z_pos = float(z)
            x_pos = float(x)
            print(z_pos)
            print(x_pos)
        else:
            z_pos = desired_loc_z
            x_pos = desired_loc_x
            found_tag = False
client = MQTTClient('motorcontrol', mqtt_broker , port)
client.connect()
print('Connected to %s MQTT broker' % (mqtt_broker))
client.set_callback(callback)          # set the callback if anything is read
client.subscribe(topic_sub.encode())   # subscribe to a bunch of topics
# Setup PWM control for four pins, two for each motor
pwm2 = PWM(Pin(26))
pwm3 = PWM(Pin(27))
pwm4 = PWM(Pin(4))
pwm5 = PWM(Pin(5))
pwm2.freq(1000)
pwm3.freq(1000)
pwm4.freq(1000)
pwm5.freq(1000)
z_pos = 0
x_pos = 0
# PD controller gains
kp_s = 8.0  # Proportional gain for speed
kd_s = 1.0  # Derivative gain for speed
kp_t = 4.0  # Proportional gain for turning
kd_t = 0.5  # Derivative gain for turning
dead_zone = 6000  # dead zone threshold, tune for motor
# Variables to track previous error for PD controllers
before_speed = 0
before_turn = 0
previous_time = time.ticks_ms()
# PD controller for speed
def pd_controller_speed(error, previous_error, time_diff):
    if time_diff > 0:
        derivative = (error - previous_error) / time_diff
    else:
        derivative = 0
    controller = (kp_s * error) + (kd_s * derivative)
    return controller * 1000
# PD controller for turning
def pd_controller_turn(x_pos, previous_x_pos, time_diff):
    error_turn = x_pos - desired_loc_x
    if time_diff > 0:
        derivative_turn = (x_pos - previous_x_pos) / time_diff
    else:
        derivative_turn = 0
    turn_signal = (kp_t * error_turn) + (kd_t * derivative_turn)
    return turn_signal * 1000.0
# Function to control motors based on speed and turn signals
def control_motors(controller_speed, turn_signal):
    # Calculate PWM for speed
    pwm_val = abs(controller_speed) + dead_zone
    pwm_val = min(65535, int(pwm_val))
    turn_signal = int(turn_signal)
    # Adjust motor PWM values based on turn signal
    if controller_speed > 0:  # Moving forward
        pwm2.duty_u16(0)   # Ensure backward pin is off
        pwm3.duty_u16(pwm_val - turn_signal)  # Apply PWM to right motor
        pwm4.duty_u16(0)   # Ensure backward pin is off
        pwm5.duty_u16(pwm_val + turn_signal)  # Reduce left motor speed
    else:  # Moving backward
        pwm3.duty_u16(0)  # Ensure forward pin is off
        pwm2.duty_u16(pwm_val + turn_signal)  # Reverse left motor
        pwm5.duty_u16(0)  # Ensure forward pin is off
        pwm4.duty_u16(pwm_val - turn_signal)  # Reverse right motor
while True:
    client.check_msg()  # check for new messages
    if found_tag:
        # Time tracking
        current_time = time.ticks_ms()
        time_diff = time.ticks_diff(current_time, previous_time) / 1000.0  # Convert to seconds
        # Speed control based on z_pos
        error_speed = -z_pos - desired_loc_z
        controller_speed = pd_controller_speed(error_speed, before_speed, time_diff)
        # Turn control based on x_pos
        turn_signal = pd_controller_turn(-x_pos, before_turn, time_diff)
        # Update previous errors and time
        before_speed = error_speed
        before_turn = -x_pos
        previous_time = current_time
        print(controller_speed, turn_signal)
        # Control the motors using the control signals
        control_motors(controller_speed, turn_signal)
    else:
        pwm2.duty_u16(0)
        pwm3.duty_u16(0)
        pwm5.duty_u16(0)
        pwm4.duty_u16(0)
    time.sleep_ms(10)
