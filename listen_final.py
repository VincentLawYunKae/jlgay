from simple_pid import PID
from picamera2 import Picamera2
from flask import Flask, Response, request
from gpiozero import Robot, Motor, DigitalInputDevice
import io
import time
import threading


app = Flask(__name__)


class Encoder(object):
    def __init__(self, pin):
        self._value = 0
        self.encoder = DigitalInputDevice(pin)
        self.encoder.when_activated = self._increment
        self.encoder.when_deactivated = self._increment
    
    def reset(self):
        self._value = 0
    
    def _increment(self):
        self._value += 1
        
    @property
    def value(self):
        return self._value


def handle_mode1():
    """
    for waypoint navigation
    """
    global motion_queue, kp_lin, ki_lin, kd_lin, kp_turn, ki_turn, kd_turn, turn_tolerance, linear_tolerance
    global kp_lin_right, ki_lin_right,kd_lin_right, kp_lin_left, ki_lin_left, kd_lin_left, dt_left, dt_right
    while True:
        # print("motion", motion)
        try:
            motion_elem = motion_queue.pop(0)
            if len(motion_elem) == 2:
                motion, dt = motion_elem
            elif len(motion_elem) == 3:
                motion, left_disp, right_disp = motion_elem
            left_encoder.reset()
            right_encoder.reset()
        except:
            motion = "stop"
        finally:
            counter = 0
            if motion == "forward":
                pid_right =  PID(kp_lin_right, ki_lin_right, kd_lin_right, setpoint=right_disp, output_limits=(0.395,0.448), starting_output=linear_speed-0.1*linear_speed)
                # pid_left = PID(kp_lin_left, ki_lin_left, kd_lin_left, setpoint=left_disp, output_limits=(0.28,0.52), starting_output=linear_speed+0.1*linear_speed)
                while (left_encoder.value < abs(left_disp) - linear_tolerance) and (right_encoder.value < abs(right_disp) - linear_tolerance):
                    # pid_left.setpoint = max(left_encoder.value, (right_encoder.value+left_encoder.value)/2)
                    pid_right.setpoint = left_encoder.value
                    # print(f"Setpoint: {left_encoder.value}, {right_encoder.value}")
                    right_speed = pid_right(right_encoder.value)
                    # left_speed = pid_left(left_encoder.value)
                    if counter < 50:
                        right_speed = linear_speed - 0.35*linear_speed
                        counter += 1
                    elif counter < 80:
                        right_speed = linear_speed - 0.25*linear_speed
                        counter += 1
                    elif counter < 120:
                        right_speed = linear_speed - 0.15*linear_speed
                        counter += 1
                    else:
                        right_speed = right_speed
                    print(f"Speed: {linear_speed}, {right_speed}")
                    pibot.value = (linear_speed, right_speed)
                    print(f"Encoder: {left_encoder.value}, {right_encoder.value}")
                    # print(f"Speed: {left_speed}, {right_speed}")
                    # pibot.value = (left_speed, right_speed)
                pibot.value = (0, 0)
            elif motion == "backward":
                pid_left = PID(kp_lin, ki_lin, kd_lin, setpoint=left_disp, output_limits=(0.48,0.52), starting_output=linear_speed)
                pid_right =  PID(kp_lin, ki_lin, kd_lin, setpoint=right_disp, output_limits=(0.48,0.52), starting_output=linear_speed)
                while (left_encoder.value < abs(left_disp) - linear_tolerance) and (right_encoder.value < abs(right_disp) - linear_tolerance):
                    pid_left.setpoint = max(left_encoder.value, (right_encoder.value+left_encoder.value)/2)
                    pid_right.setpoint = max(right_encoder.value, (right_encoder.value+left_encoder.value)/2)
                    print(f"Setpoint: {pid_left.setpoint}, {pid_right.setpoint}")
                    right_speed = pid_right(right_encoder.value)
                    left_speed = pid_left(left_encoder.value)
                    print(f"Speed: {left_speed}, {right_speed}")
                    pibot.value = (-left_speed, -right_speed)
                pibot.value = (0, 0)
            elif motion == "turn left":
                # set_point = (left_encoder.value + right_encoder.value) / 2
                pid_left = PID(kp_turn, ki_turn, kd_turn, setpoint=1, output_limits=(0.625,0.665), starting_output=turn_speed)
                pid_right = PID(kp_turn, ki_turn, kd_turn, setpoint=1, output_limits=(0.625,0.665), starting_output=turn_speed)
                start_time = time.time()
                print(f"Turn left with dt: {dt_left}")
                while (time.time() - start_time) < dt_left:
                    pid_left.setpoint = max(left_encoder.value, (right_encoder.value+left_encoder.value)/2)
                    pid_right.setpoint = max(right_encoder.value, (right_encoder.value+left_encoder.value)/2)
                    left_speed = pid_left(left_encoder.value)
                    right_speed = pid_right(right_encoder.value)
                    pibot.value = (-left_speed, right_speed)
                pibot.value = (0, 0)
            elif motion == "turn right":
                # set_point = (left_encoder.value + right_encoder.value) / 2
                pid_left = PID(kp_turn, ki_turn, kd_turn, setpoint=1, output_limits=(0.625,0.665), starting_output=turn_speed)
                pid_right = PID(kp_turn, ki_turn, kd_turn, setpoint=1, output_limits=(0.625,0.665), starting_output=turn_speed)
                start_time = time.time()
                print(f"Turn right with dt: {dt_right}")
                while (time.time() - start_time) < dt_right:
                    pid_left.setpoint = max(left_encoder.value, (right_encoder.value+left_encoder.value)/2)
                    pid_right.setpoint = max(right_encoder.value, (right_encoder.value+left_encoder.value)/2)
                    left_speed = pid_left(left_encoder.value)
                    right_speed = pid_right(right_encoder.value)
                    pibot.value = (left_speed, -right_speed)
                pibot.value = (0, 0)
            if motion != "stop":
                print('Value', left_encoder.value, right_encoder.value)
        time.sleep(0.05)
        if drive_mode == 0:
            break
    
@app.route('/linearpid')
def set_linearpid():
    global kp_lin, ki_lin, kd_lin
    kp_lin, ki_lin, kd_lin = float(request.args.get('kp')), float(request.args.get('ki')), float(request.args.get('kd'))
    print("Setting Linear PID to ", kp_lin, ki_lin, kd_lin)
    return "Setting Linear PID"

@app.route('/linpidleft')
def set_linpidleft():
    global kp_lin_left, ki_lin_left, kd_lin_left
    kp_lin_left, ki_lin_left, kd_lin_left = float(request.args.get('kp')), float(request.args.get('ki')), float(request.args.get('kd'))
    print("Setting Linear LEFT PID to ", kp_lin_left, ki_lin_left, kd_lin_left)
    return "Setting Linear LEFT PID"


@app.route('/linpidright')
def set_linpidright():
    global kp_lin_right, ki_lin_right, kd_lin_right
    kp_lin_right, ki_lin_right, kd_lin_right = float(request.args.get('kp')), float(request.args.get('ki')), float(request.args.get('kd'))
    print("Setting Linear RIGHT PID to ", kp_lin_right, ki_lin_right, kd_lin_right)
    return "Setting Linear RIGHT PID"


@app.route('/dt')
def set_dt():
    global dt_left, dt_right
    dt_left = float(request.args.get('dt_left')) # Default to 0 if not provided
    dt_right = float(request.args.get('dt_right')) 
    print(f"Reach here to set dt_left as {dt_left} and dt_right as {dt_right}")
    return str(dt_left) + " " + str(dt_right)
    
@app.route('/turnpid')
def set_turnpid():
    global kp_turn, ki_turn, kd_turn
    kp_turn, ki_turn, kd_turn = float(request.args.get('kp')), float(request.args.get('ki')), float(request.args.get('kd'))
    print("Setting Turn PID to ", kp_turn, ki_turn, kd_turn)
    return "Setting Turn PID"

@app.route('/lineartolerance')
def set_lineartolerance():
    global linear_tolerance
    linear_tolerance = int(request.args.get('tolerance'))
    print("Setting Linar tolerance to ", linear_tolerance)
    return "Setting Linear tolerance"
    
# Receive a request to capture and send a snapshot of the picamera
@app.route('/image')
def capture_image():
    stream = io.BytesIO()
    picam2.capture_file(stream, format='jpeg')
    stream.seek(0)
    return Response(stream, mimetype='image/jpeg')
    

 # Receive command to move the pibot
@app.route('/move')
def move():
    global left_speed, right_speed, motion
    left_speed, right_speed = float(request.args.get('left_speed')), float(request.args.get('right_speed'))
    if (left_speed == 0 and right_speed == 0):
        motion = 'stop'
    elif (left_speed != right_speed ):
        motion = 'turning'
    elif (left_speed > 0 and right_speed > 0):
        motion = 'forward'
    elif (left_speed < 0 and right_speed < 0):
        motion = 'backward'
    return motion

@app.route('/angle')
def set_angle():
    global motion, motion_queue, dt_right, dt_left
    dt, motion = float(request.args.get('dt')), request.args.get('motion')
    dt = dt_right if motion == 'turn right' else dt_left
    print(dt)
    motion_queue.append((motion, dt))
    print("The motion now is", motion)
    return motion

@app.route('/disp')
def set_disp():
    global left_disp, right_disp, motion, motion_queue
    left_disp, right_disp = int(request.args.get('left_disp')), int(request.args.get('right_disp'))
    if (left_disp == 0 and right_disp == 0):
        motion = 'stop'
    elif (left_disp > 0 and right_disp > 0):
        motion = 'forward'
    elif (left_disp < 0 and right_disp < 0):
        motion = 'backward'
    if motion != 'stop':
        motion_queue.append((motion, left_disp, right_disp))
    print("The motion now is", motion)
    return motion

@app.route('/mode')
def set_mode():
    global drive_mode 
    drive_mode = int(request.args.get('mode'))
    print(drive_mode)
    return str(drive_mode)
    

# Constants
in1 = 17 # may have to change this
in2 = 27 # may have to change this
ena = 18
in3 = 23 # may have to change this
in4 = 24 # may have to change this
enb = 25
enc_a = 26
enc_b = 16

# Initialize robot and encoders
pibot = Robot(right=Motor(forward=in1, backward=in2, enable=ena), left=Motor(forward=in3, backward=in4, enable=enb))

# Initialize PID controllers for wheels
left_encoder = Encoder(enc_a)
right_encoder = Encoder(enc_b)
use_pid = 0

kp_turn= 0.1
ki_turn = 0.005
kd_turn = 0.001

kp_lin = 2
ki_lin = 0.05
kd_lin = 0.01

kp_lin_left = 0.6
ki_lin_left = 0.0001
kd_lin_left = 0.0005
kp_lin_right = 0.6
ki_lin_right = 0.0001
kd_lin_right = 0.0005

left_speed = 0
right_speed = 0
linear_speed = 0.425
turn_speed = 0.65
turn_tolerance = 3
linear_tolerance = 6
motion = ''
drive_mode = 1
motion_queue = []

# Initialize the PiCamera
picam2 = Picamera2()
config = picam2.create_preview_configuration(lores={"size": (640,480)})
picam2.configure(config)
picam2.start()
time.sleep(2)

dt_left = 0
dt_right = 0

# Initialize flask
def run_flask():
    app.run(host='0.0.0.0', port=5000)
    
flask_thread = threading.Thread(target=run_flask)
flask_thread.daemon = True
flask_thread.start()

try:
    while True:
        handle_mode1()
        
except KeyboardInterrupt:
    pibot.stop()
    picam2.stop()
    print("Program interrupted by user.")