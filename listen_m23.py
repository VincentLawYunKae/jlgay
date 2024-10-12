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
    for turning in continuous mode
    """
    global use_pid, left_speed, right_speed
    flag_new_pid_cycle = True
    while True:
        ### if not using pid, just move the wheels as commanded
        if not use_pid:
            pibot.value = (left_speed, right_speed)          
            # print('Value', left_encoder.value, right_encoder.value)
        ### with pid, left wheel is set as reference, and right wheel will try to match the encoder counter of left wheel
        ### pid only runs when robot moves forward or backward. Turning does not use pid
        else:
            if (motion == 'stop') or (motion == 'turning'):
                pibot.value = (left_speed, right_speed) 
                left_encoder.reset()
                right_encoder.reset()
                flag_new_pid_cycle = True          
            else:
                left_speed, right_speed = abs(left_speed), abs(right_speed)
                if flag_new_pid_cycle:
                    pid_right = PID(kp_lin, ki_lin, kd_lin, setpoint=left_encoder.value, output_limits=(0,1), starting_output=right_speed)
                    flag_new_pid_cycle = False
                pid_right.setpoint = left_encoder.value
                right_speed = pid_right(right_encoder.value)
                if motion == 'forward': pibot.value = (left_speed, right_speed)
                else: pibot.value = (-left_speed, -right_speed)
                print('Value', left_encoder.value, right_encoder.value)
        time.sleep(0.005)
        if drive_mode == 0:
            break


def handle_mode0():
    """
    for turning in tick mock
    """
    global use_pid, left_speed, right_speed, motion_queue
    flag_new_pid_cycle = True
    while True:
        if motion_queue:
            print("Enter here to turn the motor", motion_queue)
            motion, dt = motion_queue.pop(0)
            if motion == "turn left":
                set_point = (left_encoder.value + right_encoder.value) / 2
                pid_left = PID(kp_turn, ki_turn, kd_turn, setpoint=set_point, output_limits=(0.65,0.85), starting_output=turn_speed)
                pid_right = PID(kp_turn, ki_turn, kd_turn, setpoint=set_point, output_limits=(0.65,0.85), starting_output=turn_speed)
                start_time = time.time()
                while (time.time() - start_time) < dt:
                    left_speed = pid_left(left_encoder.value)
                    right_speed = pid_right(right_encoder.value)
                    pibot.value = (-left_speed, right_speed)
                pibot.value = (0, 0)
            elif motion == "turn right":
                set_point = (left_encoder.value + right_encoder.value) / 2
                pid_left = PID(kp_turn, ki_turn, kd_turn, setpoint=set_point, output_limits=(0.65,0.85), starting_output=turn_speed)
                pid_right = PID(kp_turn, ki_turn, kd_turn, setpoint=set_point, output_limits=(0.65,0.85), starting_output=turn_speed)
                start_time = time.time()
                while (time.time() - start_time) < dt:
                    left_speed = pid_left(left_encoder.value)
                    right_speed = pid_right(right_encoder.value)
                    pibot.value = (left_speed, -right_speed)
                pibot.value = (0, 0)
            left_encoder.reset()
            right_encoder.reset()
            flag_new_pid_cycle = True          
            
        elif (motion == 'stop'):
            pibot.value = (0, 0)
            # try to reset the pid for the linera motion after handling the turn and stop
            flag_new_pid_cycle = True
            left_encoder.reset()
            right_encoder.reset() 
            
        else:
            # linear motion
            left_speed, right_speed = abs(left_speed), abs(right_speed)
            if flag_new_pid_cycle:
                pid_right = PID(kp_lin, ki_lin, kd_lin, setpoint=left_encoder.value, output_limits=(0,1), starting_output=right_speed)
                flag_new_pid_cycle = False
            pid_right.setpoint = left_encoder.value
            right_speed = pid_right(right_encoder.value)
            if motion == 'forward': pibot.value = (left_speed, right_speed)
            else: pibot.value = (-left_speed, -right_speed)
            # print('Value', left_encoder.value, right_encoder.value)
            
        time.sleep(0.005)
        if drive_mode == 1:
            break

# main function to control the robot wheels
def move_robot():
    # print("mode", drive_mode)
    if drive_mode == 0:
        handle_mode0()
    else:
        # print("motion", motion)
        handle_mode1()
    
@app.route('/turnpid')
def set_turnpid():
    global kp_turn, ki_turn, kd_turn
    kp_turn, ki_turn, kd_turn = float(request.args.get('kp')), float(request.args.get('ki')), float(request.args.get('kd'))
    print("Setting Turn PID to ", kp_turn, ki_turn, kd_turn)
    return "Setting Turn PID"
    
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
    global motion, motion_queue
    dt, motion = float(request.args.get('dt')), request.args.get('motion')
    motion_queue.append((motion, dt))
    print("The motion now is", motion)
    return motion

@app.route('/mode')
def change_mode():
    global drive_mode
    drive_mode = 0 if drive_mode == 1 else 1
    return str(drive_mode)
    
@app.route('/pid')
def set_pid():
    global use_pid, kp, ki, kd
    use_pid = int(request.args.get('use_pid'))
    if use_pid:
        kp, ki, kd = float(request.args.get('kp')), float(request.args.get('ki')), float(request.args.get('kd'))
        return "Using PID"
    else:
        return "Not using PID"
    
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

left_speed = 0
right_speed = 0

turn_speed = 0.75
motion = ''
drive_mode = 0
motion_queue = []

# Initialize the PiCamera
picam2 = Picamera2()
config = picam2.create_preview_configuration(lores={"size": (640,480)})
picam2.configure(config)
picam2.start()
time.sleep(2)

# Initialize flask
def run_flask():
    app.run(host='0.0.0.0', port=5000)
    
flask_thread = threading.Thread(target=run_flask)
flask_thread.daemon = True
flask_thread.start()

try:
    while True:
        print("mode", drive_mode)
        if drive_mode == 0:
            handle_mode0()
        else:
            handle_mode1()
except KeyboardInterrupt:
    pibot.stop()
    picam2.stop()
    print("Program interrupted by user.")