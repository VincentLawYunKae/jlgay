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
        

# main function to control the robot wheels
def move_robot():
    global use_pid, left_speed, right_speed, turn_motion_queue, motion
    flag_new_pid_cycle = True
    while True:
        if (motion == 'turning' or turn_motion_queue):
            if mode == 0:
                try:
                    left_speed, right_speed = turn_motion_queue.pop(0)
                except:
                    motion = 'stop'
                    
                # turn right
                if left_speed > right_speed:
                    left_speed, right_speed = abs(left_speed), abs(right_speed)
                    set_point = (left_encoder.value + right_encoder.value) / 2
                    pid_left = PID(kp_turn, ki_turn, kd_turn, setpoint=set_point, output_limits=(0.65,0.85), starting_output=abs(left_speed))
                    pid_right = PID(kp_turn, ki_turn, kd_turn, setpoint=set_point, output_limits=(0.65,0.85), starting_output=abs(right_speed))
                    start_time = time.time()
                    while (time.time() - start_time) < right_dt:
                        left_speed = pid_left(left_encoder.value)
                        right_speed = pid_right(right_encoder.value)
                        pibot.value = (left_speed, -right_speed)
                    pibot.value = (0, 0)
                    time.sleep(0.18)     # this is for the rotation to settling down
                    
                # turn left
                else:
                    left_speed, right_speed = abs(left_speed), abs(right_speed)
                    set_point = (left_encoder.value + right_encoder.value) / 2
                    pid_left = PID(kp_turn, ki_turn, kd_turn, setpoint=set_point, output_limits=(0.65,0.85), starting_output=abs(left_speed))
                    pid_right = PID(kp_turn, ki_turn, kd_turn, setpoint=set_point, output_limits=(0.65,0.85), starting_output=abs(right_speed))
                    start_time = time.time()
                    while (time.time() - start_time) < left_dt:
                        left_speed = pid_left(left_encoder.value)
                        right_speed = pid_right(right_encoder.value)
                        pibot.value = (-left_speed, right_speed)
                    pibot.value = (0, 0) 
                    time.sleep(0.18)
                
                # try to reset the pid for the linera motion after handling the turn and stop
                flag_new_pid_cycle = True
                left_encoder.reset()
                right_encoder.reset()   
                
            elif mode == 1:
                pibot.value = (left_speed, right_speed) 
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
                pid_right = PID(kp, ki, kd, setpoint=left_encoder.value, output_limits=(0,1), starting_output=right_speed)
                flag_new_pid_cycle = False
            pid_right.setpoint = left_encoder.value
            right_speed = pid_right(right_encoder.value)
            if motion == 'forward': pibot.value = (left_speed, right_speed)
            else: pibot.value = (-left_speed, -right_speed)
            # print('Value', left_encoder.value, right_encoder.value)
            # print('Speed', left_speed, right_speed)
        time.sleep(0.005)
    
    
# Receive confirmation whether to use pid or not to control the wheels (forward & backward)
@app.route('/pid')
def set_pid():
    global use_pid, kp, ki, kd
    use_pid = int(request.args.get('use_pid'))
    if use_pid:
        kp, ki, kd = float(request.args.get('kp')), float(request.args.get('ki')), float(request.args.get('kd'))
        return "Using PID"
    else:
        return "Not using PID"

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
        if mode == 0:
            global turn_motion_queue
            turn_motion_queue.append((left_speed, right_speed))
        motion = 'turning'
    elif (left_speed > 0 and right_speed > 0):
        motion = 'forward'
    elif (left_speed < 0 and right_speed < 0):
        motion = 'backward'
    return motion
    

@app.route('/dt')
def set_dt():
    global left_dt, right_dt
    left_dt, right_dt = float(request.args.get('left_dt')), float(request.args.get('right_dt'))
    return str(left_dt) + " " + str(right_dt)

@app.route('/mode')
def change_mode():
    global mode
    mode = 0 if mode == 1 else 1
    return str(mode)
    
    
# Constants
in1 = 17 # may have to change this
in2 = 27 # may have to change this
ena = 18
in3 = 23 # may have to change this
in4 = 24 # may have to change this
enb = 25
enc_a = 26
enc_b = 16
mode = 0

# Initialize robot and encoders
pibot = Robot(right=Motor(forward=in1, backward=in2, enable=ena), left=Motor(forward=in3, backward=in4, enable=enb))

# Initialize PID controllers for wheels
left_encoder = Encoder(enc_a)
right_encoder = Encoder(enc_b)
use_pid = 0
kp = 0
ki = 0
kd = 0
left_speed, right_speed = 0, 0
motion = ''
left_dt, right_dt = 0.041803093477052005, 0.045203093477052005
kp_turn, ki_turn, kd_turn = 0.1, 0.005, 0.001
turn_motion_queue = []

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
        move_robot()
except KeyboardInterrupt:
    pibot.stop()
    picam2.stop()
    print("Program interrupted by user.")