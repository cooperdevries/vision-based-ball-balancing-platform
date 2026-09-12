import cv2
import numpy as np
import serial
import time
import math

# Serial and camera setup
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
CAMERA_INDEX = 0
FLIP_FRAME = True
TARGET_X_OFFSET_PX = 0
TARGET_Y_OFFSET_PX = 0

# Ball detection
LOWER_WHITE = np.array([0, 0, 100], dtype=np.uint8)
UPPER_WHITE = np.array([179, 100, 255], dtype=np.uint8)
MIN_BALL_AREA = 500
BLUR_SIZE = 9
MORPH_KERNEL_SIZE = 5

# Axis mapping
SWAP_AXES = True
X_DIRECTION = -1.0
Y_DIRECTION = -1.0

# PID tuning
KP_X = 6.0
KI_X = 0.0
KD_X = 3.5
KP_Y = 6.0
KI_Y = 0.0
KD_Y = 3.5

# Limits and filtering
MAX_TILT_X_DEG = 18.0
MAX_TILT_Y_DEG = 18.0
ERROR_DEADZONE = 0.06
POSITION_FILTER_ALPHA = 0.45
DERIVATIVE_FILTER_ALPHA = 0.6
INTEGRAL_LIMIT = 0.4
CONTROL_PERIOD_SEC = 0.03
LOST_BALL_RECENTER_SEC = 9999

def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))

def apply_deadzone(error, deadzone):
    if abs(error) <= deadzone:
        return 0.0
    scaled = (abs(error) - deadzone) / (1.0 - deadzone)
    return math.copysign(scaled, error)

class PIDController:

    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_derivative_error = 0.0
        self.filtered_derivative = 0.0
        self.previous_time = None

    def update(self, proportional_error, derivative_error, now):
        if self.previous_time is None:
            self.previous_time = now
            self.previous_derivative_error = derivative_error
            return self.kp * proportional_error
        dt = clamp(now - self.previous_time, 0.005, 0.1)
        self.integral += proportional_error * dt
        self.integral = clamp(self.integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)
        # Use raw error here so derivative braking still works near the center.
        raw_derivative = (derivative_error - self.previous_derivative_error) / dt
        self.filtered_derivative = DERIVATIVE_FILTER_ALPHA * raw_derivative + (1.0 - DERIVATIVE_FILTER_ALPHA) * self.filtered_derivative
        output = self.kp * proportional_error + self.ki * self.integral + self.kd * self.filtered_derivative
        self.previous_derivative_error = derivative_error
        self.previous_time = now
        return output

def detect_ball(frame):
    blurred = cv2.GaussianBlur(frame, (BLUR_SIZE, BLUR_SIZE), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_WHITE, UPPER_WHITE)
    kernel = np.ones((MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return (None, mask)
    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < MIN_BALL_AREA:
        return (None, mask)
    moments = cv2.moments(contour)
    if moments['m00'] == 0:
        return (None, mask)
    x = int(moments['m10'] / moments['m00'])
    y = int(moments['m01'] / moments['m00'])
    (_, _), radius = cv2.minEnclosingCircle(contour)
    return ({'x': x, 'y': y, 'radius': int(radius), 'area': area}, mask)

def send_offsets(arduino, x_degrees, y_degrees):
    message = f'{x_degrees:.3f},{y_degrees:.3f}\n'
    arduino.write(message.encode('ascii'))

def main():
    arduino = None
    camera = None
    try:
        print(f'Opening Arduino on {SERIAL_PORT}...')
        arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05, write_timeout=0.05)
        time.sleep(2.0)
        arduino.reset_input_buffer()
        send_offsets(arduino, 0.0, 0.0)
        camera = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
        if not camera.isOpened():
            camera.release()
            camera = cv2.VideoCapture(CAMERA_INDEX)
        if not camera.isOpened():
            raise RuntimeError(f'Could not open camera index {CAMERA_INDEX}.')
        pid_x = PIDController(KP_X, KI_X, KD_X)
        pid_y = PIDController(KP_Y, KI_Y, KD_Y)
        filtered_x = None
        filtered_y = None
        last_control_time = 0.0
        last_ball_time = 0.0
        x_command = 0.0
        y_command = 0.0
        paused = False
        print('Press Q to quit. Press SPACE to pause and center.')
        while True:
            success, frame = camera.read()
            if not success:
                raise RuntimeError('Camera stopped returning frames.')
            if FLIP_FRAME:
                frame = cv2.flip(frame, 1)
            height, width = frame.shape[:2]
            target_x = width // 2 + TARGET_X_OFFSET_PX
            target_y = height // 2 + TARGET_Y_OFFSET_PX
            ball, mask = detect_ball(frame)
            now = time.monotonic()
            if ball is not None:
                last_ball_time = now
                if filtered_x is None:
                    filtered_x = float(ball['x'])
                    filtered_y = float(ball['y'])
                else:
                    filtered_x = POSITION_FILTER_ALPHA * ball['x'] + (1.0 - POSITION_FILTER_ALPHA) * filtered_x
                    filtered_y = POSITION_FILTER_ALPHA * ball['y'] + (1.0 - POSITION_FILTER_ALPHA) * filtered_y
                raw_error_x = (filtered_x - target_x) / (width / 2.0)
                raw_error_y = (filtered_y - target_y) / (height / 2.0)
                raw_error_x = clamp(raw_error_x, -1.0, 1.0)
                raw_error_y = clamp(raw_error_y, -1.0, 1.0)
                p_error_x = apply_deadzone(raw_error_x, ERROR_DEADZONE)
                p_error_y = apply_deadzone(raw_error_y, ERROR_DEADZONE)
                if SWAP_AXES:
                    servo_x_p_error = p_error_y
                    servo_y_p_error = p_error_x
                    servo_x_d_error = raw_error_y
                    servo_y_d_error = raw_error_x
                else:
                    servo_x_p_error = p_error_x
                    servo_y_p_error = p_error_y
                    servo_x_d_error = raw_error_x
                    servo_y_d_error = raw_error_y
                if not paused and now - last_control_time >= CONTROL_PERIOD_SEC:
                    x_command = X_DIRECTION * pid_x.update(servo_x_p_error, servo_x_d_error, now)
                    y_command = Y_DIRECTION * pid_y.update(servo_y_p_error, servo_y_d_error, now)
                    x_command = clamp(x_command, -MAX_TILT_X_DEG, MAX_TILT_X_DEG)
                    y_command = clamp(y_command, -MAX_TILT_Y_DEG, MAX_TILT_Y_DEG)
                    send_offsets(arduino, x_command, y_command)
                    last_control_time = now
                cv2.circle(frame, (int(filtered_x), int(filtered_y)), max(ball['radius'], 5), (0, 255, 0), 2)
                cv2.line(frame, (int(filtered_x), int(filtered_y)), (target_x, target_y), (0, 0, 255), 2)
                status = f'Err X:{raw_error_x:+.3f} Y:{raw_error_y:+.3f}  Cmd X:{x_command:+.2f} Y:{y_command:+.2f} deg'
                cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            else:
                # Reset tracking only after the configured lost-ball timeout.
                if now - last_ball_time >= LOST_BALL_RECENTER_SEC:
                    filtered_x = None
                    filtered_y = None
                    pid_x.reset()
                    pid_y.reset()
                    x_command = 0.0
                    y_command = 0.0
                    if now - last_control_time >= CONTROL_PERIOD_SEC:
                        send_offsets(arduino, 0.0, 0.0)
                        last_control_time = now
                cv2.putText(frame, 'BALL NOT FOUND', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            if paused:
                x_command = 0.0
                y_command = 0.0
                if now - last_control_time >= CONTROL_PERIOD_SEC:
                    send_offsets(arduino, 0.0, 0.0)
                    last_control_time = now
                cv2.putText(frame, 'PAUSED - SERVOS CENTERED', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
            cv2.drawMarker(frame, (target_x, target_y), (255, 0, 0), markerType=cv2.MARKER_CROSS, markerSize=24, thickness=2)
            cv2.imshow('Balancing Ball Controller', frame)
            cv2.imshow('Ball Mask', mask)
            key = cv2.waitKey(1) & 255
            if key == ord('q'):
                break
            if key == 32:
                paused = not paused
                pid_x.reset()
                pid_y.reset()
                send_offsets(arduino, 0.0, 0.0)
    except serial.SerialException as error:
        print(f'Serial error: {error}')
        print('Check SERIAL_PORT and close Arduino Serial Monitor.')
    except Exception as error:
        print(f'Error: {error}')
    finally:
        if arduino is not None and arduino.is_open:
            try:
                send_offsets(arduino, 0.0, 0.0)
                time.sleep(0.15)
            except Exception:
                pass
            arduino.close()
        if camera is not None:
            camera.release()
        cv2.destroyAllWindows()
        print('Stopped. Center command sent.')
if __name__ == '__main__':
    main()
