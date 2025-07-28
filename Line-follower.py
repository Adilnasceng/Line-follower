import RPi.GPIO as GPIO
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import time

# ---------------- Motor ve GPIO Ayarları ----------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

rpwm1, lpwm1 = 26, 6
rpwm2, lpwm2 = 5, 19

GPIO.setup(rpwm1, GPIO.OUT)
GPIO.setup(lpwm1, GPIO.OUT)
GPIO.setup(rpwm2, GPIO.OUT)
GPIO.setup(lpwm2, GPIO.OUT)

right_motor_forward = GPIO.PWM(rpwm1, 1000)
right_motor_backward = GPIO.PWM(lpwm1, 1000)
left_motor_forward = GPIO.PWM(rpwm2, 1000)
left_motor_backward = GPIO.PWM(lpwm2, 1000)

right_motor_forward.start(0)
right_motor_backward.start(0)
left_motor_forward.start(0)
left_motor_backward.start(0)

# ---------------- PID Parametreleri ----------------
Kp, Ki, Kd = 0.35, 0.001, 0.09
K_ang = 0.3
previous_error = 0
integral = 0

# Yol ayrımı algılama eşiği
YOL_AYRIMI_ESIGI = 230  # Yol ayrımı genişlik eşiği
intersection_count = 0  # Kaçıncı intersection olduğunu takip etmek için
rota = {1: "düz", 2: "sol", 3: "sol", 4: "dur"}  # Intersection yönleri

def set_motor_direction(control, intersection_detected):
    """Motor yönünü ve hızını PID kontrol ile ayarla"""
    global intersection_count
    control = max(min(control, 50), -50)

    if intersection_detected:
        intersection_count += 1
        print(f"Intersection {intersection_count} algılandı!")

        if intersection_count in rota:
            yon = rota[intersection_count]
            print(f"Intersection {intersection_count}: {yon} yönüne dönülüyor")
            if yon == "sol":
                right_motor_forward.ChangeDutyCycle(40)
                left_motor_forward.ChangeDutyCycle(0)
            elif yon == "sağ":
                right_motor_forward.ChangeDutyCycle(0)
                left_motor_forward.ChangeDutyCycle(40)
            elif yon == "düz":
                right_motor_forward.ChangeDutyCycle(30)
                left_motor_forward.ChangeDutyCycle(30)
            elif yon == "dur":
                print("Rota tamamlandı, robot duruyor.")
                right_motor_forward.ChangeDutyCycle(0)
                left_motor_forward.ChangeDutyCycle(0)
                GPIO.cleanup()
                exit()
            time.sleep(0.9)  # Yarım saniye dönüş
        
        right_motor_forward.ChangeDutyCycle(0)
        left_motor_forward.ChangeDutyCycle(0)
        time.sleep(0.5)

    elif control < -20:
        right_motor_forward.ChangeDutyCycle(abs(control))
        left_motor_forward.ChangeDutyCycle(0)
    elif control > 20:
        right_motor_forward.ChangeDutyCycle(0)
        left_motor_forward.ChangeDutyCycle(abs(control))
    else:
        right_motor_forward.ChangeDutyCycle(20)
        left_motor_forward.ChangeDutyCycle(20)

# ---------------- Kamera Ayarları ----------------
camera = PiCamera()
camera.resolution = (320, 208)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(320, 208))

x_last, y_last = 160, 104

# ---------------- Ana Döngü ----------------
try:
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 90, 255, cv2.THRESH_BINARY_INV)

        Blackline = cv2.inRange(image, (0, 0, 0), (90, 90, 90))
        kernel = np.ones((3, 3), np.uint8)
        Blackline = cv2.erode(Blackline, kernel, iterations=9)
        Blackline = cv2.dilate(Blackline, kernel, iterations=10)

        contours_blk, _ = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_blk_len = len(contours_blk)
        cv2.drawContours(image, contours_blk, -1, (0, 255, 0), 3)

        intersection_detected = False

        if contours_blk_len > 0:
            largest_contour = max(contours_blk, key=cv2.contourArea)
            blackbox = cv2.minAreaRect(largest_contour)
            (x_min, y_min), (h_min, w_min), ang = blackbox
            x_last = x_min
            y_last = y_min

            if ang < -45:
                ang = 90 + ang
            if w_min < h_min and ang > 0:
                ang = (90 - ang) * -1
            if w_min > h_min and ang < 0:
                ang = 90 + ang

            if w_min > YOL_AYRIMI_ESIGI and ang == 90:
                intersection_detected = True
                cv2.putText(image, "INTERSECTION!", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
                cv2.drawContours(image, [cv2.boxPoints(blackbox).astype(int)], 0, (255, 0, 0), 3)
            else:
                intersection_detected = False

            setpoint = 160
            error = int(x_min - setpoint)
            ang = int(ang)

            combined_error = error + (K_ang * ang)
            integral += combined_error
            derivative = combined_error - previous_error
            control = Kp * combined_error + Ki * integral + Kd * derivative
            previous_error = combined_error

            set_motor_direction(control, intersection_detected)

        cv2.imshow('Binary', binary)
        cv2.imshow("Original with line", image)
        rawCapture.truncate(0)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
finally:
    right_motor_forward.stop()
    left_motor_forward.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()