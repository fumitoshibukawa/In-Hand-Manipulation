import numpy as np
import cv2 as cv
import math
import serial
import time

# --- 定数定義 ---
WIDTH = 640
HEIGHT = 480
CENTER = (WIDTH // 2, HEIGHT // 2)
RADIUS = 190
PIXEL_TO_MM = 15 / 32
HSV_RANGE_BLACK = ([0, 0, 0], [179, 255, 70])

# --- カメラ初期化 ---
def init_camera():
    cap = cv.VideoCapture(0)
    cap.set(3, WIDTH)
    cap.set(4, HEIGHT)
    return cap

# --- シリアルポート初期化 ---
def init_serial():
    ser1 = serial.Serial('/dev/ttyUSB0', 115200)
    ser2 = serial.Serial('/dev/ttyUSB1', 115200)
    return ser1, ser2

# --- シリアル送信 ---
def send_serial_command(ser1, ser2, deg, x1, y1):
    command = f"A,{deg},{x1},{y1}\n".encode('ascii')
    ser1.reset_output_buffer()
    ser2.reset_output_buffer()
    for _ in range(10):
        ser1.write(command)
        ser2.write(command)
    time.sleep(0.5)

# --- 回転・前処理 ---
def preprocess_frame(frame):
    trans = cv.getRotationMatrix2D(CENTER, 150.0, 1.0)
    affine = cv.warpAffine(frame, trans, (WIDTH, HEIGHT), borderValue=(255, 255, 255))
    blurred = cv.blur(affine, (6, 6))
    masked = cv.rectangle(blurred, (0, 0), (230, 90), (255, 255, 255), thickness=-1)
    return masked

# --- マーカー検出 ---
def detect_marker(frame, start_flag, myPoints, file, ser1, ser2, flag):
    lower, upper = map(np.array, HSV_RANGE_BLACK)
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower, upper)

    kernel = np.ones((5, 5), np.uint8)
    processed = cv.dilate(cv.erode(mask, kernel, iterations=1), kernel, iterations=2)
    contours, _ = cv.findContours(processed, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        if cv.contourArea(contour) < 200:
            continue

        x, y, w, h = cv.boundingRect(contour)
        center_x = x + w // 2
        center_y = y + h // 2
        asp_ratio = w / float(h)
        r = math.sqrt((center_x - CENTER[0])**2 + (center_y - CENTER[1])**2)

        if 0.2 < asp_ratio < 3.0 and r < RADIUS:
            x1 = (center_x - CENTER[0]) * PIXEL_TO_MM
            y1 = (CENTER[1] - center_y) * PIXEL_TO_MM

            angle_deg = math.degrees(math.atan2(y1, x1))
            fai_x = int(angle_deg + 90)
            if fai_x > 180:
                fai_x -= 360

            fai_y = int(math.asin(r / RADIUS) * 57.2958 * (60 / 90))

            radian = math.atan2(center_y - CENTER[1], center_x - CENTER[0])
            deg = 90 + (180 - math.degrees(radian) if radian >= 0 else -180 - math.degrees(radian))
            if deg > 180:
                deg -= 360

            if start_flag:
                myPoints.append((center_x, center_y))
                file.write(f"{flag},{int(deg)},{x1},{y1},{fai_y},{fai_x}\n")
                send_serial_command(ser1, ser2, deg, x1, y1)

            cv.putText(frame, f"x={x1:.1f}", (5, 25), 1, 1.5, (220, 70, 90), 2)
            cv.putText(frame, f"y={y1:.1f}", (5, 55), 1, 1.5, (220, 70, 90), 2)
            cv.putText(frame, f"fai={fai_x}", (105, 25), 1, 1.5, (220, 70, 90), 2)
            cv.putText(frame, f"theta={fai_y}", (105, 55), 1, 1.5, (220, 70, 90), 2)
            cv.putText(frame, f"deg={int(deg)}", (55, 85), 1, 1.5, (220, 70, 90), 2)
            cv.circle(frame, (center_x, center_y), 3, (0, 255, 0), 3)
            cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            if abs(center_x - CENTER[0]) <= 12 and abs(center_y - CENTER[1]) <= 12:
                return 1

    return flag

# --- メイン関数 ---
def main():
    cap = init_camera()
    ser1, ser2 = init_serial()
    file = open('movie/log_straight.txt', 'w')
    file.write('flag,deg,x1,y1,theta,fai\n')

    start_flag = False
    flag = 0
    myPoints = []
    count = 0

    while True:
        key = cv.waitKey(10)
        if key == ord('q') or flag == 1:
            break
        elif key == ord('s'):
            start_flag = True
        elif key == ord('u'):
            for ser in [ser1, ser2]:
                ser.reset_output_buffer()
                ser.write(b'u\n')
            time.sleep(0.5)

        ret, frame = cap.read()
        if not ret:
            continue

        processed = preprocess_frame(frame)
        flag = detect_marker(processed, start_flag, myPoints, file, ser1, ser2, flag)

        if start_flag:
            cv.imwrite(f'movie/output-{count}.jpg', processed)
            count += 1

        cv.imshow("tracking", processed)

    file.close()
    cap.release()
    cv.destroyAllWindows()

    # --- 軌道を描いた画像を保存 ---
    cap = init_camera()
    ret, frame = cap.read()
    cap.release()
    if ret:
        while True:
            cv.circle(frame, CENTER, 2, (0, 0, 250), thickness=2, lineType=cv.LINE_AA)
            cv.circle(frame, CENTER, RADIUS, (255, 255, 255), thickness=2, lineType=cv.LINE_AA)
            if myPoints:
                for point in myPoints:
                    cv.circle(frame, point, 2, (235, 210, 100), thickness=1, lineType=cv.LINE_AA)
            cv.imshow("trajectory", frame)
            if cv.waitKey(10) == ord("q"):
                cv.imwrite('movie/kidou.jpg', frame)
                break
        cv.destroyAllWindows()

if __name__ == '__main__':
    main()
