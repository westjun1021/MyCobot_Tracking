import cv2
import cv2.aruco as aruco
import numpy as np
import serial
import time
import atexit
import math
import threading

# =========================================================
# 1. 설정값
# =========================================================
PORT = 'COM4'
BAUD = 38400  
W, H = 640, 480 
ALPHA = 0.2 
SEND_INTERVAL = 0.08 

# UI 컬러 정의
COLOR_CYAN = (255, 255, 0)
COLOR_GREEN = (0, 255, 0)
COLOR_RED = (0, 0, 255)
COLOR_WHITE = (255, 255, 255)
COLOR_BLACK = (0, 0, 0)

# =========================================================
# 2. 카메라 쓰레드
# =========================================================
class CameraGetter:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(3, W); self.cap.set(4, H)
        self.grabbed, self.frame = self.cap.read()
        self.started = False
        self.read_lock = threading.Lock()

    def start(self):
        if self.started: return
        self.started = True
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()
        return self

    def update(self):
        while self.started:
            grabbed, frame = self.cap.read()
            with self.read_lock:
                self.grabbed = grabbed
                self.frame = frame
            time.sleep(0.005)

    def read(self):
        with self.read_lock:
            frame = self.frame.copy() if self.frame is not None else None
            grabbed = self.grabbed
        return grabbed, frame

    def stop(self):
        self.started = False
        self.thread.join()
        self.cap.release()

# =========================================================
# 3. 메인 로직
# =========================================================
try:
    ser = serial.Serial(PORT, BAUD)
    print("✅ 로봇 연결 성공")
except:
    ser = None

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX 

camera_matrix = np.array([[500, 0, W/2], [0, 500, H/2], [0, 0, 1]], float)
dist_coeffs = np.zeros((5, 1))

curr_pos = {'x': 0, 'y': 0, 'z': 0}
last_send = 0

def send_angles(angles):
    if ser and ser.is_open:
        msg = ",".join(f"{a:.2f}" for a in angles) + "\n"
        ser.write(msg.encode())

def cleanup():
    if ser: send_angles([0.0]*6)
    if ser: ser.close()
    video_getter.stop()

# 텍스트 그리기 헬퍼
def draw_text(img, text, pos, color=COLOR_WHITE, scale=0.5, thickness=1):
    cv2.putText(img, text, (pos[0]+1, pos[1]+1), cv2.FONT_HERSHEY_SIMPLEX, scale, COLOR_BLACK, thickness+1)
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness)

#좌표 정보를 "중앙 기준 오차(Offset)"로 변경한 HUD
def draw_hud(img, detected, j_vals=None, offsets=None):
    h, w = img.shape[:2]
    cx, cy = w // 2, h // 2
    
    # 1. 중앙 조준선
    gap = 20; length = 40
    cv2.line(img, (cx - gap - length, cy), (cx - gap, cy), COLOR_CYAN, 1)
    cv2.line(img, (cx + gap, cy), (cx + gap + length, cy), COLOR_CYAN, 1)
    cv2.line(img, (cx, cy - gap - length), (cx, cy - gap), COLOR_CYAN, 1)
    cv2.line(img, (cx, cy + gap), (cx, cy + gap + length), COLOR_CYAN, 1)
    cv2.circle(img, (cx, cy), 2, COLOR_RED, -1)

    # 2. 좌측 데이터 패널
    overlay = img.copy()
    cv2.rectangle(overlay, (0, 0), (230, h), COLOR_BLACK, -1)
    img = cv2.addWeighted(overlay, 0.3, img, 0.7, 0)

    # 3. 상태 표시
    status_text = "STATUS: SEARCHING"
    status_color = COLOR_RED
    if detected:
        status_text = "STATUS: TRACKING"
        status_color = COLOR_GREEN
            
    draw_text(img, status_text, (20, 30), status_color, 0.6, 2)
    cv2.rectangle(img, (10, 10), (220, h-10), status_color, 1)

    # 4. 데이터 표시
    draw_text(img, "---------------------", (20, 50), COLOR_WHITE)
    draw_text(img, "[ JOINTS (Deg) ]", (20, 70), COLOR_CYAN, 0.5, 2)
    if j_vals is not None:
        draw_text(img, f"J1: {j_vals[0]:.1f}  J2: {j_vals[1]:.1f}", (20, 95))
        draw_text(img, f"J3: {j_vals[2]:.1f}  J5: {j_vals[4]:.1f}", (20, 120))
    else:
        draw_text(img, "No Data", (20, 100), (100,100,100))

    draw_text(img, "---------------------", (20, 160), COLOR_WHITE)
    #좌표 데이터 (Offset)
    draw_text(img, "[ TARGET OFFSET ]", (20, 180), COLOR_CYAN, 0.5, 2)
    
    if offsets is not None:
        dx, dy, dist_cm = offsets
        
        # dX 표시 (좌우 오차)
        color_x = COLOR_GREEN if abs(dx) < 20 else COLOR_WHITE
        dir_x = "R" if dx > 0 else "L"
        draw_text(img, f"dX : {abs(dx):.0f} px ({dir_x})", (20, 205), color_x)
        
        # dY 표시 (상하 오차)
        color_y = COLOR_GREEN if abs(dy) < 20 else COLOR_WHITE
        dir_y = "UP" if dy > 0 else "DN"
        draw_text(img, f"dY : {abs(dy):.0f} px ({dir_y})", (20, 230), color_y)
        
        # 거리 표시 (cm 단위)
        draw_text(img, f"DST: {dist_cm:.1f} cm", (20, 255), COLOR_CYAN, 0.6, 2)

    else:
        draw_text(img, "No Target", (20, 220), (100,100,100))
        
    return img

atexit.register(cleanup)

video_getter = CameraGetter(0).start()
CX, CY = W // 2, H // 2

while True:
    ret, frame = video_getter.read()
    if not ret or frame is None: continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    display = cv2.flip(frame, 1)

    j_vals = None; offsets = None; detected = False

    if ids is not None:
        idx = -1
        for i in range(len(ids)):
            if ids[i][0] == 0: idx = i; break

        if idx != -1:
            detected = True
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.04, camera_matrix, dist_coeffs)
            c = corners[idx][0]

            raw_x = (c[0][0] + c[2][0]) / 2
            raw_y = (c[0][1] + c[2][1]) / 2
            raw_z = tvecs[idx][0][2]

            if curr_pos['z'] == 0:
                curr_pos['x'], curr_pos['y'], curr_pos['z'] = raw_x, raw_y, raw_z
            else:
                curr_pos['x'] = curr_pos['x']*(1-ALPHA) + raw_x*ALPHA
                curr_pos['y'] = curr_pos['y']*(1-ALPHA) + raw_y*ALPHA
                curr_pos['z'] = curr_pos['z']*(1-ALPHA) + raw_z*ALPHA
            
            tx, ty, tz = curr_pos['x'], curr_pos['y'], curr_pos['z']
            
            # 좌표 계산 (중앙 기준 오차 및 cm 거리)
            # Offset X: 화면 중앙에서 오른쪽이 (+), 왼쪽이 (-)
            offset_x = tx - CX 
            # Offset Y: 화면 중앙에서 위쪽이 (+), 아래쪽이 (-) -> 직관성을 위해 반전
            offset_y = CY - ty 
            # Distance: m -> cm 변환
            dist_cm = tz * 100
            
            offsets = [offset_x, offset_y, dist_cm]

            # 각도 계산 (로봇 제어용)
            j1 = np.interp(tx, [0, W], [-90, 90])
            j2 = np.interp(ty, [0, H], [50, -50])
            j3 = np.interp(tz, [0.1, 0.5], [60, -60])
            j5 = np.clip(-(j2 + j3), -90, 90) # 수평 유지
            j4, j6 = 0.0, 0.0 

            new_angles = np.array([j1, j2, j3, j4, j5, j6])
            j_vals = new_angles

            if time.time() - last_send > SEND_INTERVAL:
                send_angles(new_angles)
                last_send = time.time()

            # 마커 시각화
            mx, my = int(W-tx), int(ty)
            cv2.rectangle(display, (mx-20, my-20), (mx+20, my+20), COLOR_GREEN, 2)
            cv2.line(display, (mx, my), (CX, CY), COLOR_GREEN, 1) # 오차 연결선

    # HUD 그리기
    display = draw_hud(display, detected, j_vals, offsets)

    cv2.imshow("Robot Controller HUD", display)
    if cv2.waitKey(1) == ord('q'): break

cleanup()
cv2.destroyAllWindows()