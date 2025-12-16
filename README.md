# 🤖 MyCobot Tracking  
**ArUco 마커 기반 로봇팔 실시간 추적 시스템**

<p align="center">
  <img src="https://img.shields.io/badge/Language-Python-blue?logo=python" />
  <img src="https://img.shields.io/badge/OpenCV-4.x-5C3EE8?logo=opencv" />
  <img src="https://img.shields.io/badge/Hardware-MyCobot_280-orange" />
  <img src="https://img.shields.io/badge/Serial-Arduino-green" />
</p>

---

## 📌 프로젝트 개요  
**MyCobot Tracking**은 컴퓨터 비전 기반 마커 인식을 통해  
로봇팔이 사람처럼 자연스럽게 대상 물체를 따라 움직이도록 설계된  
**실시간 위치 추적 시스템(Real-Time Object Tracking System)**입니다.

ArUco 마커를 카메라로 인식하고,  
로봇 좌표계로 좌표를 변환한 뒤,  
Inverse Kinematics(IK, 역기구학)을 통해 부드럽게 따라가는 동작을 구현했습니다.

---

## 🎯 주요 기능

### ✔ 1. **ArUco 마커 인식 & Pose 추정**  
- OpenCV ArUco 라이브러리 사용  
- **위치(x, y) + 회전값(yaw)**까지 추정  
- 카메라 캘리브레이션 기반 정확도 향상

### ✔ 2. **카메라 좌표 → 로봇 좌표 변환**  
- MyCobot 기준 좌표계로 normalization  
- 거리 단위(mm) 보정  
- 오차 필터링(거리 threshold)

### ✔ 3. **Inverse Kinematics 기반 부드러운 움직임**  
- 속도 튐 현상 제거  
- low-pass filter 적용 → 사람 같은 가속/감속  
- 각 관절별 안전 각도 범위 제한

### ✔ 4. **Arduino 시리얼 통신 기반 제어**  
- Python → Serial → MyCobot-Arduino  
- delay 없는 고속 통신  
- 관절 안정화 알고리즘 적용

### ✔ 5. **실시간 화면 UI 출력**  
- 마커 박스  
- 중심점 표시  
- 로봇팔 목표 좌표 표시

---

## 🧠 시스템 구조
/MyCobot_Tracking
├── main.py # 메인 실행 파일
├── camera_config.py # 해상도 및 카메라 설정
├── aruco_detector.py # 마커 인식 및 pose 계산
├── coordinate_mapper.py # 카메라 좌표 → 로봇 좌표 변환
├── ik_solver.py # 역기구학 계산
├── serial_controller.py # Arduino Serial 송신
└── utils/ # 필터, 보정 함수




---

## 📸 시연 이미지 / 영상 (넣으면 포트폴리오 2배 상승)
추천 이미지:
- 마커 인식 화면  
- 로봇팔이 마커를 따라가는 사진  
- 좌표 출력 UI  
- 전체 시스템 구성(카메라 + 로봇)

원하시면 제가 **샘플 시연 이미지 스타일 템플릿**도 만들어드릴게요.

---

## 🚀 실행 방법

1. Python 3.9+ 설치  
2. 필요한 라이브러리 설치  
pip install opencv-python pyserial numpy



3. Arduino 업로드 (myCobot firmware)  
4. main.py 실행  
5. 마커를 움직이면 로봇팔이 따라감

---

## 🔧 하드웨어 세팅

- **MyCobot 280 (Arduino 버전)**  
- **노트북 기본 카메라 또는 USB 웹캠**  
- 마커 크기: **7cm × 7cm**  
- 로봇은 카메라 앞 **약 20~30cm** 위치에 두는 것이 안정적

---

## 📌 알고리즘 상세

### 📍 위치 필터링
- 작은 떨림 제거를 위한 **moving average filter** 적용  
- 좌표 변화량 threshold 미만일 경우 무시  

### 📍 IK 알고리즘
- 목표 위치 → joint angle 변환  
- singularity 회피  
- 관절 속도 제한  

### 📍 실시간 안정화
- 30~60 FPS 기반 추적  
- delay 최소화 로직  
- sudden jump 방지

---

## 🔮 향후 개선 사항

- Hand-eye calibration 적용 → 절대 정확도 향상  
- 여러 개의 마커를 동시에 추적  
- 3D trajectory tracking  
- ROS2 기반 확장  

---

## 👤 개발자  
**Jun (westjun1021)**  
- Computer Vision & Robotics Developer  
- MyCobot, ARKit, FastAPI 기반 실시간 시스템 개발 경험 보유








