# MyCobot Aruco Tracking Robot

웹캠을 통해 Aruco 마커를 인식하고, MyCobot 로봇 팔이 실시간으로 마커를 추적하는 프로젝트입니다.

## ✨ 주요 기능
- **Real-time Tracking:** OpenCV를 활용한 마커 6축(X, Y, Z, Rx, Ry, Rz) 인식
- **Smooth Motion:** 30ms 주기의 마이크로 스텝 제어 및 스무딩 알고리즘(Weighted Moving Average) 적용
- **Smart HUD:** 마커 오차(Offset) 및 거리 정보 실시간 디스플레이
- **Robot Control:** 3축 제어(J1, J2, J3) 및 손목 수평 유지(Self-Leveling) 알고리즘 적용

## 🛠️ 하드웨어
- Elephant Robotics MyCobot 280 (Arduino ver)
- 일반 USB 웹캠

## 🚀 실행 방법
1. 아두이노에 `arduino/robot_firmware.ino` 업로드
2. 파이썬 라이브러리 설치: `pip install -r python/requirements.txt`
3. 파이썬 실행: `python python/main.py`