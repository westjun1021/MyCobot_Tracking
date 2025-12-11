#include <MyCobotBasic.h>
#include <SoftwareSerial.h>

MyCobotBasic myCobot;
SoftwareSerial myLaptop(10, 11);

// 현재 위치와 목표 위치를 저장할 변수
float currentAngles[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float targetAngles[6]  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// [핵심 설정 1] 스무딩 값 0.15 
float SMOOTH = 0.15; 

unsigned long lastMoveTime = 0;

void setup() {
  Serial.begin(115200); // 디버깅용 (시리얼 모니터는 115200으로 봐도 됨)
  delay(500);
  
  myCobot.setup();
  myCobot.powerOn();
  delay(500);
  
  // SoftwareSerial은 115200을 버티지 못하며, 파이썬 코드도 38400입니다.
  myLaptop.begin(38400); 
  
  // 데이터 읽기 시간 제한 (5ms)
  // 데이터가 꼬여도 로봇이 1초간 멍때리는 것을 방지합니다.
  myLaptop.setTimeout(5);

  // [시작 알림]
  Angles init = {0,0,0,0,0,0};
  myCobot.writeAngles(init, 50); 
  delay(1000);
  Angles check = {0,0,0,0,0,20}; 
  myCobot.writeAngles(check, 50);
  delay(1000);
  
  pinMode(13, OUTPUT);
}

void loop() {
  // 1. 데이터 수신
  while (myLaptop.available()) {
    String s = myLaptop.readStringUntil('\n');
    
    // 노이즈 필터링
    if (s.length() < 5) continue;

    int commaCount = 0;
    for(int i=0; i<s.length(); i++) if(s.charAt(i) == ',') commaCount++;

    if (commaCount == 5) {
      int idx[5];
      idx[0] = s.indexOf(',');
      for(int i=1; i<5; i++) idx[i] = s.indexOf(',', idx[i-1] + 1);

      targetAngles[0] = constrain(s.substring(0, idx[0]).toFloat(), -160.0, 160.0);
      targetAngles[1] = constrain(s.substring(idx[0]+1, idx[1]).toFloat(), -100.0, 100.0);
      targetAngles[2] = constrain(s.substring(idx[1]+1, idx[2]).toFloat(), -100.0, 100.0);
      targetAngles[3] = constrain(s.substring(idx[2]+1, idx[3]).toFloat(), -100.0, 100.0);
      targetAngles[4] = constrain(s.substring(idx[3]+1, idx[4]).toFloat(), -100.0, 100.0);
      targetAngles[5] = constrain(s.substring(idx[4]+1).toFloat(), -170.0, 170.0);
      
      digitalWrite(13, !digitalRead(13)); 
    }
  }

  // 2. 이동 로직 (30ms 주기)
  if (millis() - lastMoveTime > 30) { 
    Angles sendData;
    bool meaningfulMove = false;

    for (int i = 0; i < 6; i++) {
      float diff = targetAngles[i] - currentAngles[i];
      
      // 스무딩 적용
      currentAngles[i] += diff * SMOOTH;
      sendData[i] = currentAngles[i];
      
      // 변화량 체크
      if (abs(diff) > 0.1) meaningfulMove = true;
    }
    
    // 의미 있는 움직임이 있을 때만 전송
    if (meaningfulMove) {
        myCobot.writeAngles(sendData, 80); 
    }
    
    lastMoveTime = millis();
  }
}
