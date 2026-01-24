# Charging Robot Operation - 설치 가이드

## 목차
- [시스템 요구사항](#시스템-요구사항)
- [Python 가상환경 설정](#python-가상환경-설정)
- [의존성 패키지 설치](#의존성-패키지-설치)
- [Qt5와 OpenCV 충돌 해결](#qt5와-opencv-충돌-해결)
- [실행 방법](#실행-방법)
- [문제 해결](#문제-해결)

---

## 시스템 요구사항

### 운영체제
- Ubuntu 20.04 이상 (또는 Debian 기반 Linux)
- Python 3.10 이상

### 필수 시스템 패키지
```bash
sudo apt update
sudo apt install python3.10-venv python3-dev build-essential
```

### 하드웨어
- Intel RealSense D435 카메라 (선택사항)
- Modbus TCP 지원 로봇 컨트롤러

---

## Python 가상환경 설정

Qt5와 OpenCV의 플러그인 충돌을 방지하기 위해 **가상환경 사용을 강력히 권장**합니다.

### 1. 가상환경 생성

프로젝트 루트 디렉토리에서:

```bash
cd ~/Project/Charging_Robot_Operation
python3 -m venv charging_robot
```

### 2. 가상환경 활성화

```bash
source charging_robot/bin/activate
```

활성화되면 프롬프트 앞에 `(charging_robot)`이 표시됩니다.

### 3. pip 업그레이드

```bash
pip install --upgrade pip
```

---

## 의존성 패키지 설치

### 필수 패키지 설치

가상환경이 활성화된 상태에서:

```bash
pip install PyQt5 \
            numpy \
            opencv-python \
            pyrealsense2 \
            pymodbus \
            pyyaml \
            netifaces \
            scipy \
            matplotlib
```

### 패키지 목록 및 용도

| 패키지 | 버전 | 용도 |
|--------|------|------|
| PyQt5 | >= 5.15 | GUI 프레임워크 |
| numpy | >= 1.20 | 수치 연산 |
| opencv-python | >= 4.5 | 이미지 처리 및 ArUco 마커 감지 |
| pyrealsense2 | >= 2.50 | Intel RealSense 카메라 제어 |
| pymodbus | >= 3.0 | Modbus TCP 통신 |
| pyyaml | >= 5.4 | 설정 파일 파싱 |
| netifaces | >= 0.11 | 네트워크 인터페이스 조회 |
| scipy | >= 1.7 | 과학 연산 (좌표 변환 등) |
| matplotlib | >= 3.3 | 3D 좌표계 시각화 |

---

## Qt5와 OpenCV 충돌 해결

### 문제 원인

OpenCV는 자체 Qt 플러그인을 포함하고 있어 PyQt5와 플러그인 로더가 충돌할 수 있습니다.
일반적으로 다음과 같은 오류가 발생합니다:

```
qt.qpa.plugin: Could not load the Qt platform plugin "xcb" in
"/path/to/cv2/qt/plugins" even though it was found.
```

### 해결 방법: OpenCV Qt 플러그인 제거 (권장)

가상환경 내부의 OpenCV Qt 플러그인 디렉토리를 제거합니다:

```bash
# 가상환경 활성화 상태에서
rm -rf charging_robot/lib/python3.10/site-packages/cv2/qt
```

이 방법은 다음과 같은 이점이 있습니다:
- OpenCV의 GUI 기능(`cv2.imshow` 등)은 비활성화되지만, 현재 프로젝트는 PyQt5 QLabel로만 이미지를 표시하므로 문제없음
- PyQt5와 OpenCV를 모두 정상적으로 사용 가능
- headless 모드 없이 완전한 기능 사용

### 대안: opencv-python-headless 사용

만약 시스템 전체에서 OpenCV GUI 기능이 필요 없다면:

```bash
pip uninstall opencv-python opencv-contrib-python
pip install opencv-python-headless
```

---

## 실행 방법

### 1. 가상환경 활성화

```bash
cd ~/Project/Charging_Robot_Operation
source charging_robot/bin/activate
```

### 2. 애플리케이션 실행

```bash
cd scripts
python3 main.py
```

### 3. 종료

GUI를 닫거나 터미널에서 `Ctrl+C`

### 4. 가상환경 비활성화 (작업 완료 후)

```bash
deactivate
```

---

## 문제 해결

### 1. Qt 플러그인 오류

**증상:**
```
qt.qpa.plugin: Could not load the Qt platform plugin "xcb"
```

**해결:**
```bash
rm -rf charging_robot/lib/python3.10/site-packages/cv2/qt
```

### 2. ModuleNotFoundError

**증상:**
```
ModuleNotFoundError: No module named 'xxx'
```

**해결:**
가상환경이 활성화되어 있는지 확인 후 해당 패키지 설치:
```bash
source charging_robot/bin/activate
pip install <package-name>
```

### 3. RealSense 카메라 인식 안 됨

**증상:**
카메라를 시작할 수 없음

**해결:**
1. USB 연결 확인
2. RealSense SDK 설치 확인:
   ```bash
   rs-enumerate-devices
   ```
3. 권한 문제 해결:
   ```bash
   sudo usermod -a -G video $USER
   # 재로그인 필요
   ```

### 4. Modbus 연결 실패

**증상:**
로봇 연결 시 타임아웃 또는 실패

**해결:**
1. 네트워크 연결 확인
2. IP 주소 확인
3. 방화벽 설정 확인:
   ```bash
   sudo ufw allow 502/tcp
   ```

### 5. Python 버전 호환성

이 프로젝트는 Python 3.10에서 테스트되었습니다. 다른 버전 사용 시:

```bash
# Python 3.10 설치
sudo apt install python3.10 python3.10-venv
python3.10 -m venv charging_robot
```

---

## 개발 환경 설정

### Git ignore 설정

가상환경을 git에서 제외:

```bash
echo "charging_robot/" >> .gitignore
```

### requirements.txt 생성 (선택사항)

현재 설치된 패키지 목록을 파일로 저장:

```bash
pip freeze > requirements.txt
```

향후 설치 시:

```bash
pip install -r requirements.txt
```

---

## 추가 정보

### 프로젝트 구조
```
Charging_Robot_Operation/
├── charging_robot/          # Python 가상환경 (git ignore)
├── config/                  # 설정 파일
├── docs/                    # 문서
├── scripts/                 # 메인 애플리케이션 코드
│   ├── main.py             # 진입점
│   ├── main_window.py      # 메인 윈도우
│   ├── Robot/              # 로봇 제어 모듈
│   ├── Sensor/             # 센서 모듈 (카메라, ArUco)
│   ├── services/           # 비즈니스 로직 서비스
│   ├── tabs/               # GUI 탭 모듈
│   └── utils/              # 유틸리티 함수
└── ui/                      # Qt Designer UI 파일
```

### 관련 문서
- [좌표계 시스템](./coordinate_systems.md)
- [리팩토링 문서](./refactoring/)
- [로봇 스크립트](./robot_scripts/)

---

## 라이선스

이 프로젝트는 KAIST에서 개발되었습니다.

## 문의

문제가 발생하면 이슈를 등록하거나 개발팀에 문의하세요.
