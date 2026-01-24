#!/bin/bash
###############################################################################
# Charging Robot Operation - Dependency Installation Script
# 충전 로봇 운영 시스템 - 의존성 설치 스크립트
###############################################################################

set -e  # 에러 발생 시 스크립트 중단

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 로그 함수
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Python 버전 확인
check_python_version() {
    log_info "Python 버전 확인 중..."
    if ! command -v python3 &> /dev/null; then
        log_error "Python3가 설치되어 있지 않습니다."
        exit 1
    fi

    # Python으로 직접 버전 체크 (더 안정적)
    PYTHON_VERSION=$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')
    log_info "Python 버전: $PYTHON_VERSION"

    # Python으로 버전 비교
    VERSION_CHECK=$(python3 -c "import sys; print(1 if sys.version_info >= (3, 8) else 0)")

    if [ "$VERSION_CHECK" -eq "0" ]; then
        log_error "Python 3.8 이상이 필요합니다. 현재 버전: $PYTHON_VERSION"
        exit 1
    fi
}

# 1. 시스템 패키지 업데이트
install_system_packages() {
    log_info "시스템 패키지 업데이트 중..."
    sudo apt-get update

    log_info "시스템 의존성 패키지 설치 중..."
    sudo apt-get install -y \
        python3-dev \
        python3-pip \
        build-essential \
        cmake \
        git \
        wget \
        curl \
        pkg-config \
        libssl-dev \
        libusb-1.0-0-dev \
        libudev-dev \
        libgtk-3-dev \
        libglfw3-dev \
        libgl1-mesa-dev \
        libglu1-mesa-dev

    log_info "Qt5 라이브러리 설치 중..."
    sudo apt-get install -y \
        qtbase5-dev \
        qtchooser \
        qt5-qmake \
        qtbase5-dev-tools \
        python3-pyqt5 \
        python3-pyqt5.qtsvg \
        pyqt5-dev-tools

    log_info "OpenCV 관련 시스템 라이브러리 설치 중..."
    sudo apt-get install -y \
        libopencv-dev \
        python3-opencv \
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        libv4l-dev \
        libxvidcore-dev \
        libx264-dev
}

# 2. Intel RealSense SDK 설치
install_realsense_sdk() {
    log_info "Intel RealSense SDK 설치 확인 중..."

    if dpkg -l | grep -q librealsense2; then
        log_warn "Intel RealSense SDK가 이미 설치되어 있습니다."
        return
    fi

    log_info "Intel RealSense 저장소 추가 중..."
    sudo mkdir -p /etc/apt/keyrings
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | \
        sudo tee /etc/apt/sources.list.d/librealsense.list

    sudo apt-get update

    log_info "Intel RealSense SDK 설치 중..."
    sudo apt-get install -y \
        librealsense2-dkms \
        librealsense2-utils \
        librealsense2-dev \
        librealsense2-dbg
}

# 3. Python 패키지 설치
install_python_packages() {
    log_info "pip 업그레이드 중..."
    python3 -m pip install --upgrade pip setuptools wheel

    log_info "Python 패키지 설치 중..."

    # GUI 관련
    log_info "  - GUI 라이브러리 (PyQt5, PySide2)..."
    pip3 install PyQt5 PyQt5-sip PySide2

    # 컴퓨터 비전 핵심
    log_info "  - 컴퓨터 비전 라이브러리 (OpenCV, RealSense)..."
    pip3 install opencv-python opencv-contrib-python pyrealsense2

    # AprilTag/ArUco 마커 검출
    log_info "  - 마커 검출 라이브러리..."
    pip3 install pupil-apriltags

    # 딥러닝 프레임워크
    log_info "  - 딥러닝 프레임워크 (PyTorch)..."
    pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

    # YOLO
    log_info "  - YOLO (Ultralytics)..."
    pip3 install ultralytics

    # 과학 계산
    log_info "  - 과학 계산 라이브러리 (NumPy, SciPy, scikit-learn)..."
    pip3 install numpy scipy scikit-learn scikit-image

    # 3D 비전 및 포인트 클라우드
    log_info "  - 3D 비전 라이브러리 (Open3D)..."
    pip3 install open3d

    # Modbus 통신
    log_info "  - Modbus 통신 라이브러리..."
    pip3 install pymodbus

    # 시각화 및 플로팅
    log_info "  - 시각화 라이브러리 (Matplotlib)..."
    pip3 install matplotlib

    # 설정 관리
    log_info "  - 설정 관리 (Hydra)..."
    pip3 install hydra-core omegaconf

    # 네트워크 인터페이스
    log_info "  - 네트워크 도구..."
    pip3 install netifaces

    # 이미지 처리
    log_info "  - 이미지 처리 (Pillow)..."
    pip3 install Pillow

    # 데이터 처리
    log_info "  - 데이터 처리 도구..."
    pip3 install pandas pyyaml
}

# 4. SAM2 모델 설치 (선택적)
install_sam2_optional() {
    log_warn "SAM2 모델은 선택적 설치입니다. 설치하시겠습니까? (y/N)"
    read -r response
    if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
        log_info "SAM2 설치 중..."
        pip3 install git+https://github.com/facebookresearch/segment-anything-2.git
    else
        log_info "SAM2 설치를 건너뜁니다."
    fi
}

# 5. 설치 검증
verify_installation() {
    log_info "설치 검증 중..."

    local failed=0

    # Python 패키지 검증
    local packages=("PyQt5" "PySide2" "cv2" "numpy" "scipy" "torch" "pyrealsense2" "pymodbus" "open3d" "matplotlib")

    for package in "${packages[@]}"; do
        if python3 -c "import $package" 2>/dev/null; then
            log_info "  ✓ $package"
        else
            log_error "  ✗ $package 설치 실패"
            failed=1
        fi
    done

    if [ $failed -eq 0 ]; then
        log_info "모든 패키지가 성공적으로 설치되었습니다!"
    else
        log_error "일부 패키지 설치에 실패했습니다. 위의 에러 메시지를 확인하세요."
        exit 1
    fi
}

# 6. 권한 설정 (RealSense 카메라 접근)
setup_permissions() {
    log_info "USB 장치 접근 권한 설정 중..."

    # RealSense udev 규칙 설정
    if [ ! -f /etc/udev/rules.d/99-realsense-libusb.rules ]; then
        sudo tee /etc/udev/rules.d/99-realsense-libusb.rules > /dev/null <<EOF
# Intel RealSense D400 series
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3a", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad1", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad2", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad3", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad4", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad5", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0af6", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0afe", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0aff", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b00", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b01", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b03", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b0c", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b0d", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3d", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b48", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b49", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b4b", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b4d", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b52", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5b", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5c", MODE="0666", GROUP="plugdev"
EOF
        sudo udevadm control --reload-rules
        sudo udevadm trigger
        log_info "RealSense udev 규칙이 설정되었습니다."
    fi

    # 현재 사용자를 plugdev 그룹에 추가
    sudo usermod -a -G plugdev $USER
    log_warn "재부팅 후 권한 설정이 적용됩니다."
}

# 메인 실행
main() {
    echo "========================================================================="
    echo "  Charging Robot Operation - Dependency Setup"
    echo "  충전 로봇 운영 시스템 - 의존성 설치"
    echo "========================================================================="
    echo ""

    check_python_version
    echo ""

    install_system_packages
    echo ""

    install_realsense_sdk
    echo ""

    install_python_packages
    echo ""

    install_sam2_optional
    echo ""

    setup_permissions
    echo ""

    verify_installation
    echo ""

    echo "========================================================================="
    log_info "설치가 완료되었습니다!"
    echo ""
    log_info "다음 명령어로 프로그램을 실행할 수 있습니다:"
    echo "  cd /home/argoon/Project/Charging_Robot_Operation/scripts"
    echo "  python3 main.py"
    echo ""
    log_warn "재부팅 후 USB 카메라 권한이 적용됩니다."
    echo "========================================================================="
}

# 스크립트 실행
main "$@"
