# Laser Center Extraction Module

## 파일 위치
`scripts/Sensor/laser/extract_laser_center.py`

## 개요
붉은색 레이저 띠의 중심선을 서브픽셀 정밀도로 추출하는 모듈.
ArUco 마커 기반 ROI 설정을 지원하여 반사광 등 노이즈를 제거한다.

## 처리 파이프라인

```
입력 이미지 (BGR)
    │
    ├─ [--aruco 모드] ──────────────────────────┐
    │   detect_aruco_markers()                  │
    │       → ArUco 5x5_50, 완화 파라미터       │
    │       → corners 리스트                    │
    │   aruco_roi_mask()                        │
    │       → 마커 bbox × (1 + 2×margin) ROI    │
    │                                           │
    ├───────────────────────────────────────────┘
    │
    ▼
extract_red_mask()
    │  HSV 빨간색 분리 (H:0~10 + 170~180)
    │  모폴로지 Open/Close 노이즈 제거
    │  [ArUco 모드] AND roi_mask
    ▼
열(column)별 가중 중심 계산
    │  Red 채널 intensity × mask
    │  weighted centroid: Σ(row × weight) / Σ(weight)
    │  stripe 폭 필터: min=3, max=80 px
    ▼
fit_laser_line()
    │  반복 MAD 기반 outlier 제거 (max 3회)
    │  threshold = median(residual) + 3.0 × max(MAD, 1.0)
    │  최종 1차 다항식 피팅 (polyfit)
    ▼
출력: cols, centers_y, coeffs, angle_deg
```

## 함수 상세

### `extract_red_mask(image) → mask`
- HSV 색공간에서 빨간색 영역 추출
- H=0~10 (lower red) + H=170~180 (upper red) OR 결합
- S, V 최소 50으로 흰색/회색 제외
- 3x3 타원 커널로 Open → Close 노이즈 제거

### `extract_laser_center(image, min_stripe_width=3, max_stripe_width=80) → (cols, centers_y, mask)`
- 전체 이미지에서 열별 레이저 중심 추출
- Red 채널 intensity를 가중치로 사용 (서브픽셀 정밀도)
- stripe 폭 필터로 노이즈/반사광 영역 제거

### `fit_laser_line(cols, centers_y, degree=1, mad_scale=3.0, max_iterations=3) → (coeffs, inlier_cols, inlier_y)`
- 반복적 MAD(Median Absolute Deviation) 기반 outlier 제거
- 적응형 임계값: `max(median + 3×MAD, 3.0)` px
- 수렴 시 조기 종료 (inlier 수 변화 없음)
- 최종 polyfit으로 직선(또는 다항식) 계수 반환

### `detect_aruco_markers(image) → corners_list`
- DICT_5X5_50 딕셔너리 사용
- 완화된 검출 파라미터 (작은 마커, 저해상도 대응):
  - `minMarkerPerimeterRate = 0.005`
  - `adaptiveThreshWinSizeMax = 53`
  - `polygonalApproxAccuracyRate = 0.08`

### `aruco_roi_mask(image_shape, aruco_corners_list, margin_ratio=0.5) → roi_mask`
- 각 마커 bounding box를 `margin_ratio`만큼 확장
- margin_ratio=0.5 → 마커 크기의 50% 여유 (총 2배 영역)
- 복수 마커 ROI 합집합

### `extract_laser_center_with_aruco(image, margin_ratio=0.5) → (cols, centers_y, red_mask, aruco_corners_list, roi_mask)`
- ArUco 검출 → ROI 생성 → ROI 내 레이저 중심 추출
- ArUco 미검출 시 전체 이미지 fallback

### `visualize(image, cols, centers_y, mask, coeffs, aruco_corners_list, roi_mask) → None`
- matplotlib 기반 3패널 시각화
  - 좌: 원본 + ArUco ROI(빨간 점선) + 중심점(녹색) + 피팅선(노란색)
  - 중: Red Mask (ROI 적용 후)
  - 우: ArUco ROI Mask

### `process_image(image_path, visualize_result=True, use_aruco=False, margin_ratio=0.5) → dict`
- 통합 처리 함수
- 반환: `{cols, centers_y, coeffs, angle_deg, aruco_count}`

## CLI 사용법

```bash
# 기본 (전체 이미지, 시각화 포함)
python ~/Project/Charging_Robot_Operation/scripts/Sensor/laser/extract_laser_center.py

# ArUco ROI 모드
python ~/Project/Charging_Robot_Operation/scripts/Sensor/laser/extract_laser_center.py --aruco

# margin 조절 + 시각화 없이
python ~/Project/Charging_Robot_Operation/scripts/Sensor/laser/extract_laser_center.py --aruco --margin 0.8 --no-vis

# 다른 이미지 지정
python ~/Project/Charging_Robot_Operation/scripts/Sensor/laser/extract_laser_center.py /path/to/image.png --aruco
```

## API 사용법

```python
from scripts.Sensor.laser.extract_laser_center import (
    extract_laser_center,
    extract_laser_center_with_aruco,
    fit_laser_line,
)

# 전체 이미지 모드
cols, centers_y, mask = extract_laser_center(image)
coeffs, inlier_cols, inlier_y = fit_laser_line(cols, centers_y)

# ArUco ROI 모드
cols, centers_y, mask, aruco_corners, roi_mask = extract_laser_center_with_aruco(image)
coeffs, inlier_cols, inlier_y = fit_laser_line(cols, centers_y)

# 기울기 (도)
angle_deg = np.degrees(np.arctan(coeffs[0]))
```

## 테스트 결과 (vision_20260214_122957.png)

| 모드 | 검출 포인트 | Inlier | 기울기 | Y범위 (px) |
|------|-----------|--------|--------|-----------|
| 전체 이미지 | 416 | 352 | 0.54° | 358.1~369.7 |
| ArUco ROI | 359 | 338 | 0.55° | 358.1~369.7 |

ArUco ROI 모드가 장치 내부 반사광 등 노이즈 영역을 제거하여 더 깨끗한 결과를 제공한다.

## 의존성
- OpenCV (cv2) — ArUco, HSV, morphology
- NumPy — 가중 중심, polyfit
- matplotlib — 시각화 (선택)
