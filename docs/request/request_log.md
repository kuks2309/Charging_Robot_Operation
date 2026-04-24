# Request Log

---

## 2026-04-24 | 레이저 삼각측량 캘리브레이션 데이터 분석 + 모델 재피팅 + 3D 재구성

### 요청 내용

1. `data/laser_scan/calibration/vertical/` 폴더의 수직 캘리브레이션 데이터 분석
2. 레이저 삼각측량법 원리 기반 정밀 분석 (20명 에이전트 병렬 투입)
3. 추가 실험 2개 데이터 포함한 3-scan 통합 분석
4. 3D 형상 재구성 및 평면 추출
5. 이슈 픽스 문서 참조하여 기존 모델 히스토리 확인 후 재피팅
6. Config 업데이트 + 3D viewer 수정

### 분석 과정

#### Phase 1: 20명 에이전트 초기 분석 (Scan 1)
- 데이터 품질, 베이스라인, 노이즈, 계단 검출, 회귀, 감도, 교차검증 등 20개 관점 병렬 분석
- **근본 원인 발견**: `tab_laser_calibration.py` Line 1284에서 `step_tread_mm=2.0` (실제 10.0) 로딩 버그 → RMSE 57px 캘리브레이션 실패
- 7구간 플랫 영역 식별 (바닥→5단→바닥), 폐합 오차 0.2px

#### Phase 2: 추가 실험 2개 + 3-scan 통합
- Scan 2 (baseline=797px), Scan 3 (baseline=844px) 추가 데이터 분석
- 3개 스캔 통합으로 로봇 높이 변화 추정: scan1→2: 10mm, scan2→3: 19mm
- 재파라미터화 모델 (delta_z=1497) 구축 → RMSE 5-7px
- **α-δz 축퇴 발견**: alpha=0.5°~14.5° 전 범위에서 RMSE 3.08~3.15px로 사실상 동일

#### Phase 3: 10명 에이전트 검증
- V1~V10 독립 검증: 수학적 정합성, RMSE, 버그, 물리적 타당성, FOV, 감도, 노이즈, 교차검증, 축퇴 영향
- V4: 플레이트 시차로 실제 d0≈400mm 추정 (delta_z=1497 → D≈2000mm과 모순)
- V5: step jump으로 D≈1910mm 추정 (V4와 해석 차이 → 수직/수평 좌표 혼동)

#### Phase 4: 이슈 픽스 참조 + Ray-tracing 모델 재피팅
- `docs/issues_and_fixes/issues_and_fixes.md`에서 기존 모델 히스토리 확인:
  - 03-07: 비선형 ray-tracing 모델 (h=70, Bx=-30, α=13.278°) — 수평 스캔 기반
  - 03-21: 장비 이동 (X→Y축), Bx→By 이름 변경 — **재캘리브레이션 없었음**
- 03-21 수직 스캔 데이터 3개로 ray-tracing 모델 재피팅:
  - h=47.56mm, By=66.75mm, α=12.863°, d0=362.7mm
  - **RMSE=0.642px**, 최대 step 높이 오차 **0.45mm**

#### Phase 5: 3D 재구성 + 평면 추출
- 선형 모델: Step 5 오차 +2.61mm → ray-tracing: **+0.29mm** (9배 개선)
- 6개 평면 추출: 전이구간 오염으로 가짜 기울기 발생 → 트림 후 모든 step 0.6°~2.8° 수렴

### 수정 파일

| 파일 | 변경 내용 |
|------|----------|
| `config/laser/vertical/laser_vertical_triangulation_calib.json` | v3 ray-tracing 파라미터 업데이트 (2026-04-24 17:10 KST) |
| `scripts/tools/laser_3d_viewer.py` | ray-tracing 역모델 적용 (config에서 파라미터 로드) |
| `docs/laser_calibration/vertical_triangulation_analysis_report.md` | 전체 분석 과정 문서화 |

### 미수정 (별도 작업 필요)

| 파일 | 필요 변경 | 비고 |
|------|----------|------|
| `scripts/tabs/tab_laser_calibration.py` Line 1284 | `cfg.get('step_interval_mm', 2.0)` → `cfg.get('step_tread_mm', 10.0)` | step_tread 5배 오류 버그 수정 |
| `scripts/services/laser_scan_service.py` | v3 파라미터(h=47.56, By=66.75, α=12.863) 로드 로직 업데이트 | 기존 h_mm/By_mm/alpha_deg 키 유지, 값만 변경됨 |

### 결과 요약

| 항목 | v2 (선형, δz=1497) | v3 (ray-tracing) |
|------|-------------------|-----------------|
| d0 | ~2000mm (비물리적) | **362.7mm** (물리적) |
| RMSE | 7.12px | **0.642px** |
| 최대 step 오차 | 2.61mm | **0.45mm** |
| α | 축퇴 (결정 불가) | **12.863°** (기존 13.278°과 일치) |
| 물리적 정합성 | 부정확 | 확인됨 |

### 생성된 분석 파일

- `data/laser_scan/calibration/vertical/analysis/` — 50+ 그래프 (20명+10명 에이전트)
- `.omc/scientist/reports/` — 20+ 리포트
- `.omc/scientist/figures/` — 30+ 그래프
- `data/laser_scan/calibration/vertical/analysis/3d_raytrace_v3_result.png` — 최종 결과

### 교훈

1. **이슈 픽스 문서를 반드시 먼저 참조** — 기존 모델 히스토리와 장비 변경 이력 확인 필수
2. **장비 이동 후 재캘리브레이션 필수** — 이름만 변경(Bx→By)하고 값을 유지하면 물리적 의미 상실
3. **축퇴 파라미터 주의** — h, By는 개별 값이 아닌 조합 K=h+By·tan(α)만 식별 가능
4. **선형 근사의 한계** — d0≈360mm에서 50mm protrusion은 14% 거리 변화 → 비선형 보정 필수
5. **3D 재구성 시 전이구간 제거** — 계단 경계의 레이저 산란이 가짜 기울기 생성

---
