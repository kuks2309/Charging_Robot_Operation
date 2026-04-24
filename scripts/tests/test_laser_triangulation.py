"""
단위 테스트: scripts/utils/laser_triangulation.py

테스트 대상:
- estimate_height: 알려진 지그 높이(10mm 단위)로 역산 정확도 검증
- detect_step_transitions: 5단 계단 감지 정확도
- calibrate_sensitivity: 모델 파라미터 범위 검증
- 엣지 케이스: 동일 Y_px(높이 0), 극단 Y_px 값

실제 데이터: data/laser_scan/calibration/vertical/scan_raw_20260321_195848.csv
실행: charging_robot/bin/python3 -m pytest scripts/tests/test_laser_triangulation.py -v
"""

import math
import os

import numpy as np
import pytest

# conftest.py가 scripts/ 를 sys.path에 추가한다
from utils.laser_triangulation import (
    BX_MM,
    CY_PX,
    DELTA_Z_MM,
    FY_PX,
    LASER_ALPHA_DEG,
    SENS_INTERCEPT,
    SENS_SLOPE,
    _sensitivity_at,
    calibrate_sensitivity,
    detect_step_transitions,
    estimate_depth,
    estimate_height,
)

# ---------------------------------------------------------------------------
# 경로 상수
# ---------------------------------------------------------------------------
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
CSV_PATH = os.path.join(
    _REPO_ROOT,
    "data", "laser_scan", "calibration", "vertical",
    "scan_raw_20260321_195848.csv",
)

# ---------------------------------------------------------------------------
# 헬퍼 — 합성 계단 데이터 생성
# ---------------------------------------------------------------------------

def _make_step_scan(
    n_steps: int = 5,
    pts_per_step: int = 5,
    y_floor: float = 773.7,
    step_delta_y: float = 27.0,
    z_start: float = 452.0,
    z_step: float = 10.0,
    noise_std: float = 0.0,
    gap_pts: int = 2,
    rng: np.random.Generator | None = None,
) -> tuple[list[float], list[float]]:
    """n_steps 단 계단의 합성 (z_tcp, y_px) 시퀀스를 반환한다.

    알고리즘 특성: 인접 점의 |dy| < threshold면 flat으로 분류한다.
    계단 사이에 경계점이 없으면 마지막 계단 점과 다음 계단 첫 점이
    서로의 "이전 flat" 조건으로 같은 그룹에 포함된다.
    따라서 gap_pts > 0의 전이 구간이 필요하다.

    각 계단은 pts_per_step 포인트 + 다음 계단과 gap_pts 전이 포인트로 구성.
    계단이 높아질수록 y_px가 감소한다(이미지 좌표).
    """
    if rng is None:
        rng = np.random.default_rng(42)
    z_vals: list[float] = []
    y_vals: list[float] = []
    z_cur = z_start
    for step in range(n_steps):
        y_level = y_floor - step * step_delta_y
        for j in range(pts_per_step):
            z_vals.append(z_cur)
            noise = float(rng.normal(0, noise_std)) if noise_std > 0 else 0.0
            y_vals.append(y_level + noise)
            z_cur += 0.5
        # 마지막 계단 뒤에는 전이 구간 불필요
        if step < n_steps - 1:
            y_next = y_floor - (step + 1) * step_delta_y
            for g in range(gap_pts):
                z_vals.append(z_cur)
                frac = (g + 1) / (gap_pts + 1)
                y_vals.append(y_level + frac * (y_next - y_level))
                z_cur += 0.5
    return z_vals, y_vals


# ---------------------------------------------------------------------------
# _sensitivity_at (내부 헬퍼 — 공개 테스트)
# ---------------------------------------------------------------------------

class TestSensitivityAt:
    """_sensitivity_at 선형 모델 기본 동작 검증."""

    def test_returns_float(self):
        """임의 y_px에 대해 float를 반환해야 한다."""
        result = _sensitivity_at(700.0)
        assert isinstance(result, float)

    def test_linear_model_value_at_known_y(self):
        """y=700px에서 모델값이 SENS_SLOPE*700 + SENS_INTERCEPT여야 한다."""
        expected = SENS_SLOPE * 700.0 + SENS_INTERCEPT
        assert _sensitivity_at(700.0) == pytest.approx(expected, rel=1e-9)

    def test_sensitivity_decreases_as_y_increases(self):
        """SENS_SLOPE < 0이므로 y가 커질수록 sensitivity가 작아야 한다."""
        assert _sensitivity_at(800.0) < _sensitivity_at(600.0)

    def test_sensitivity_is_positive_in_operating_range(self):
        """실운용 Y 범위(400–900px)에서 sensitivity는 양수여야 한다."""
        for y in [400.0, 600.0, 773.7, 900.0]:
            assert _sensitivity_at(y) > 0, f"sensitivity <= 0 at y={y}"


# ---------------------------------------------------------------------------
# estimate_height — 합성 데이터 정확도
# ---------------------------------------------------------------------------

class TestEstimateHeightSynthetic:
    """estimate_height: 합성 알려진 높이로 역산 정확도 검증."""

    def test_zero_delta_y_returns_zero_height(self):
        """y_base == y_measured이면 높이 0mm를 반환해야 한다."""
        result = estimate_height(773.7, 773.7)
        assert result == pytest.approx(0.0, abs=1e-9)

    def test_height_is_positive_when_y_measured_less_than_y_base(self):
        """y_measured < y_base(표면이 바닥보다 높음)이면 양수를 반환해야 한다."""
        result = estimate_height(773.7, 746.5)
        assert result > 0.0

    def test_height_is_negative_when_y_measured_greater_than_y_base(self):
        """y_measured > y_base(표면이 바닥보다 낮음)이면 음수를 반환해야 한다."""
        result = estimate_height(773.7, 800.0)
        assert result < 0.0

    def test_known_height_10mm_accuracy_within_3mm(self):
        """실측 지그 step1(~27px 차이) 역산값이 10mm ± 3mm 이내여야 한다.

        실측값: y_floor≈773.7px, step1≈746.5px (delta≈27.2px)
        sensitivity(y_mid≈760px) ≈ SENS_SLOPE*760 + SENS_INTERCEPT
        """
        y_base = 773.7
        y_step1 = 746.5  # 실측 step1 plateau 중앙값 근사
        result = estimate_height(y_base, y_step1)
        assert result == pytest.approx(10.0, abs=3.0), (
            f"step1 높이 역산 오차 초과: {result:.2f}mm (기대 10mm ± 3mm)"
        )

    def test_known_height_20mm_step2_accuracy_within_5mm(self):
        """실측 지그 step2(~54.5px 차이) 역산값이 20mm ± 5mm 이내여야 한다."""
        y_base = 773.7
        y_step2 = 719.2  # 실측 step2 plateau 중앙값 근사
        result = estimate_height(y_base, y_step2)
        assert result == pytest.approx(20.0, abs=5.0), (
            f"step2 높이 역산 오차 초과: {result:.2f}mm (기대 20mm ± 5mm)"
        )

    def test_height_proportional_for_uniform_steps(self):
        """동일 간격 계단에서 역산 높이가 단조증가해야 한다."""
        y_base = 773.7
        step_delta_px = 27.0  # step당 약 27px
        heights = [
            estimate_height(y_base, y_base - k * step_delta_px)
            for k in range(1, 6)
        ]
        for i in range(len(heights) - 1):
            assert heights[i] < heights[i + 1], (
                f"높이가 단조증가하지 않음: heights={heights}"
            )

    def test_symmetry_negation(self):
        """(y_base, y_measured)와 (y_measured, y_base)의 결과는 부호만 달라야 한다.

        sensitivity 평가점(y_mid)이 동일하므로 정확히 반대부호여야 한다.
        """
        h_pos = estimate_height(773.7, 746.5)
        h_neg = estimate_height(746.5, 773.7)
        assert h_pos == pytest.approx(-h_neg, rel=1e-9)


# ---------------------------------------------------------------------------
# estimate_height — 엣지 케이스
# ---------------------------------------------------------------------------

class TestEstimateHeightEdgeCases:
    """estimate_height 엣지 케이스."""

    def test_extreme_low_y_px(self):
        """극단적으로 낮은 y_px(0px 근처)에서도 ValueError 없이 반환해야 한다."""
        result = estimate_height(773.7, 1.0)
        assert isinstance(result, float)

    def test_extreme_high_y_px(self):
        """극단적으로 높은 y_px(1079px 근처)에서도 ValueError 없이 반환해야 한다."""
        result = estimate_height(773.7, 1079.0)
        assert isinstance(result, float)

    def test_large_delta_y_returns_large_height(self):
        """큰 픽셀 차이는 큰 높이를 반환해야 한다."""
        small = estimate_height(773.7, 770.0)
        large = estimate_height(773.7, 600.0)
        assert large > small


# ---------------------------------------------------------------------------
# estimate_height — 실제 CSV 데이터 검증
# ---------------------------------------------------------------------------

class TestEstimateHeightRealData:
    """실제 스캔 CSV 데이터로 estimate_height 정확도 검증."""

    @pytest.fixture(scope="class")
    def plateau_y_values(self):
        """CSV에서 각 계단 plateau의 중앙값 Y_px를 반환한다.

        plateau는 낮은 표준편차(<5px) 포인트 그룹에서 선택한다.
        실측 plateau capture 인덱스(수동 확인):
          floor  : captures 0,1   → y ≈ 773.7px
          step1  : captures 3-6   → y ≈ 746.5px
          step2  : captures 10-13 → y ≈ 719.2px
          step3  : captures 17-19 → y ≈ 689.7px
          step4  : captures 22-26 → y ≈ 658.5px
          step5  : captures 29-31 → y ≈ 625.9px
        """
        try:
            import pandas as pd
        except ImportError:
            pytest.skip("pandas 미설치 — 실데이터 테스트 건너뜀")

        if not os.path.exists(CSV_PATH):
            pytest.skip(f"CSV 파일 없음: {CSV_PATH}")

        df = pd.read_csv(CSV_PATH)
        laser = df[df["roi_name"] == "roi_laser"]

        def median_y(capture_ids):
            return float(
                laser[laser["capture_idx"].isin(capture_ids)]["center_y_px"].median()
            )

        return {
            "floor":  median_y([0, 1]),
            "step1":  median_y([3, 4, 5, 6]),
            "step2":  median_y([10, 11, 12, 13]),
            "step3":  median_y([17, 18, 19]),
            "step4":  median_y([22, 23, 24, 25, 26]),
            "step5":  median_y([29, 30, 31]),
        }

    def test_step1_height_within_3mm_of_10mm(self, plateau_y_values):
        """step1 역산 높이가 10mm ± 3mm 이내여야 한다."""
        y_floor = plateau_y_values["floor"]
        y_step1 = plateau_y_values["step1"]
        h = estimate_height(y_floor, y_step1)
        assert h == pytest.approx(10.0, abs=3.0), (
            f"step1 실측 역산: {h:.2f}mm (기대 10mm ± 3mm)"
        )

    def test_step2_height_within_5mm_of_20mm(self, plateau_y_values):
        """step2 역산 높이가 20mm ± 5mm 이내여야 한다."""
        y_floor = plateau_y_values["floor"]
        y_step2 = plateau_y_values["step2"]
        h = estimate_height(y_floor, y_step2)
        assert h == pytest.approx(20.0, abs=5.0), (
            f"step2 실측 역산: {h:.2f}mm (기대 20mm ± 5mm)"
        )

    def test_step3_height_within_7mm_of_30mm(self, plateau_y_values):
        """step3 역산 높이가 30mm ± 7mm 이내여야 한다."""
        y_floor = plateau_y_values["floor"]
        y_step3 = plateau_y_values["step3"]
        h = estimate_height(y_floor, y_step3)
        assert h == pytest.approx(30.0, abs=7.0), (
            f"step3 실측 역산: {h:.2f}mm (기대 30mm ± 7mm)"
        )

    def test_step_heights_monotonically_increase(self, plateau_y_values):
        """floor → step5로 갈수록 역산 높이가 단조증가해야 한다."""
        y_floor = plateau_y_values["floor"]
        steps = ["step1", "step2", "step3", "step4", "step5"]
        heights = [estimate_height(y_floor, plateau_y_values[s]) for s in steps]
        for i in range(len(heights) - 1):
            assert heights[i] < heights[i + 1], (
                f"단조증가 위반: step{i+1}={heights[i]:.2f}mm >= step{i+2}={heights[i+1]:.2f}mm"
            )

    def test_floor_self_height_is_zero(self, plateau_y_values):
        """floor 기준점으로 floor 자신의 높이를 추정하면 0mm이어야 한다."""
        y_floor = plateau_y_values["floor"]
        h = estimate_height(y_floor, y_floor)
        assert h == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# detect_step_transitions — 합성 데이터
# ---------------------------------------------------------------------------

class TestDetectStepTransitionsSynthetic:
    """detect_step_transitions: 합성 5단 계단으로 감지 정확도 검증."""

    def test_detects_correct_number_of_steps(self):
        """노이즈 없는 5단 계단에서 5개 플랫 구간을 감지해야 한다."""
        z_vals, y_vals = _make_step_scan(n_steps=5, pts_per_step=5)
        steps = detect_step_transitions(z_vals, y_vals, threshold=3.0, min_flat_len=3)
        assert len(steps) == 5, f"감지된 계단 수: {len(steps)} (기대: 5)"

    def test_step_indices_are_zero_based_sequential(self):
        """step_index가 0부터 순차적으로 증가해야 한다."""
        z_vals, y_vals = _make_step_scan(n_steps=5, pts_per_step=5)
        steps = detect_step_transitions(z_vals, y_vals, threshold=3.0, min_flat_len=3)
        for i, step in enumerate(steps):
            assert step["step_index"] == i

    def test_flat_y_mean_matches_known_levels(self):
        """각 플랫 구간의 y_mean이 합성 계단 레벨과 2px 이내로 일치해야 한다."""
        y_floor = 773.7
        step_dy = 27.0
        z_vals, y_vals = _make_step_scan(
            n_steps=5, pts_per_step=5, y_floor=y_floor, step_delta_y=step_dy
        )
        steps = detect_step_transitions(z_vals, y_vals, threshold=3.0, min_flat_len=3)
        for i, step in enumerate(steps):
            expected_y = y_floor - i * step_dy
            assert step["flat_y_mean"] == pytest.approx(expected_y, abs=2.0), (
                f"step{i} y_mean={step['flat_y_mean']:.2f}px (기대 {expected_y:.1f}px)"
            )

    def test_transition_z_exists_between_adjacent_steps(self):
        """마지막 계단을 제외한 모든 계단에 transition_z가 None이 아니어야 한다."""
        z_vals, y_vals = _make_step_scan(n_steps=5, pts_per_step=5)
        steps = detect_step_transitions(z_vals, y_vals, threshold=3.0, min_flat_len=3)
        for step in steps[:-1]:
            assert step["transition_z"] is not None, (
                f"step{step['step_index']}의 transition_z가 None"
            )

    def test_last_step_transition_z_is_none(self):
        """마지막 계단의 transition_z는 None이어야 한다."""
        z_vals, y_vals = _make_step_scan(n_steps=5, pts_per_step=5)
        steps = detect_step_transitions(z_vals, y_vals, threshold=3.0, min_flat_len=3)
        assert steps[-1]["transition_z"] is None

    def test_flat_indices_are_sorted(self):
        """각 플랫 구간의 flat_indices가 오름차순이어야 한다."""
        z_vals, y_vals = _make_step_scan(n_steps=5, pts_per_step=5)
        steps = detect_step_transitions(z_vals, y_vals, threshold=3.0, min_flat_len=3)
        for step in steps:
            idx = step["flat_indices"]
            assert idx == sorted(idx), f"flat_indices가 정렬되지 않음: {idx}"

    def test_returns_empty_list_for_single_point(self):
        """입력이 1개 포인트이면 빈 리스트를 반환해야 한다."""
        result = detect_step_transitions([500.0], [773.7])
        assert result == []

    def test_returns_empty_list_for_empty_input(self):
        """빈 입력에 대해 빈 리스트를 반환해야 한다."""
        result = detect_step_transitions([], [])
        assert result == []

    def test_detects_single_plateau_in_flat_data(self):
        """변화가 없는 시퀀스에서 플랫 구간 1개를 감지해야 한다."""
        z_vals = list(range(10))
        y_vals = [773.7] * 10
        steps = detect_step_transitions(z_vals, y_vals, threshold=3.0, min_flat_len=3)
        assert len(steps) == 1

    def test_noisy_data_still_detects_5_steps(self):
        """1px 노이즈가 있어도 threshold=5로 5단을 감지해야 한다."""
        rng = np.random.default_rng(0)
        z_vals, y_vals = _make_step_scan(
            n_steps=5, pts_per_step=7, noise_std=1.0, rng=rng
        )
        steps = detect_step_transitions(z_vals, y_vals, threshold=5.0, min_flat_len=3)
        assert len(steps) == 5, f"노이즈 환경 감지 계단 수: {len(steps)}"

    def test_min_flat_len_filters_short_plateaus(self):
        """min_flat_len=10이면 pts_per_step=5 계단은 감지하지 않아야 한다."""
        z_vals, y_vals = _make_step_scan(n_steps=5, pts_per_step=5)
        steps = detect_step_transitions(
            z_vals, y_vals, threshold=3.0, min_flat_len=10
        )
        assert len(steps) == 0, (
            f"짧은 플랫 구간이 걸러지지 않음: {len(steps)}개 감지"
        )

    def test_flat_y_std_is_near_zero_on_noiseless_data(self):
        """노이즈 없는 데이터에서 flat_y_std가 0에 가까워야 한다."""
        z_vals, y_vals = _make_step_scan(n_steps=3, pts_per_step=5, noise_std=0.0)
        steps = detect_step_transitions(z_vals, y_vals, threshold=3.0, min_flat_len=3)
        for step in steps:
            assert step["flat_y_std"] == pytest.approx(0.0, abs=1e-6)

    def test_result_dict_has_required_keys(self):
        """반환 딕셔너리에 필수 키가 모두 포함되어야 한다."""
        z_vals, y_vals = _make_step_scan(n_steps=2, pts_per_step=5)
        steps = detect_step_transitions(z_vals, y_vals, threshold=3.0, min_flat_len=3)
        required = {"step_index", "flat_z_mean", "flat_y_mean",
                    "flat_y_std", "flat_indices", "transition_z"}
        for step in steps:
            assert required.issubset(step.keys()), (
                f"누락된 키: {required - set(step.keys())}"
            )


# ---------------------------------------------------------------------------
# detect_step_transitions — 실제 CSV 데이터
# ---------------------------------------------------------------------------

class TestDetectStepTransitionsRealData:
    """실제 스캔 CSV 데이터로 detect_step_transitions 검증."""

    @pytest.fixture(scope="class")
    def real_scan_series(self):
        """CSV에서 roi_laser의 (z_tcp, median_y) 시퀀스를 반환한다."""
        try:
            import pandas as pd
        except ImportError:
            pytest.skip("pandas 미설치")

        if not os.path.exists(CSV_PATH):
            pytest.skip(f"CSV 파일 없음: {CSV_PATH}")

        df = pd.read_csv(CSV_PATH)
        laser = df[df["roi_name"] == "roi_laser"]
        agg = (
            laser.groupby(["capture_idx", "z_tcp_mm"])["center_y_px"]
            .median()
            .reset_index()
        )
        agg = agg.sort_values("z_tcp_mm", ascending=False)  # 내림차순 z = 스캔 순서
        # 이상치 캡처(27, 32, 33, 34, 35) 제외 — wrap-around 아티팩트
        agg = agg[~agg["capture_idx"].isin([27, 32, 33, 34, 35])]
        return {
            "z": agg["z_tcp_mm"].tolist(),
            "y": agg["center_y_px"].tolist(),
        }

    def test_detects_at_least_4_plateau_groups(self, real_scan_series):
        """실데이터에서 최소 4개 이상의 plateau 그룹을 감지해야 한다.

        threshold=5px, min_flat_len=2로 실측 5단 계단에서 4개 이상을 감지한다.
        (일부 계단은 전이 노이즈로 인해 병합될 수 있다.)
        """
        steps = detect_step_transitions(
            real_scan_series["z"],
            real_scan_series["y"],
            threshold=5.0,
            min_flat_len=2,
        )
        assert len(steps) >= 4, (
            f"실데이터 plateau 감지 수: {len(steps)} (기대 >= 4)"
        )

    def test_highest_plateau_y_mean_above_740(self, real_scan_series):
        """가장 큰 y_mean을 가진 plateau가 740px 이상이어야 한다.

        실데이터에서 floor(~773px)와 step1(~746px) 일부가 병합되더라도
        최고 y_mean 그룹은 740px 이상을 유지해야 한다.
        """
        steps = detect_step_transitions(
            real_scan_series["z"],
            real_scan_series["y"],
            threshold=5.0,
            min_flat_len=2,
        )
        assert len(steps) >= 1
        highest = max(steps, key=lambda s: s["flat_y_mean"])
        assert highest["flat_y_mean"] >= 740.0, (
            f"최고 plateau y_mean={highest['flat_y_mean']:.2f}px (기대 >= 740px)"
        )


# ---------------------------------------------------------------------------
# calibrate_sensitivity
# ---------------------------------------------------------------------------

class TestCalibrateSensitivity:
    """calibrate_sensitivity: 파라미터 범위 및 기본 동작 검증."""

    def _make_scan_data(self, y_floor: float = 773.7, n_steps: int = 5,
                        rise_mm: float = 10.0, noise: float = 0.0,
                        rng=None) -> dict:
        """알려진 sensitivity 선형 모델로부터 합성 scan_data를 생성한다.

        calibrate_sensitivity는 y_px ~ sensitivity 선형 회귀를 수행한다.
        R²가 높으려면 sensitivity 값이 y_px에 따라 선형으로 변해야 한다.
        이를 위해 각 포인트마다 다른 y_base_px(바닥 Y 위치)를 사용해
        y_px 범위가 분산되도록 한다.
        """
        if rng is None:
            rng = np.random.default_rng(7)
        # 각 계단마다 y_base를 10px씩 달리해 y_px 범위를 분산시킨다
        y_bases = [y_floor - k * 10.0 for k in range(n_steps)]
        points = []
        for k, yb in enumerate(y_bases):
            h_mm = (k + 1) * rise_mm
            sens = SENS_SLOPE * yb + SENS_INTERCEPT
            y_px = yb - sens * h_mm
            if noise > 0:
                y_px += float(rng.normal(0, noise))
            points.append({
                "y_px": y_px,
                "height_mm": h_mm,
                "y_base_px": yb,
            })
        return {"points": points}

    def test_returns_dict_with_required_keys(self):
        """반환 딕셔너리에 필수 키가 모두 포함되어야 한다."""
        data = self._make_scan_data()
        result = calibrate_sensitivity(data)
        required = {"slope", "intercept", "r_squared", "n_points",
                    "sens_values", "y_values"}
        assert required.issubset(result.keys())

    def test_slope_is_negative(self):
        """sensitivity 기울기(slope)는 음수여야 한다 (모델 특성)."""
        data = self._make_scan_data(n_steps=5)
        result = calibrate_sensitivity(data)
        assert result["slope"] < 0, f"slope={result['slope']:.6f} (기대 < 0)"

    def test_intercept_is_positive(self):
        """intercept는 양수여야 한다 (0px에서의 sensitivity > 0)."""
        data = self._make_scan_data(n_steps=5)
        result = calibrate_sensitivity(data)
        assert result["intercept"] > 0, f"intercept={result['intercept']:.4f}"

    def test_r_squared_is_high_on_linear_model_data(self):
        """선형 모델에서 생성한 데이터에서 R²가 0.9 이상이어야 한다."""
        data = self._make_scan_data(n_steps=5, noise=0.0)
        result = calibrate_sensitivity(data)
        assert result["r_squared"] >= 0.9, (
            f"R²={result['r_squared']:.4f} (기대 >= 0.9)"
        )

    def test_r_squared_is_between_0_and_1(self):
        """R²는 항상 0 이상 1 이하여야 한다."""
        data = self._make_scan_data(n_steps=5, noise=1.0)
        result = calibrate_sensitivity(data)
        assert 0.0 <= result["r_squared"] <= 1.0

    def test_n_points_matches_input_count(self):
        """n_points가 유효 포인트 수(height_mm != 0)와 일치해야 한다."""
        data = self._make_scan_data(n_steps=4)
        result = calibrate_sensitivity(data)
        assert result["n_points"] == 4

    def test_sens_values_length_matches_n_points(self):
        """sens_values 길이가 n_points와 같아야 한다."""
        data = self._make_scan_data(n_steps=5)
        result = calibrate_sensitivity(data)
        assert len(result["sens_values"]) == result["n_points"]

    def test_y_values_length_matches_n_points(self):
        """y_values 길이가 n_points와 같아야 한다."""
        data = self._make_scan_data(n_steps=5)
        result = calibrate_sensitivity(data)
        assert len(result["y_values"]) == result["n_points"]

    def test_raises_value_error_when_fewer_than_2_points(self):
        """포인트가 1개 이하이면 ValueError를 발생시켜야 한다."""
        data = {"points": [{"y_px": 746.0, "height_mm": 10.0, "y_base_px": 773.7}]}
        with pytest.raises(ValueError, match="최소 2개"):
            calibrate_sensitivity(data)

    def test_raises_value_error_when_points_is_empty(self):
        """포인트 리스트가 비어 있으면 ValueError를 발생시켜야 한다."""
        with pytest.raises(ValueError):
            calibrate_sensitivity({"points": []})

    def test_raises_value_error_when_key_missing(self):
        """포인트에 'height_mm' 키가 없으면 ValueError를 발생시켜야 한다."""
        data = {"points": [
            {"y_px": 746.0},
            {"y_px": 719.0},
        ]}
        with pytest.raises(ValueError, match="height_mm"):
            calibrate_sensitivity(data)

    def test_skips_zero_height_points(self):
        """height_mm=0인 포인트(바닥 기준)는 sensitivity 계산에서 제외해야 한다."""
        data = {"points": [
            {"y_px": 773.7, "height_mm": 0.0, "y_base_px": 773.7},  # 제외
            {"y_px": 746.5, "height_mm": 10.0, "y_base_px": 773.7},
            {"y_px": 719.2, "height_mm": 20.0, "y_base_px": 773.7},
            {"y_px": 689.7, "height_mm": 30.0, "y_base_px": 773.7},
        ]}
        result = calibrate_sensitivity(data)
        assert result["n_points"] == 3, (
            f"height=0 포인트가 포함됨: n_points={result['n_points']}"
        )

    def test_slope_within_order_of_magnitude_of_known(self):
        """피팅된 slope가 알려진 SENS_SLOPE(≈-0.0055)의 10배 이내여야 한다."""
        data = self._make_scan_data(n_steps=5, noise=0.0)
        result = calibrate_sensitivity(data)
        assert abs(result["slope"]) < abs(SENS_SLOPE) * 10 + 0.01, (
            f"slope={result['slope']:.6f}가 예상 범위 벗어남"
        )

    def test_intercept_within_plausible_range(self):
        """intercept가 물리적으로 타당한 범위(0 < intercept < 20 px/mm)여야 한다."""
        data = self._make_scan_data(n_steps=5)
        result = calibrate_sensitivity(data)
        assert 0 < result["intercept"] < 20.0, (
            f"intercept={result['intercept']:.4f} out of range"
        )


# ---------------------------------------------------------------------------
# calibrate_sensitivity — 실제 CSV 데이터
# ---------------------------------------------------------------------------

class TestCalibrateSensitivityRealData:
    """실제 측정 plateau Y값으로 calibrate_sensitivity 검증."""

    @pytest.fixture(scope="class")
    def real_scan_data(self):
        """CSV plateau 중앙값을 사용한 scan_data 딕셔너리를 반환한다."""
        try:
            import pandas as pd
        except ImportError:
            pytest.skip("pandas 미설치")

        if not os.path.exists(CSV_PATH):
            pytest.skip(f"CSV 파일 없음: {CSV_PATH}")

        df = pd.read_csv(CSV_PATH)
        laser = df[df["roi_name"] == "roi_laser"]

        def median_y(capture_ids):
            return float(
                laser[laser["capture_idx"].isin(capture_ids)]["center_y_px"].median()
            )

        y_floor = median_y([0, 1])
        plateau_map = {
            10: median_y([3, 4, 5, 6]),
            20: median_y([10, 11, 12, 13]),
            30: median_y([17, 18, 19]),
            40: median_y([22, 23, 24, 25, 26]),
            50: median_y([29, 30, 31]),
        }

        points = [{"y_px": y_floor, "height_mm": 0.0, "y_base_px": y_floor}]
        for h_mm, y_px in plateau_map.items():
            points.append({"y_px": y_px, "height_mm": h_mm, "y_base_px": y_floor})

        return {"points": points}

    def test_r_squared_above_0_8_on_real_data(self, real_scan_data):
        """실제 지그 데이터에서 피팅 R²가 0.8 이상이어야 한다."""
        result = calibrate_sensitivity(real_scan_data)
        assert result["r_squared"] >= 0.8, (
            f"실데이터 R²={result['r_squared']:.4f} (기대 >= 0.8)"
        )

    def test_slope_is_negative_on_real_data(self, real_scan_data):
        """실데이터 피팅 slope가 음수여야 한다."""
        result = calibrate_sensitivity(real_scan_data)
        assert result["slope"] < 0

    def test_intercept_is_positive_on_real_data(self, real_scan_data):
        """실데이터 피팅 intercept가 양수여야 한다."""
        result = calibrate_sensitivity(real_scan_data)
        assert result["intercept"] > 0

    def test_n_points_excludes_zero_height(self, real_scan_data):
        """height_mm=0 바닥 기준점을 제외한 포인트 수(5)를 반환해야 한다."""
        result = calibrate_sensitivity(real_scan_data)
        assert result["n_points"] == 5


# ---------------------------------------------------------------------------
# 상수 무결성
# ---------------------------------------------------------------------------

class TestModuleConstants:
    """모듈 상수가 물리적으로 유효한 범위에 있는지 검증한다."""

    def test_fy_px_is_positive(self):
        """FY_PX(초점거리)는 양수여야 한다."""
        assert FY_PX > 0

    def test_laser_alpha_deg_is_in_valid_range(self):
        """LASER_ALPHA_DEG는 0°~90° 사이여야 한다."""
        assert 0.0 < LASER_ALPHA_DEG < 90.0

    def test_delta_z_mm_is_positive(self):
        """DELTA_Z_MM은 양수여야 한다 (보정 거리 > 0)."""
        assert DELTA_Z_MM > 0

    def test_sens_slope_is_negative(self):
        """SENS_SLOPE는 음수여야 한다 (y 증가 → sensitivity 감소)."""
        assert SENS_SLOPE < 0

    def test_sens_intercept_is_positive(self):
        """SENS_INTERCEPT는 양수여야 한다."""
        assert SENS_INTERCEPT > 0

    def test_bx_mm_is_nonzero(self):
        """BX_MM(baseline)은 0이 아니어야 한다."""
        assert BX_MM != 0.0

    def test_cy_px_is_near_image_center(self):
        """CY_PX는 이미지 높이(1080px)의 절반 근처(240–540px)여야 한다."""
        assert 240.0 <= CY_PX <= 540.0
