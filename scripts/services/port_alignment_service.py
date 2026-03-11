"""port_alignment_service.py — AI 검출 기반 포트 중심 정렬 알고리즘

YOLOv8-seg 검출 결과(Det 리스트)를 입력받아 이미지 중심 대비
포트 중심의 오프셋을 계산하는 순수 함수들을 제공한다.

UI / 로봇 제어 코드와 완전히 분리된 알고리즘 레이어.
"""
import math
import time
from itertools import combinations
from typing import Callable, Optional


def compute_vertical_alignment(dets, img_h):
    """커플러 기구학적 중심과 이미지 수평 중심선 간 수직 오프셋 계산.

    커플러 배치:
        ○ Circle_A (좌)  ○ Circle_A (우)   ← 2개 동일 높이
                 ○ Circle_A (하)            ← 커플러 기구학적 중심 ≈ 이미지 중심선
        □ Port_T (상)
        □ Port_T (하)

    수직 정렬 기준 (우선순위):
      1차: Circle_A 하단 (Y 최대값) — 커플러 기구학적 중심에 가장 근접
      2차: Port_T 2개 Y 평균 — Circle_A 미검출 시 fallback

    수직 정렬 목표: dy == 0  (기준점 Y == 이미지 중심 Y)

    Args:
        dets: parse_detections() 반환 Det 리스트 (cls, cx, cy, conf)
        img_h: 원본 프레임 높이 (px)

    Returns:
        (ref_cy, img_cy, dy_px)
            dy_px > 0 : 기준점이 이미지 중심보다 아래
            dy_px < 0 : 기준점이 이미지 중심보다 위
        Circle_A·Port_T 모두 미검출 시 None
    """
    circles = sorted([d for d in dets if d.cls == 'Circle_A'], key=lambda d: d.cy)
    if circles:
        ref_cy = circles[-1].cy  # Y 최대 = 하단 원 = 커플러 기구학적 중심
    else:
        ports = sorted([d for d in dets if d.cls == 'Port_T'],
                       key=lambda d: d.conf, reverse=True)[:2]
        if not ports:
            return None
        ref_cy = sum(d.cy for d in ports) / len(ports)
    img_cy = img_h / 2.0
    dy_px  = ref_cy - img_cy
    return ref_cy, img_cy, dy_px


def compute_horizontal_alignment(dets, img_w):
    """커플러 기구학적 중심과 이미지 수직 중심선 간 수평 오프셋 계산.

    수평 정렬 기준 (우선순위):
      1차: Circle_A 전체 cx 평균 — 좌/우/하 3원의 centroid X = 수평 중심
      2차: Port_T confidence top-2 cx 평균 — Circle_A 미검출 시 fallback

    수평 정렬 목표: dx == 0  (기준점 X == 이미지 중심 X)

    Args:
        dets: parse_detections() 반환 Det 리스트 (cls, cx, cy, conf)
        img_w: 원본 프레임 너비 (px)

    Returns:
        (ref_cx, img_cx, dx_px)
            dx_px > 0 : 기준점이 이미지 중심보다 오른쪽
            dx_px < 0 : 기준점이 이미지 중심보다 왼쪽
        Circle_A·Port_T 모두 미검출 시 None
    """
    circles = [d for d in dets if d.cls == 'Circle_A']
    if circles:
        ref_cx = sum(d.cx for d in circles) / len(circles)
    else:
        ports = sorted([d for d in dets if d.cls == 'Port_T'],
                       key=lambda d: d.conf, reverse=True)[:2]
        if not ports:
            return None
        ref_cx = sum(d.cx for d in ports) / len(ports)
    img_cx = img_w / 2.0
    dx_px  = ref_cx - img_cx
    return ref_cx, img_cx, dx_px


def compute_ry_angle(dets) -> 'Optional[float]':
    """YOLO 검출 기반 Ry 기울기 각도 계산.

    소스 (우선 사용 가능한 소스 모두 계산 후 outlier 필터 → 평균):
      H: Circle_A 수평 가장 넓은 쌍 (≥2개)
         angle_h = -degrees(atan2(dy, dx))
      S: Circle_A 삼각형 대칭축 (정확히 3개)
         mid = 상위 2개 중점, bottom = 최하단
         angle_s = -degrees(atan2(bottom.cx - mid_cx, bottom.cy - mid_cy))
      V: Port_T 수직 쌍 (≥2개)
         angle_v = -degrees(atan2(dx, dy)) where dx = bottom.cx - top.cx, dy = bottom.cy - top.cy

    Outlier filter (≥2 소스): median ±5° 초과 제거
    반환: 생존 소스 평균 (degrees) 또는 None (소스 없음)
    """
    circles = [d for d in dets if d.cls == 'Circle_A']
    ports   = [d for d in dets if d.cls == 'Port_T']
    angles  = {}

    # H source
    if len(circles) >= 2:
        best_pair = max(combinations(circles, 2),
                        key=lambda p: abs(p[0].cx - p[1].cx))
        ca, cb = best_pair
        dx = cb.cx - ca.cx
        dy = cb.cy - ca.cy
        angles['H'] = -math.degrees(math.atan2(dy, dx))

    # S source
    if len(circles) == 3:
        sorted_c = sorted(circles, key=lambda d: d.cy)
        top1, top2, bottom = sorted_c[0], sorted_c[1], sorted_c[2]
        mid_cx = (top1.cx + top2.cx) / 2
        mid_cy = (top1.cy + top2.cy) / 2
        dx = bottom.cx - mid_cx
        dy = bottom.cy - mid_cy
        angles['S'] = -math.degrees(math.atan2(dx, dy))

    # V source
    if len(ports) >= 2:
        pt_top    = min(ports, key=lambda d: d.cy)
        pt_bottom = max(ports, key=lambda d: d.cy)
        dx = pt_bottom.cx - pt_top.cx
        dy = pt_bottom.cy - pt_top.cy
        angles['V'] = -math.degrees(math.atan2(dx, dy))

    if not angles:
        return None

    # Outlier filter
    if len(angles) >= 2:
        vals = list(angles.values())
        median_val = sorted(vals)[len(vals) // 2]
        filtered = {k: v for k, v in angles.items() if abs(v - median_val) <= 5.0}
        if filtered:
            angles = filtered

    valid_keys = list(angles.keys())
    return sum(angles[k] for k in valid_keys) / len(valid_keys)


class PortAlignmentService:
    """AI 검출 기반 포트 정렬 서비스 — 수직/수평/Ry (UI 독립 레이어).

    NOTE: 호출 전 TF4가 설정되어 있어야 함 (호출자 책임).
          다른 정렬 서비스와 동일하게 set_robot()으로 robot 주입.
    """

    COARSE_TEST_MM  = 2.0    # px/mm 실측 테스트 이동량
    COARSE_MAX_MM   = 20.0   # coarse 1회 최대 보정량 안전 한계
    DEAD_ZONE_PX    = 3.0    # 이 이하이면 정렬 불필요

    FINE_MAX_STEP_MM  = 0.5   # 미세 정렬 1회 최대 이동량 (보수적 시작)
    FINE_MIN_STEP_MM  = 0.05  # 로봇 해상도 하한 (이하이면 의미 없음)
    FINE_CONVERGE_PX  = 1.5   # 수렴 판정 임계값 (px)
    FINE_ENTRY_PX     = 10.0  # 미세 정렬 진입 조건 (coarse 잔여 < 이 값)
    FINE_MAX_ITER     = 15    # 최대 반복 횟수
    FINE_SETTLE_S     = 0.35  # 미세 이동 후 안정화 대기
    MAX_RY_CORRECTION_DEG = 10.0  # Ry 1회 보정 최대 각도 안전 한계

    def __init__(self, log_fn=None):
        self.robot = None
        self._log = log_fn or (lambda _: None)

    def set_robot(self, robot):
        """로봇 클라이언트 주입 (main_window 연결/해제 시 호출)."""
        self.robot = robot

    def align_vertical(
        self,
        measure_fn: Callable[[], Optional[float]],
        tcp_axis: str = 'y',
    ) -> tuple:
        """수직 정렬 실행 (coarse → fine 자동 수행).

        coarse 단계:
          1. d0 측정
          2. +COARSE_TEST_MM 테스트 이동
          3. d1 측정 → px_per_mm = (d1-d0) / COARSE_TEST_MM
          4. correction_mm = -d1 / px_per_mm 보정 이동

        fine 단계 (_fine_align):
          잔여 오프셋이 FINE_ENTRY_PX 이하일 때 proportional correction 반복

        Args:
            measure_fn: () -> Optional[float]
                현재 dy_px를 반환하는 콜백. None=검출 실패.
                내부적으로 충분한 대기(3-phase wait 등) 후 반환해야 함.
            tcp_axis: 이동 TCP 축 ('y' 기본). TF4 기준 수직 = TCP Y.

        Returns:
            (success: bool, message: str)
        """
        if self.robot is None or not self.robot.is_connected:
            return False, "로봇 미연결"

        # 초기 오프셋
        d0 = measure_fn()
        if d0 is None:
            return False, "초기 검출 실패"

        if abs(d0) < self.DEAD_ZONE_PX:
            return True, f"이미 정렬됨 (잔여={d0:+.1f}px)"

        self._log(f"[PortAlign] 1단계: d0={d0:+.1f}px, 테스트 이동 +{self.COARSE_TEST_MM}mm")

        # 테스트 이동
        ok, msg = self.robot.send_tcp_linear(tcp_axis, self.COARSE_TEST_MM, wait=True)
        if not ok:
            return False, f"테스트 이동 실패: {msg}"

        # 테스트 이동 후 검출
        d1 = measure_fn()
        if d1 is None:
            self.robot.send_tcp_linear(tcp_axis, -self.COARSE_TEST_MM, wait=True)
            return False, "테스트 이동 후 검출 실패 — 원위치 복귀"

        delta_px = d1 - d0
        self._log(f"[PortAlign] d1={d1:+.1f}px, 변화={delta_px:+.1f}px")

        if abs(delta_px) < 2:
            self._log("[PortAlign] 픽셀 변화 < 2px — 원위치 복귀")
            self.robot.send_tcp_linear(tcp_axis, -self.COARSE_TEST_MM, wait=True)
            return False, "측정 불안정 (픽셀 변화 < 2px)"

        px_per_mm = delta_px / self.COARSE_TEST_MM
        correction_mm = -d1 / px_per_mm
        self._log(
            f"[PortAlign] {px_per_mm:.2f} px/mm ({abs(1/px_per_mm):.3f} mm/px)"
            f" → 보정 {correction_mm:+.2f}mm"
        )

        if abs(correction_mm) > self.COARSE_MAX_MM:
            return False, f"보정 과대 ({correction_mm:.1f}mm > ±{self.COARSE_MAX_MM}mm)"

        ok, msg = self.robot.send_tcp_linear(tcp_axis, correction_mm, wait=True)
        if not ok:
            return False, f"보정 이동 실패: {msg}"

        # 미세 정렬
        final_px = self._fine_align(tcp_axis, px_per_mm, measure_fn)

        if final_px is not None:
            return True, f"정렬 완료: 잔여={final_px:+.1f}px"
        return True, "정렬 완료 (미세 정렬 측정 불가)"

    def align_ry(
        self,
        measure_fn: 'Callable[[], Optional[float]]',
        threshold_deg: float = 0.5,
    ) -> tuple:
        """Ry 각도 보정 (반복 루프, 최대 5회).

        단계 (매 반복):
          1. measure_fn()으로 현재 Ry 각도 측정
          2. |angle| < threshold_deg → 정렬 완료
          3. |angle| > MAX_RY_CORRECTION_DEG → 클램프 후 경고 로그
          4. send_base_rotate('ry', -angle) 보정
          5. threshold 미달이면 다음 반복

        NOTE: TF4 설정은 호출자 책임.

        Args:
            measure_fn: () -> Optional[float]
                현재 Ry 기울기 각도(degrees)를 반환하는 콜백.
                내부적으로 3-phase wait 후 반환 권장.
            threshold_deg: 이 이하면 정렬 완료 (기본 0.5°)

        Returns:
            (success: bool, message: str)
        """
        if self.robot is None or not self.robot.is_connected:
            return False, "로봇 미연결"

        MAX_ITER = 5
        for iteration in range(1, MAX_ITER + 1):
            angle = measure_fn()
            if angle is None:
                return False, f"Ry 측정 실패 (검출 없음, 반복 {iteration})"

            self._log(f"[PortAlign][Ry] 측정 [{iteration}/{MAX_ITER}]: {angle:+.2f}°")

            if abs(angle) < threshold_deg:
                return True, f"Ry 정렬 완료: 잔여={angle:+.2f}° (반복 {iteration}회)"

            # 안전 클램프
            correction = -angle
            if abs(correction) > self.MAX_RY_CORRECTION_DEG:
                correction = math.copysign(self.MAX_RY_CORRECTION_DEG, correction)
                self._log(
                    f"[PortAlign][Ry] 경고: 보정량 클램프 → {correction:+.2f}°"
                )

            self._log(f"[PortAlign][Ry] 보정 [{iteration}]: send_base_rotate('ry', {correction:+.2f}°)")
            ok, msg = self.robot.send_base_rotate('ry', correction, wait=True)
            if not ok:
                return False, f"Ry 이동 실패: {msg}"

        # 최종 잔여 확인
        residual = measure_fn()
        if residual is not None:
            self._log(f"[PortAlign][Ry] 최종 잔여: {residual:+.2f}° ({MAX_ITER}회 완료)")
            return True, f"Ry 보정 완료: 잔여={residual:+.2f}° ({MAX_ITER}회)"
        return True, f"Ry 보정 완료 ({MAX_ITER}회, 잔여 측정 불가)"

    def align_horizontal(
        self,
        measure_fn: Callable[[], Optional[float]],
    ) -> tuple:
        """수평 정렬 실행 — align_vertical(tcp_axis='x') 래퍼.

        커플러 수평 중심(compute_horizontal_alignment 기준)을
        이미지 수평 중심(img_w/2)에 맞춘다.

        coarse+fine 알고리즘은 align_vertical과 동일.
        TCP X 이동(tool.transx, TF4 기준)으로 수평 방향 보정.

        NOTE: TF4 설정은 호출자 책임.

        Args:
            measure_fn: () -> Optional[float]
                현재 dx_px를 반환하는 콜백.
                내부적으로 3-phase wait 후 반환 권장.

        Returns:
            (success: bool, message: str)
        """
        return self.align_vertical(measure_fn, tcp_axis='x')

    def _fine_align(
        self,
        tcp_axis: str,
        px_per_mm: float,
        measure_fn: Callable[[], Optional[float]],
    ) -> Optional[float]:
        """미세 정렬: coarse 잔여를 FINE_CONVERGE_PX 이내로 수렴.

        proportional correction: step = clamp(-offset/px_per_mm, ±FINE_MAX_STEP_MM)
        부호 반전 2회 감지 시 발산 중단.
        """
        if abs(px_per_mm) < 8.0:
            self._log(f"[PortAlign][Fine] px/mm={px_per_mm:.1f} < 8, 건너뜀")
            return None

        offset = measure_fn()
        if offset is None:
            self._log("[PortAlign][Fine] 초기 측정 실패")
            return None

        if abs(offset) > self.FINE_ENTRY_PX:
            self._log(f"[PortAlign][Fine] 잔여={offset:+.1f}px > {self.FINE_ENTRY_PX}px, 건너뜀")
            return offset

        self._log(f"[PortAlign][Fine] 시작: 잔여={offset:+.1f}px")

        prev_sign = None
        reversal_count = 0

        for i in range(self.FINE_MAX_ITER):
            if abs(offset) <= self.FINE_CONVERGE_PX:
                self._log(f"[PortAlign][Fine] 수렴: {offset:+.1f}px (반복 {i})")
                return offset

            # proportional correction, 최소/최대 이동량 클램프
            raw_step = -offset / px_per_mm
            step_mm = max(-self.FINE_MAX_STEP_MM, min(self.FINE_MAX_STEP_MM, raw_step))
            if abs(step_mm) < self.FINE_MIN_STEP_MM:
                self._log(f"[PortAlign][Fine] 스텝 {step_mm:.3f}mm < {self.FINE_MIN_STEP_MM}mm, 종료")
                return offset

            ok, msg = self.robot.send_tcp_linear(tcp_axis, step_mm, wait=True)
            if not ok:
                self._log(f"[PortAlign][Fine] 이동 실패: {msg}")
                break

            time.sleep(self.FINE_SETTLE_S)

            new_offset = measure_fn()
            if new_offset is None:
                self._log(f"[PortAlign][Fine] 측정 실패 (반복 {i+1})")
                break

            # 부호 반전 감지 (발산 방지)
            curr_sign = 1 if new_offset > 0 else (-1 if new_offset < 0 else 0)
            if curr_sign != 0 and prev_sign is not None and prev_sign != 0:
                if curr_sign != prev_sign:
                    reversal_count += 1
                    if reversal_count >= 2:
                        self._log(f"[PortAlign][Fine] 부호 반전 2회, 발산 중단: {new_offset:+.1f}px")
                        return new_offset
                else:
                    reversal_count = 0
            prev_sign = curr_sign
            offset = new_offset

        self._log(f"[PortAlign][Fine] 종료: 잔여={offset:+.1f}px")
        return offset
