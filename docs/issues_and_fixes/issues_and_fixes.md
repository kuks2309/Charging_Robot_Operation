# Issues and Fixes Log

---

## 2026-02-07 | TF5 툴프레임 설정 오류 수정

**증상:** 모션 테스트 탭에서 TF5 라디오 버튼 선택 시 "툴프레임 설정 실패: 잘못된 툴프레임 번호: 5 (0-4만 가능)" 오류 발생

**원인:** `scripts/Robot/communication/modbus_client.py`의 `send_set_toolframe()` 메서드에서 frame=5 케이스가 누락됨. `CMD_TOOLFRAME_5 = 46`은 이미 정의되어 있었으나 validation 로직에 포함되지 않음.

**수정 파일:** `scripts/Robot/communication/modbus_client.py` (line 512-515)

**수정 내용:**
- `elif frame == 5: cmd = self.CMD_TOOLFRAME_5` 분기 추가
- 에러 메시지 범위 "0-4만 가능" → "0-5만 가능" 변경
- 독스트링 업데이트: 유효 범위 0-5, TF5는 Hand-Eye Calibration 용도 명시

**확인:** 로봇 티치펜던트에서 WO T5 (X=29.23, Y=80.24, Z=23.40) 존재 확인됨
