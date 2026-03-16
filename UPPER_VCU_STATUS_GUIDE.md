# upper_vcu_st 의미 설명서

이 문서는 `upper_vcu_st`(VCU 내부 상태)의 각 멤버가 어떤 의미인지, 상위 제어기에서 어떻게 해석하면 되는지 예시 중심으로 설명합니다.

## 1) `upper_vcu_st`란?
- VCU가 현재 판단한 운전 상태를 담는 구조체입니다.
- 이 값은 CAN `0x18FF0310` 프레임으로 상위 제어기에 전달됩니다.

---

## 2) 멤버별 의미

### `control_src`
- 현재 제어 권한이 어디에 있는지 표시
- 주의: 이 값은 `0x18FF0310` payload에 직접 들어가는 필드는 아니며, 내부 상태를 기반으로 `vcu_fsm_status_mask`를 구성할 때 사용됩니다.
- 값 의미
  - `0`: 정지/무제어
  - `1`: RC 제어
  - `2`: Upper 제어

예시:
- `control_src=1`이면 조종기(RC) 입력으로 주행 중임

### `stop_reason`
- 정지 원인 코드
- 주의: 이 값도 payload 직접 필드가 아니라, `vcu_fsm_status_mask` 해석 보조용 내부 상태입니다.
- 값 의미
  - `0`: 정지 아님(정상 제어 중)
  - `1`: upper 강제 정지
  - `2`: RC 비상정지
  - `3`: 모터 드라이버 fault
  - `4`: timeout(입력 신선도 문제)

예시:
- `stop_reason=2`이면 RC E-stop이 눌려 정지 상태

### `power_supply_value`
- 전원 관련 값(현재 구현은 left driver 전압값 기반, 범위 `-500~500`으로 클램프)
- CAN `0x18FF0310`의 `data[0:1]`로 전달

예시:
- `power_supply_value=320`이면 전원 상태가 정상 범위에서 높음 쪽으로 해석 가능(프로젝트 단위 기준)

### `md_left_fault_msg`
- left 모터 드라이버 fault 코드
- CAN `data[2]`

예시:
- `md_left_fault_msg=0` -> left 드라이버 fault 없음
- `md_left_fault_msg!=0` -> left 드라이버 경고/에러 존재

### `md_right_fault_msg`
- right 모터 드라이버 fault 코드
- CAN `data[3]`

예시:
- `md_right_fault_msg=0x04` -> right 드라이버 특정 fault 비트 활성

### `rc_status_mask`
- RC 상태를 bit mask로 표시
- CAN `data[4]`
- 비트 정의:
  - bit0: `RC_ST_ENABLE`
  - bit1: `RC_ST_EMERGENCY_STOP`
  - bit2: `RC_ST_FAILSAFE`
  - bit3: `RC_ST_FRESH`
  - bit4: `RC_ST_CULTIVATOR_DOWN`
  - bit5: `RC_ST_CULTIVATOR_ON`

예시:
- `rc_status_mask = 0b00001001` (0x09)
  - ENABLE=1, FRESH=1
  - 나머지 비트 0
- `rc_status_mask = 0b00010110` (0x16)
  - E-stop=1, failsafe=1, cultivator_down=1

### `vcu_fsm_status_mask`
- VCU FSM 상태를 bit mask로 표시
- CAN `data[5]`
- 비트 정의:
  - bit0: `VCU_ST_SRC_NONE`
  - bit1: `VCU_ST_SRC_RC`
  - bit2: `VCU_ST_SRC_UPPER`
  - bit3: `VCU_ST_STOP_UPPER`
  - bit4: `VCU_ST_STOP_RC_EMG`
  - bit5: `VCU_ST_STOP_MOTOR_FAULT`
  - bit6: `VCU_ST_STOP_TIMEOUT`
  - bit7: `VCU_ST_RUNNING`

예시:
- `vcu_fsm_status_mask = 0b10000010` (0x82)
  - SRC_RC + RUNNING
- `vcu_fsm_status_mask = 0b00010001` (0x11)
  - SRC_NONE + STOP_RC_EMG

### `relay_st`
- 릴레이 동작 상태(bit mask)
- CAN `data[6]`

예시:
- `relay_st=0x05` -> bit0, bit2 릴레이 ON

---

## 3) CAN 프레임 매핑(`0x18FF0310`)
- `data[0:1]` = `power_supply_value`
- `data[2]` = `md_left_fault_msg`
- `data[3]` = `md_right_fault_msg`
- `data[4]` = `rc_status_mask`
- `data[5]` = `vcu_fsm_status_mask`
- `data[6]` = `relay_st`
- `data[7]` = reserved (`0`)

---

## 4) 상태 해석 빠른 예시

### 예시 A: RC로 정상 주행 중
- `control_src=1`
- `stop_reason=0`
- `rc_status_mask=0x09` (enable + fresh)
- `vcu_fsm_status_mask=0x82` (SRC_RC + RUNNING)

해석:
- RC 입력이 살아있고, 비상정지 없이 주행 중

### 예시 B: upper 강제 정지
- `control_src=0`
- `stop_reason=1`
- `vcu_fsm_status_mask`에 `SRC_NONE + STOP_UPPER`

해석:
- Upper 명령으로 즉시 정지 상태

### 예시 C: 모터 fault로 정지
- `stop_reason=3`
- `md_left_fault_msg` 또는 `md_right_fault_msg`가 0이 아님
- `vcu_fsm_status_mask`에 `STOP_MOTOR_FAULT`

해석:
- VCU가 모터 드라이버 fault를 감지해 주행 차단
