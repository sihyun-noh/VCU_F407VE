---
document_type: can_interface_spec
project_name: AGMO 한국 쓰리축
topic: Upper Controller CAN Interface Spec
status: Draft
updated: 2026-05-22
release_version: AGMO-VCU-CAN-2026.05.22-DRAFT
owner: VCU Gateway
---

# Upper Controller CAN Interface Spec

> 대상: Upper Controller 개발팀
> 범위: Upper Controller <-> VCU Gateway CAN 통신 명세
> 기준 코드: `user/vcu_gateway.c`, `user/vcu_gateway.h`
> 상태: 현재 코드 기준 Draft

> 릴리즈 버전: `AGMO-VCU-CAN-2026.05.22-DRAFT`
> 릴리즈 기준일: 2026-05-22
> 전달 목적: 상위제어기 개발팀 구현/검증용 CAN 인터페이스 기준 문서

## 1. Common Rules

| 항목 | 값 |
| ---- | -- |
| CAN speed | 500 kbps |
| CAN frame | Extended ID |
| DLC | 기본 8 bytes |
| Multi-byte endian | Big-endian, MSB first |
| signed 값 | two's complement |
| Gateway status period | 200ms |
| Drive CMD period | 200ms 권장, timeout 발생 시 STOP |

Endian 예시:

| 값      | 타입        | Hex byte |
| ------ | --------- | -------- |
| `+50`  | int16 BE  | `00 32`  |
| `-50`  | int16 BE  | `FF CE`  |
| `300`  | uint16 BE | `01 2C`  |
| `500`  | uint16 BE | `01 F4`  |
| `1500` | uint16 BE | `05 DC`  |

## 2. CAN ID Summary

### 2-1. Upper -> Gateway RX

| CAN ID       | Name              | Period  | DLC | 설명                                        |
| ------------ | ----------------- | ------- | --- | -------------------------------------------------- |
| `0x18FF0200` | Drive CMD         | 200ms   | 8   | 상위 주행 명령, 현재 Auto 주행 기본 명령                         |
| `0x18FF0210` | Config CMD        | 변경 시 1회 | 8   | automation, force stop, force active, relay, accel, drive cmd select |
| `0x18FF0220` | Auto Direct Drive CMD | 선택 시 200ms 권장 | 8   | 좌/우 driver 직접 입력, mixer bypass |
| `0x18FF0230` | Weed Actuator CMD | 이벤트성    | 8   | actuator 목표 위치 설정/이동/정지                            |
| `0x18FF0240` | Weed Blade CMD    | 이벤트성    | 8   | blade RPM 설정/RUN/STOP                              |

### 2-2. Gateway -> Upper TX

| CAN ID | Name | Period | DLC | 설명 |
| ------ | ---- | ------ | --- | ----------- |
| `0x18FF0300` | Motor Status RPM | 200ms | 8 | 모터 드라이버 RPM 피드백 |
| `0x18FF0310` | Gateway Status | 200ms | 8 | VCU/RC/FSM/timeout 상태 |
| `0x18FF0320` | Weed Actuator Status | 200ms | 8 | actuator 상태/위치/error |
| `0x18FF0330` | Weed Blade Status | 200ms | 8 | blade 상태/fault/RPM |
| `0x18FF4000` | Vehicle Motion | 200ms | 8 | 차량 motion monitor, test/debug |
| `0x18FF4010` | Vehicle Monitor | 200ms | 8 | mixer/debug monitor, test/debug |

## 3. Upper -> Gateway RX Detail

## 3-1. `0x18FF0200` Drive CMD

현재 자동주행 주행 명령은 이 CAN ID를 기준으로 동작한다. Upper automation 상태에서 이 메시지가 timeout되면 VCU는 상위 주행 명령을 신뢰하지 않고 STOP 처리한다.

| Byte | Name | Type | Endian | 단위/범위 | 설명 |
| ---- | ---- | ---- | ------ | ---------- | ----------- |
| data[0:1] | `throttle_cmd` | int16 | BE | -500 ~ 500 | 전진/후진 정규화 명령 |
| data[2:3] | `steering_cmd` | int16 | BE | -500 ~ 500 | 좌/우 조향 정규화 명령 |
| data[4:5] | `max_driver_input_cmd` | uint16 | BE | 0 ~ 2000 권장 | 정규화 500일 때 driver 입력 기준값 |
| data[6:7] | `max_speed_kmh_x100` | uint16 | BE | km/h x 100 | 정규화 500일 때 기준 최고 속도 |

정규화 의미:

```text
driver_input = max_driver_input_cmd * (throttle_cmd / 500)
estimated_speed_kmh = (max_speed_kmh_x100 / 100.0) * (throttle_cmd / 500)
```

주의:

- `max_speed_kmh_x100`는 `km/h x 100`이다.
- 3.00km/h는 `300`을 전송한다. `3`을 전송하면 0.03km/h로 해석된다.
- 5.00km/h는 `500`을 전송한다.

예시, 3.00km/h 기준에서 전진 50%, 직진:

| 항목 | 10진수 | Hex |
| ----- | ------- | --- |
| `throttle_cmd` | `250` | `00 FA` |
| `steering_cmd` | `0` | `00 00` |
| `max_driver_input_cmd` | `396` | `01 8C` |
| `max_speed_kmh_x100` | `300` | `01 2C` |

Payload:

```text
00 FA 00 00 01 8C 01 2C
```

결과 의미:

```text
실제 driver 입력 = 396 * (250 / 500) = 198
예상 속도 = 3.00km/h * (250 / 500) = 1.50km/h
```

## 3-2. `0x18FF0210` Config CMD

공통 config 명령이다. actuator/blade 직접 명령은 포함하지 않는다. actuator는 `0x18FF0230`, blade는 `0x18FF0240`을 사용한다.

| Byte | Name | Type | Endian | 단위/범위 | 설명 |
| ---- | ---- | ---- | ------ | ---------- | ----------- |
| data[0] | `automation` | uint8 | - | bit0 | Upper automation 요청 |
| data[1] | `upper_force_stop` | uint8 | - | bit0 | 전체 강제 정지 |
| data[2] | `upper_force_active` | uint8 | - | bit0 | Upper 강제 제어 요청 |
| data[3] | `relay_mask` | uint8 | - | bit mask | relay 명령 |
| data[4] | `left_accel_cmd` | uint8 | - | 0 ~ 255 | left motor 가속도 설정값 |
| data[5] | `right_accel_cmd` | uint8 | - | 0 ~ 255 | right motor 가속도 설정값 |
| data[6] | `upper_drive_cmd_select` | uint8 | - | bit0 | `0`=`0x18FF0200` 사용, `1`=`0x18FF0220` 사용 |
| data[7] | Reserved | uint8 | - | - | 0 권장 |

예시, automation ON, accel 100:

```text
01 00 00 00 64 64 00 00
```

정책:

- Config CMD는 변경 시 1회 전송 성격이다.
- 현재 VCU FSM은 config timeout을 hard gate로 사용하지 않는다.
- 자동주행 진입은 RC remote automation ON과 Upper automation ON이 함께 필요하다.
- `data[6] bit0`은 Upper 주행 명령 경로를 선택한다. 기본 `0`은 기존 `0x18FF0200`, set `1`은 `0x18FF0220` direct 명령을 사용한다.

## 3-3. `0x18FF0220` Auto Direct Drive CMD

`0x18FF0210 data[6] bit0`이 `1`일 때 사용하는 좌/우 driver 직접 입력 명령이다. VCU mixer를 bypass하지만, 실제 설치 방향 부호는 VCU가 적용한다.

| Byte | Name | Type | Endian | 단위/범위 | 설명 |
| ---- | ---- | ---- | ------ | ---------- | ----------- |
| data[0:1] | `left_driver_input_cmd` | int16 | BE | -2000 ~ 2000 권장 | 논리 좌측 driver 직접 입력 |
| data[2:3] | `right_driver_input_cmd` | int16 | BE | -2000 ~ 2000 권장 | 논리 우측 driver 직접 입력 |
| data[4:5] | `max_driver_input_cmd` | uint16 | BE | 0 ~ 2000 권장 | clamp 기준값, `0`이면 VCU 기본값 사용 |
| data[6:7] | `max_speed_kmh_x100` | uint16 | BE | km/h x 100 | monitor/reference 기준 속도, 예: 300 = 3.00km/h |

주의:

- `data[6] bit0=0`이면 FSM은 기존 `0x18FF0200 Drive CMD`를 사용한다.
- `data[6] bit0=1`이면 FSM은 `0x18FF0220` freshness를 확인하고, timeout 시 STOP 처리한다.

## 3-4. `0x18FF0230` Weed Actuator CMD

actuator 목표 위치 설정, 이동 요청, 정지를 담당한다.

| Byte      | Name                 | Type   | Endian | 단위/범위 | 설명                                  |
| --------- | -------------------- | ------ | ------ | ---------- | -------------------------------------------- |
| data[0]   | `command_type`       | uint8  | -      | enum       | `0=STOP`, `1=SET_TARGET`, `2=MOVE_TO_TARGET` |
| data[1]   | `stage`              | uint8  | -      | enum       | `0=UP`, `1=MID`, `2=DOWN`                    |
| data[2:3] | `target_position_mm` | uint16 | BE     | 0 ~ 200mm  | actuator 목표 위치                               |
| data[4]   | `option`             | uint8  | -      | reserved   | 0 권장                                         |
| data[5:7] | Reserved             | -      | -      | -          | 0 권장                                         |

Command type:

| 값 | Name | 의미 |
| ----- | ---- | ------- |
| `0` | `STOP` | actuator 동작 정지 또는 요청 해제 |
| `1` | `SET_TARGET` | 목표 위치 저장, 즉시 이동하지 않음 |
| `2` | `MOVE_TO_TARGET` | 목표 위치로 이동 요청 |

Stage:

| Value | Name | Position |
| ----- | ---- | -------- |
| `0` | `UP` | 0mm |
| `1` | `MID` | 90mm |
| `2` | `DOWN` | 180mm |

예시, 180mm로 이동 요청:

```text
02 02 00 B4 00 00 00 00
```

예시, 90mm 목표만 저장:

```text
01 01 00 5A 00 00 00 00
```

현재 코드 수신 경로:

```text
decode_upper_weed_actuator_cmd() -> g_latest.upper_cmd_weed -> FSM 선택 로직
```

## 3-5. `0x18FF0240` Weed Blade CMD

blade RPM 설정, RUN, STOP을 담당한다.

| Byte | Name | Type | Endian | 단위/범위 | 설명 |
| ---- | ---- | ---- | ------ | ---------- | ----------- |
| data[0] | `command_type` | uint8 | - | enum | `0=STOP`, `1=SET_RPM`, `2=RUN` |
| data[1] | `mode` | uint8 | - | enum | `0=SYNC`, 추후 확장 |
| data[2:3] | `left_blade_rpm` | uint16 | BE | 0 ~ 2000rpm | left blade 목표 rpm |
| data[4:5] | `right_blade_rpm` | uint16 | BE | 0 ~ 2000rpm | right blade 목표 rpm |
| data[6:7] | Reserved | - | - | - | 0 권장 |

Command type:

| 값 | Name | 의미 |
| ----- | ---- | ------- |
| `0` | `STOP` | blade 정지 |
| `1` | `SET_RPM` | 목표 RPM 저장, 즉시 RUN 아님 |
| `2` | `RUN` | 목표 RPM으로 blade ON |

예시, 좌/우 1500rpm RUN:

```text
02 00 05 DC 05 DC 00 00
```

예시, 좌/우 1500rpm 설정만 저장:

```text
01 00 05 DC 05 DC 00 00
```

예시, blade STOP:

```text
00 00 00 00 00 00 00 00
```

현재 코드 수신 경로:

```text
decode_upper_blade_cmd() -> g_latest.upper_cmd_blade -> FSM 선택 로직
```

## 4. Gateway -> Upper TX Detail

## 4-1. `0x18FF0300` Motor Status RPM

| Byte | Name | Type | Endian | 단위 | 설명 |
| ---- | ---- | ---- | ------ | ---- | ----------- |
| data[0:1] | `driver_left_axis1_rpm` | int16 | BE | rpm/driver feedback | left driver axis1 피드백 |
| data[2:3] | `driver_left_axis2_rpm` | int16 | BE | rpm/driver feedback | left driver axis2 피드백 |
| data[4:5] | `driver_right_axis1_rpm` | int16 | BE | rpm/driver feedback | right driver axis1 피드백 |
| data[6:7] | `driver_right_axis2_rpm` | int16 | BE | rpm/driver feedback | right driver axis2 피드백 |

## 4-2. `0x18FF0310` Gateway Status

| Byte | Name | Type | Endian | 설명 |
| ---- | ---- | ---- | ------ | ----------- |
| data[0:1] | `power_supply_value` | int16 | BE | 전원 전압/전원 상태값 |
| data[2] | `md_left_fault_msg` | uint8 | - | left motor driver fault code |
| data[3] | `md_right_fault_msg` | uint8 | - | right motor driver fault code |
| data[4] | `rc_status_mask` | uint8 | - | RC 상태 bit mask |
| data[5] | `fsm_status_mask` | uint8 | - | FSM 상태 bit mask |
| data[6] | `relay_st` | uint8 | - | relay 상태 bit mask |
| data[7] | `timeout_detail_code` | uint8 | - | timeout 상세 코드 |

RC status mask, data[4]:

| Bit  | Define                    | 의미                                       |
| ---- | ------------------------- | --------------------------------------------- |
| bit0 | `RC_ST_ENABLE`            | RC B 버튼 enable 상태                            |
| bit1 | `RC_ST_EMERGENCY_STOP`    | RC A 버튼 비상정지 상태                            |
| bit2 | `RC_ST_FAILSAFE`          | RC receiver failsafe, 조종기 신호 disconnect 상태    |
| bit3 | `RC_ST_FRESH`             | RC 데이터가 실시간으로 갱신 중인지 표시, clear이면 timeout 가능         |
| bit4 | `RC_ST_CULTIVATOR_DOWN`   | 작업기 다운, 왼쪽 토글                                 |
| bit5 | `RC_ST_CULTIVATOR_ON`     | 제초모터 ON, 오른쪽 토글                               |
| bit6 | `RC_ST_REMOTE_AUTOMATION` | RC D 버튼 remote automation 상태                 |
| bit7 | `RC_ST_DRIVE_MODE`        | RC C 버튼 주행 모드, `0=민첩형`, `1=안정형` |

FSM status mask, data[5]:

| Bit  | Define                    | 의미              |
| ---- | ------------------------- | -------------------- |
| bit0 | `FSM_ST_MODE_SAFE_STOP`   | 안전 정지 상태       |
| bit1 | `FSM_ST_MODE_MANUAL_RC`   | RC 수동 제어 상태       |
| bit2 | `FSM_ST_MODE_AUTO_ARMED`  | 자동모드 진입 준비 상태      |
| bit3 | `FSM_ST_MODE_AUTO_ACTIVE` | 상위제어기 명령으로 실제 자동주행 제어 중인 상태     |
| bit4 | `FSM_ST_STOP_UPPER_FORCE` | 상위제어기 강제 정지 발생  |
| bit5 | `FSM_ST_STOP_RC_EMG`      | RC 비상정지 발생 |
| bit6 | `FSM_ST_STOP_MOTOR_FAULT` | 모터 fault 발생       |
| bit7 | `FSM_ST_STOP_TIMEOUT`     | timeout으로 정지 발생      |

Timeout detail code, data[7]:

| 값 | Define | 의미 |
| ----- | ------ | ------- |
| `0` | `TO_NONE` | timeout 없음 |
| `1` | `TO_RC` | RC timeout |
| `2` | `TO_UPPER_CFG` | upper config timeout, 현재 hard gate로 사용하지 않음 |
| `3` | `TO_UPPER_DRIVE` | 상위제어기 주행 명령 timeout |
| `4` | `TO_MOTOR_LEFT` | left motor 피드백 timeout |
| `5` | `TO_MOTOR_RIGHT` | right motor 피드백 timeout |
| `6` | `TO_MULTIPLE` | 복수 timeout |
| `7` | `TO_UPPER_AUTO` | 선택된 `0x18FF0220` auto direct 명령 timeout |

## 4-3. `0x18FF0320` Weed Actuator Status

| Byte | Name | Type | Endian | 단위 | 설명 |
| ---- | ---- | ---- | ------ | ---- | ----------- |
| data[0] | `actuator_state` | uint8 | - | enum | VCU가 해석한 actuator 요약 상태 |
| data[1] | `error_code` | uint8 | - | code | actuator 원본 error code |
| data[2:3] | `target_position_mm` | uint16 | BE | mm | VCU가 현재 목표로 잡은 위치 |
| data[4:5] | `actual_position_mm` | uint16 | BE | mm | actuator 피드백 위치 |
| data[6] | `status_flags` | uint8 | - | bit mask | actuator 원본 status flags |
| data[7] | `meta_bits` | uint8 | - | bit mask | VCU가 해석한 상태 메타 정보 |

Actuator state:

| 값 | Name | 의미 |
| ----- | ---- | ------- |
| `0` | `UNKNOWN` | 상태 판단 불가 |
| `1` | `HOME` | 원위치, 0mm 근처 |
| `2` | `MOVING_DOWN` | 목표가 현재보다 크고 이동 중 |
| `3` | `TARGET_REACHED` | 목표 위치 도달 |
| `4` | `MOVING_UP` | 목표가 현재보다 작고 이동 중 |
| `5` | `POSITION_MISMATCH` | 목표와 실제 위치 차이 큼 |
| `6` | `STOPPED` | 정지 또는 동작 요청 없음 |
| `7` | `FAULT` | error code 존재 |
| `8` | `TIMEOUT` | actuator status timeout 발생 |

Actuator meta bits, data[7]:

| Bit | Define | 의미 |
| --- | ------ | ------- |
| bit0 | `ACT_META_VALID` | status를 한 번 이상 수신 |
| bit1 | `ACT_META_FRESH` | status가 최근 수신됨 |
| bit2 | `ACT_META_TIMEOUT` | status timeout 발생 |
| bit3 | `ACT_META_MOVING` | 이동 중 |
| bit4 | `ACT_META_TARGET_REACHED` | 목표 위치 도달 |
| bit5 | `ACT_META_COMMAND_ACTIVE` | actuator 명령 활성 상태 |
| bit6 | `ACT_META_FAULT` | fault 존재 |
| bit7 | Reserved | 0 |

## 4-4. `0x18FF0330` Weed Blade Status

| Byte      | Name                 | Type  | Endian | 단위     | 설명                         |
| --------- | -------------------- | ----- | ------ | -------- | ----------------------------------- |
| data[0]   | `blade_state`        | uint8 | -      | enum     | VCU가 해석한 blade 요약 상태 |
| data[1]   | `fault_summary`      | uint8 | -      | bit mask | 좌/우 blade fault 요약      |
| data[2:3] | `left_feedback_rpm`  | int16 | BE     | rpm      | left blade 피드백 rpm             |
| data[4:5] | `right_feedback_rpm` | int16 | BE     | rpm      | right blade 피드백 rpm            |
| data[6]   | `cmd_rpm_scaled`     | uint8 | -      | rpm / 10 | 현재 명령 rpm 축약값          |
| data[7]   | `meta_bits`          | uint8 | -      | bit mask | VCU가 해석한 상태 메타 정보            |

Blade state:

| 값 | Name | 의미 |
| ----- | ---- | ------- |
| `0` | `UNKNOWN` | 상태 판단 불가 |
| `1` | `STOPPED` | blade 정지 |
| `2` | `RUNNING` | blade RUN 명령 활성 |
| `3` | `SET_RPM_ONLY` | 목표 RPM 저장, RUN 아님 |
| `4` | `FAULT` | 좌/우 blade fault 존재 |
| `5` | `TIMEOUT` | 좌/우 blade status timeout 발생 |

Fault summary, data[1]:

| Bit | Define | 의미 |
| --- | ------ | ------- |
| bit0 | `BLADE_FAULT_LEFT` | left blade fault 존재 |
| bit1 | `BLADE_FAULT_RIGHT` | right blade fault 존재 |
| bit2 | `BLADE_FAULT_ANY` | 좌/우 중 하나라도 blade fault 존재 |
| bit3~7 | Reserved | 0 |

Blade meta bits, data[7]:

| Bit | Define | 의미 |
| --- | ------ | ------- |
| bit0 | `BLADE_META_LEFT_VALID` | left status를 한 번 이상 수신 |
| bit1 | `BLADE_META_LEFT_FRESH` | left status가 최근 수신됨 |
| bit2 | `BLADE_META_RIGHT_VALID` | right status를 한 번 이상 수신 |
| bit3 | `BLADE_META_RIGHT_FRESH` | right status가 최근 수신됨 |
| bit4 | `BLADE_META_RUNNING` | RUN 상태 |
| bit5 | `BLADE_META_COMMAND_ACTIVE` | blade 명령 활성 상태 |
| bit6 | `BLADE_META_FAULT` | fault 존재 |
| bit7 | Reserved | 0 |

예시, 좌/우 1500rpm RUN 정상:

```text
02 00 05 DC 05 DC 96 3F
```

## 5. Control Policy

## 5-1. Manual RC / Auto Handover

기본 제어권은 RC 우선이다.

Upper 자동주행이 활성화되려면 다음 조건이 함께 필요하다.

1. RC 데이터 fresh
2. RC B 버튼 enable 상태 ON
3. RC D 버튼 remote automation 상태 ON
4. Upper Config CMD의 automation ON
5. Upper Drive CMD fresh

Upper Drive CMD가 timeout되면 FSM은 STOP 상태로 전환한다.

## 5-2. Stop Priority

STOP 조건은 일반 주행 명령보다 우선한다.

우선순위:

1. Upper force stop
2. RC emergency stop
3. 좌/우 모터 fault 또는 timeout
4. auto/force upper 상황에서 Upper Drive CMD timeout
5. RC timeout 또는 유효한 제어 소스 없음

## 5-3. Weed / Blade Source Policy

- RC mode에서는 RC switch 값으로 actuator/blade 목표를 만든다.
- Auto mode에서는 안전 기본값에서 시작한 뒤, 유효한 Upper weed/blade CMD가 있을 때만 해당 값을 사용한다.
- actuator/blade status는 각각 `0x18FF0320`, `0x18FF0330`으로 200ms 주기로 보고한다.

## 6. Implementation Checklist for Upper Controller

- [ ] CAN extended frame 사용
- [ ] CAN speed 500 kbps 설정
- [ ] multi-byte payload Big-endian 구현
- [ ] `0x18FF0200` Drive CMD를 200ms 주기로 송신
- [ ] `max_speed_kmh_x100`는 `km/h x 100`으로 송신, 예: 3km/h = 300
- [ ] `0x18FF0210` Config CMD는 공통 설정만 송신
- [ ] actuator 명령은 `0x18FF0230` 사용
- [ ] blade 명령은 `0x18FF0240` 사용
- [ ] Gateway status `0x18FF0310`의 FSM/timeout bit를 모니터링
- [ ] actuator status `0x18FF0320` 모니터링
- [ ] blade status `0x18FF0330` 모니터링

## 7. Notes

- `0x18FF0220`은 `0x18FF0210 data[6] bit0=1`일 때 FSM 주행 로직에서 사용한다.
- Vehicle Motion `0x18FF4000`, Vehicle Monitor `0x18FF4010`은 test/debug 성격이다.
- actuator/blade 명령은 `0x18FF0210`에서 분리되어 있다.
- 현재 명세는 VCU Gateway 코드 기준 Draft이며, 실제 장비 테스트 결과에 따라 수정될 수 있다.
