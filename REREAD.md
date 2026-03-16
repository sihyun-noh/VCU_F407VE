# REREAD

이 문서는 현재 VCU Gateway 동작을 빠르게 다시 읽기 위한 요약본입니다.

## 핵심 구조
- 입력:
  - SBUS (`CH3 throttle`, `CH1 steering`)
  - Upper CMD CAN (`0x18FF0200` 속도, `0x18FF0210` 설정)
  - Motor Driver 상태 CAN (left/right status)
- 내부:
  - `sbus_thread` -> RC intent 업데이트
  - `fsm_thread` -> 우선순위 판단 + 모터 명령/상태 생성
  - `canrx`/`cantx` 분리 스레드 -> 수신 지연 최소화
- 출력:
  - Driver1/2 명령 송신
  - Upper 상태 송신 (`0x18FF0310`)
  - Upper 모터 피드백 송신 (`0x18FF0300`)

## 차동구동(현재 규칙)
- 기본식:
  - `left = throttle + steering`
  - `right = throttle - steering`
- 보호:
  - 주행 중(`throttle != 0`)에는 `|steering| <= |throttle|`로 제한
  - 제자리 회전(`throttle == 0`) 허용

## Upper 상태 프레임 (`0x18FF0310`)
- `data[0:1]`: power supply
- `data[2]`: left fault
- `data[3]`: right fault
- `data[4]`: RC status mask
- `data[5]`: VCU FSM status mask
- `data[6]`: relay status
- `data[7]`: reserved

## 참고 문서
- `DIFFERENTIAL_DRIVE.md`
- `UPPER_VCU_STATUS_GUIDE.md`
- `VCU_GATEWAY_UPDATE_SUMMARY.md`
