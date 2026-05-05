# Result

- 상태: `completed`
- 마지막 갱신 ROS time: `1777971404.287`

## 결과
- DIGGING 시간: `273.247 s`
  AMR1이 DIGGING을 시작한 순간부터 AMR2가 target pallet을 lift하고 shared pre_dock에 도착할 때까지
- RELOCATE 시간: `108.050 s`
  AMR1이 MOVE_TO_RELOCATION을 시작한 순간부터 LOWERING을 완료할 때까지
- 전체 시간: `759.796 s`
  주문을 받고 움직이기 시작한 순간부터 AMR1이 초기 위치에 도착할 때까지
- AMR1 Nav2 이동 시간: `304.413 s`
  AMR1이 Nav2로 이동한 총 시간 (모든 leg 합계)
- AMR2 Nav2 이동 시간: `397.703 s`
  AMR2가 Nav2로 이동한 총 시간 (모든 leg 합계)

## 이벤트 시각
- AMR1 MOVE 시작 (주문 시작): `1777970644.490`
- AMR1 첫 shared pre_dock 도착: `1777970789.464`
- AMR1 DIGGING 시작: `1777970789.465`
- AMR2 target pallet lift 후 shared pre_dock 도착: `1777971062.712`
- AMR1 MOVE_TO_RELOCATION 시작: `1777971089.062`
- AMR1 blocker pallet 재배치 완료 (LOWERING 완료): `1777971197.112`
- AMR1 초기 위치 도착: `1777971404.287`
