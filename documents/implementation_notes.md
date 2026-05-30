# 구현 노트 — 논문 vs. 코드 차이

논문(IEEE TRO 2025)에 기술된 내용과 공개 코드(`simulation/stonefish` 브랜치) 사이의 차이를 정리한다.

---

## Online Sensor Calibration (Section V)

논문은 두 가지 online calibration을 핵심 기여로 제시한다:
- **Extrinsic calibration**: T_ID (IMU↔DVL), T_DC (DVL↔Camera)
- **DVL transducer misalignment calibration**: 각 빔의 α_n, β_n (n=1..4)

### Extrinsic Calibration (T_ID, T_DC) — 미구현

논문 Section V-A의 5단계 coarse-to-fine 절차:
1. Vision-only Bundle Adjustment
2. E 초기 추정 (DVL 변위 + IMU 회전 잔차)
3. 자이로 바이어스 포함 E 정제
4. 중력 방향 R_WI0 초기화
5. 전체 정제 (식 22)

**코드 현황**: 구현되지 않았다. T_ID, T_DC는 YAML에서 고정 로드되며
최적화 변수로 취급되지 않는다.

`/aqua_slam/calibrate` 서비스 → `RosHandling::CalibrateDVLGyro()`
→ `LocalMapping::InitializeDvlIMU()` 로 연결되는데, 이것은
**자이로 바이어스 초기화**이지 extrinsic calibration이 아니다.

### DVL Misalignment Calibration — Dead code

논문 Section V-B의 3단계:
1. Vision-only BA
2. DVL body velocity 최적화
3. 빔 각도 파라미터 O* = {α_n, β_n} 최적화

**코드 현황**: g2o 엣지 타입은 정의되어 있다:
- `EdgeDVLBeamCalibration1` — `include/G2oTypes.h:1853`
- `EdgeDVLBeamCalibration2` — `include/G2oTypes.h:1928`
- 사용 코드 — `src/Optimizer.cc:14018`, `src/Optimizer.cc:14113`

그러나 결과 저장 경로가 개발자 로컬 머신으로 하드코딩되어 있어
정식 실행 경로에 통합되지 않은 실험 코드임을 알 수 있다:
```cpp
// src/Optimizer.cc:12719, 13004
ofstream f("/home/da/project/ros/orb_dvl2_ws/src/dvl2/calibration_results/...");
```

### Rapid Linear Approximation (Section V-C) — 부분 구현

식 (25), (26)의 선형화는 `DVLGroPreIntegration` 내부에 부분적으로
반영되어 있으나, extrinsic calibration과 연동된 완전한 형태는 없다.

---

## 구현된 것 vs. 미구현 요약

| 논문 기술 | 코드 상태 | 관련 파일 |
|-----------|-----------|-----------|
| T_ID, T_DC extrinsic calibration | ❌ 미구현 | YAML 고정 (`data/*.yaml`) |
| DVL 빔 각도 misalignment 교정 | ❌ Dead code | `Optimizer.cc:14018`, `G2oTypes.h:1853` |
| IMU 자이로 바이어스 초기화 | ✅ 구현 | `LocalMapping::InitializeDvlIMU()` |
| 가속도계 바이어스 + 중력 방향 | ✅ 부분 구현 | `Optimizer::DvlIMUInitOptimization()` |
| Rapid linear approximation | ⚠️ 부분 | `DVLGroPreIntegration` |

---

## 배경

논문 말미에 *"source code will be released upon acceptance"* 라고 명시되어 있으며,
calibration 모듈은 정리가 덜 된 채로 공개된 것으로 추정된다.
향후 extrinsic calibration 구현 시 논문 식 (22)와 Appendix VIII-C, VIII-D를 참고할 것.
