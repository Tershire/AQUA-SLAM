# Code Analysis Findings — Integration-Relevant Notes

**작성일**: 2026-06-06  
**배경**: 핵심 소스 파일 분석 중 발견한 사항 중 VISO 통합 및 향후 작업에 영향을 주는 것들을 정리.  
**참고 문서**: [VISO 통합 계획](2026-05-30_viso_integration_plan.md), [code_analysis/](../code_analysis/)

---

## 1. 두 레벨 tightly-coupled 구조 — 현재 구현 gap

**이상적 구조**: sensor fusion은 per-frame tracking(20Hz)과 keyframe BA(~4Hz) 두 레벨에서 모두 tightly-coupled되어야 한다.

```
센서 입력 (20Hz)
    ↓
[Per-frame tracking]  → /orb_pose, /orb_odom (20Hz)
  E = λv·E_visual + λd·E_DVL + λr·E_rot
    ↓ (키프레임 선별 ~4Hz)
[Keyframe BA]          → /orb_path (~4Hz)
  슬라이딩 윈도우, 바이어스·속도 최적화
```

**현재 AQUA-SLAM 구현 상태**:

| 레벨 | 이상 | 현재 | 출력 토픽 |
|---|---|---|---|
| Per-frame (~20Hz) | camera + DVL + gyro tightly-coupled | **시각 전용** `PoseOptimization()` | /orb_pose, /orb_odom |
| Keyframe BA (~4Hz) | camera + DVL + IMU tightly-coupled | **구현됨** `LocalDVLIMUBundleAdjustment()` | /orb_path |

**per-frame 레벨 gap 원인**:  
`TrackLocalMapWithDvlGyro()`가 작성되어 있으나 두 곳 모두 주석 처리됨 (`src/Tracking.cc` L.2410, L.5111).  
내부의 `PoseDvlGyrosOPtimizationLastFrame/LastKeyFrame()`은 카메라·DVL·gyro 잔차를 하나의 그래프에서 최적화하는 구조이나 미완성:
- 자이로 바이어스 고정 (추정 안 함)
- 속도 상태 없음 (`//todo_tightly: maybe add velocity`)
- IMU prior edge 주석 처리
- Jacobian 주석 → 수치 미분 fallback

개발자 코멘트: `"use them make visual tracking easy to lose"` — DVL 노이즈가 tracking을 불안정하게 만들어 비활성화. 미완성 구현 문제도 있음.

**통합 시 의미**:
- **/orb_path는 이미 tightly-coupled** — sonar를 `LocalDVLIMUBundleAdjustment()`에 추가하는 것이 가장 완성도 높고 즉시 적용 가능한 경로 (통합 계획 1단계)
- per-frame에 sonar를 추가하려면 `TrackLocalMapWithDvlGyro()`의 미완성 항목(바이어스, 속도, Jacobian)을 먼저 완성해야 함 — 선결 과제가 많아 후순위
- `/orb_pose`가 visual-only라는 사실은 SAR 미션에서 큰 문제가 아님: keyframe BA 결과인 `/orb_path`가 핵심 위치 추정 출력이기 때문

---

## 2. EdgeDvlIMU Jacobian 전부 주석 — 수치 미분 사용 중

**위치**: `include/G2oTypes.h`, `EdgeDvlIMU::linearizeOplus()`  
**상태**: `linearizeOplus` 전체 주석 → g2o 기본 수치 미분으로 fallback

수치 미분은 해석 미분보다 느리고 정밀도도 낮다. 특히 sonar residual처럼 새 edge를 추가할 때 같은 방식으로 구현하면 성능 저하가 누적된다.

**통합 시 의미**:
- `EdgeSonarPoint` 구현 시 **해석 Jacobian을 직접 유도해서 작성**하는 것이 권장됨
- 기존 `EdgeDvlIMU`의 수치 미분도 장기적으로는 해석 Jacobian으로 교체 검토 필요

---

## 3. 실제 LocalDVLIMUBundleAdjustment에서 쓰이는 edge는 EdgeDvlIMU가 아님

**위치**: `src/DvlGyroOptimizer.cpp`, `LocalDVLIMUBundleAdjustment()`  
**실제 사용 edge**: `EdgeSE3DVLBA` + 시각 재투영 edge

`EdgeDvlIMU`는 초기화(`DvlIMUInitOptimization`) 등에서 사용되고, Local BA에서는 `EdgeSE3DVLBA`가 DVL 구속 조건을 담당한다.

**통합 시 의미**:
- `EdgeSonarPoint`를 Local BA에 추가할 때 참고해야 할 구조는 `EdgeDvlIMU`가 아닌 **`EdgeSE3DVLBA`**
- `include/G2oTypes.h`에서 `EdgeSE3DVLBA` 구조를 먼저 파악할 것

---

## 4. LoopClosing의 Global BA 주석 처리

**위치**: `src/LoopClosing.cc`, `CorrectLoop()` 내부  
**상태**: GBA(Global Bundle Adjustment) 호출 비활성

Loop closure 검출 후 포즈 그래프 최적화(`OptimizeEssentialGraph`)는 수행되지만, 전역 BA는 꺼져 있다. 루프 클로저의 정확도가 제한될 수 있다.

**통합 시 의미**:
- 현재도 루프 클로저 후 지도가 완전히 정렬되지 않을 수 있음
- sonar 통합 후 루프 클로저 정확도 검증 시 이 점을 감안해야 함

---

## 5. KeyFrameCulling() 주석 처리 — KF 무한 누적

**위치**: `src/LocalMapping.cc` L.246–248  
**상태**: `if(mpTracker->mCalibrated)` 블록 전체 주석

KF가 제거되지 않아 장시간 운용 시 메모리와 BA 계산량이 계속 늘어난다.

**통합 시 의미**:
- sonar 데이터도 KF마다 저장된다면 메모리 증가 폭이 더 커짐
- 장시간 SAR 미션에서 실질적 문제가 될 수 있으므로 culling 재활성화 검토 필요

---

## 6. mPoorVision 플래그 — RUSSO adaptive weight 구현 위치

**위치**: `src/Tracking.cc` L.5090  
**의미**: 시각 feature가 부족할 때 세워지는 플래그

RUSSO의 adaptive weight 아이디어(시각 열화 시 sonar 가중치 λ_s 증가)를 구현하려면 이 플래그를 확인하거나, 직접 tracked feature 수를 체크하면 된다.

```cpp
// 구현 예시 (DvlGyroOptimizer.cpp 내 LocalDVLIMUBundleAdjustment 진입부)
float lambda_s = (n_tracked_features < VISUAL_DEGRADE_THRESH)
                 ? lambda_s_high : lambda_s_normal;
```

---

## 7. EdgeDVLBeamCalibration1/2 — Dead Code 위치

**위치**: `include/G2oTypes.h` L.1853, L.1928 / `src/Optimizer.cc` L.14018, L.14113  
**상태**: 호출 체인이 끊겨 있음. 개발자 로컬 경로 하드코딩 포함

논문 Section V-B의 DVL misalignment calibration(αₙ, βₙ)에 해당하는 코드이나 미완성 상태.  
→ 별도 문서: [implementation_notes.md](../implementation_notes.md)

---

## 8. DVL과 IMU를 같은 벡터에 혼합 전달

**위치**: `src/node.cpp`, `SyncWithImu()`  
**구조**: `vGyroDVLMeas` 벡터에 IMU와 DVL 측정값을 함께 넣고 `angular_v != 0` 여부로 구분

sonar 입력을 추가할 때 동일한 벡터에 혼합하는 방식은 지양하고, 별도 인자 또는 별도 큐로 분리하는 것이 권장됨.

---

## 요약 — 통합 작업 전 확인 사항

| 항목 | 파일 | 조치 필요 여부 |
|---|---|---|
| `TrackLocalMapWithDvlGyro()` 활성화 | `Tracking.cc` | 선택 (1단계 이후) |
| `EdgeSE3DVLBA` 구조 파악 | `G2oTypes.h` | **필수** (EdgeSonarPoint 설계 전) |
| `EdgeSonarPoint` Jacobian 해석 유도 | 신규 | **필수** |
| `KeyFrameCulling()` 재활성화 검토 | `LocalMapping.cc` | 권장 |
| sonar 입력을 별도 큐로 분리 | `node.cpp` | **권장** |
| `mPoorVision` 플래그 기반 adaptive λ_s | `Tracking.cc` / `DvlGyroOptimizer.cpp` | 선택 (RUSSO 아이디어) |
