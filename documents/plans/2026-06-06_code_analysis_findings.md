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
- **/orb_path는 이미 tightly-coupled** — sonar를 `LocalDVLIMUSonarBundleAdjustment()`에 추가하는 것이 가장 완성도 높고 즉시 적용 가능한 경로 (통합 계획 1단계)
- per-frame에 sonar를 추가하려면 `TrackLocalMapWithDvlGyro()`의 미완성 항목(바이어스, 속도, Jacobian)을 먼저 완성해야 함 — 선결 과제가 많아 후순위
- `/orb_pose`가 visual-only라는 사실은 SAR 미션에서 큰 문제가 아님: keyframe BA 결과인 `/orb_path`가 핵심 위치 추정 출력이기 때문

---

## 2. EdgeDvlIMU Jacobian 전부 주석 — 수치 미분 사용 중

**위치**: `include/G2oTypes.h`, `EdgeDvlIMU::linearizeOplus()`  
**상태**: `linearizeOplus` 전체 주석 → g2o 기본 수치 미분으로 fallback

수치 미분은 해석 미분보다 느리고 정밀도도 낮다. 특히 sonar residual처럼 새 edge를 추가할 때 같은 방식으로 구현하면 성능 저하가 누적된다.

**통합 시 의미**:
- `EdgeSonar` 구현 시 **해석 Jacobian을 직접 유도해서 작성**하는 것이 권장됨
- 기존 `EdgeDvlIMU`의 수치 미분도 장기적으로는 해석 Jacobian으로 교체 검토 필요

---

## 3. 실제 LocalDVLIMUBundleAdjustment에서 쓰이는 edge는 EdgeDvlIMU가 아님

**위치**: `src/DvlGyroOptimizer.cpp`, `LocalDVLIMUBundleAdjustment()`  
**실제 사용 edge**: `EdgeSE3DVLBA` + 시각 재투영 edge

`EdgeDvlIMU`는 초기화(`DvlIMUInitOptimization`) 등에서 사용되고, Local BA에서는 `EdgeSE3DVLBA`가 DVL 구속 조건을 담당한다.

**통합 시 의미**:
- `EdgeSonar`를 Local BA에 추가할 때 참고해야 할 구조는 `EdgeDvlIMU`가 아닌 **`EdgeSE3DVLBA`**
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

## 9. sonar edge 네이밍 및 설계 방향

기존 코드 컨벤션에 맞춰 sonar BA edge는 **`EdgeSonar`** 로 명명한다.

카메라 edge가 BA / Pose-only / DVL-gyro variant 세 축으로 분화된 것과 달리, sonar는 현재 keyframe BA(`LocalDVLIMUSonarBundleAdjustment`)에만 추가하므로 `EdgeSonar` 하나로 충분하다. per-frame tracking(`TrackLocalMapWithDvlGyro`)이 활성화될 때 `EdgeSonarOnlyPose`가 추가로 필요해진다.

소나 점은 fixed anchor로 처리 (map point처럼 vertex로 올리지 않음) — VISO Eq.10 스타일.

---

## 10. ROS1(upstream) vs ROS2 포팅 검증 결과

**배경**: EdgeSonar 작업을 이어가기 전에, 포팅된 코드(`migration/ros2` 브랜치, 현재 `simulation/stonefish`의 기반)가 원본 ROS1 코드(`upstream/main`, SenseRoboticsLab 공식 저장소, `main` 브랜치와 byte-identical)와 알고리즘적으로 동일하게 동작하는지 검증. `upstream` remote를 추가/fetch하여 핵심 파일 diff를 비교.

**결론**: 포팅은 거의 전부 기계적 변환(ROS1→ROS2 로깅/메시지, OpenCV3→4 enum, `mT_gyro_dvl`→`mT_imu_dvl` 등 필드 리네이밍)이며, 핵심 SLAM 알고리즘(`Optimizer.cc`, `g2o_BA.cpp`)은 로직 변경 없음. 발견된 예외 3건:

1. **`PnPsolver.cc::find_betas_approx_1` 부호 버그 — 영향 없음**
   upstream은 `b4[0] < 0`일 때 `betas[1..3]`에도 부호 반전을 적용하는데, 포팅본은 `betas[0] = sqrt(fabs(b4[0]))`만 남기고 나머지 반전이 누락됨 (`find_betas_approx_2/3`는 정상 보존). `PnPsolver` 클래스는 upstream·포팅본 모두 인스턴스화되지 않는 dead code (relocalization은 `MLPnPsolver` 사용) → 현재 영향 없음. 이 클래스를 나중에 재사용할 경우에만 한 줄 수정 필요.

2. **`Tracking.cc::StereoInitialization()` — mpLastKeyFrame NULL assert 제거: 검증됨, 정당한 완화**
   upstream: `else if (pKFini->mnId != 0) { assert(mpLastKeyFrame); }` → 포팅본: 주석으로 대체.
   `KeyFrame::nNextId`(`src/KeyFrame.cc:31`)는 프로세스 전역 static 카운터. tracking LOST 시(`Tracking.cc:2305-2325`) `CreateMapInAtlas()`로 새 서브맵을 만들며 `mpLastKeyFrame`을 명시적으로 NULL 처리하는데(`Tracking.cc:2318-2319`), 이때 새로 생성되는 KF의 `mnId`는 전역 카운터를 이어받으므로 0이 아님. 즉 "mnId≠0인데 mpLastKeyFrame==NULL"은 **멀티맵 재초기화의 정상 경로**이며, upstream의 assert는 이 정상 경로에서 (디버그 빌드라면) 매번 실패했을 것으로 보임. 포팅본의 제거는 버그가 아니라 원본의 과도한 체크를 고친 것으로 판단.

3. **`Tracking.cc::CreateNewKeyFrame()` — mpLastKeyFrame NULL assert 제거: 미해결**
   동일 패턴의 assert 제거이나, 이 함수는 `NeedNewKeyFrame()`을 통해 트래킹이 이미 `[OK]` 상태(= `StereoInitialization()`이 `mpLastKeyFrame`을 이미 세팅한 이후)에만 호출됨. 2번과 달리 이 조건이 실제로 발생하는 정상 경로를 아직 확인하지 못함 (loss-integration 분기까지는 추적 안 함). sonar 통합과 직접 관련 없어 보류하되, 추후 KF 연결 관련 문제가 관찰되면 재확인 대상.

참고로 `Tracking.cc`에 추가된 `T_body_imu` 파라미터, `LocalMapping.cc::GetTravelDistance()`의 SVD 재직교화는 검토 결과 의도적이고 정당한 개선으로 확인됨 (각각 [coordinate_frames.md](../coordinate_frames.md), float→double 변환 시 `Sophus::SO3` assert 방지).

**통합 작업 관점에서 결론**: 위 3건 모두 sonar 통합이 딛고 설 기반 함수(`Optimizer.cc`, `g2o_BA.cpp`, DVL/IMU 관련 로직)에는 영향 없음 → **알고리즘 기반은 원본과 동일하다고 보고 다음 단계(EdgeSonar 구현) 진행 가능**.

---

## 요약 — 통합 작업 전 확인 사항

| 항목 | 파일 | 조치 필요 여부 |
|---|---|---|
| `TrackLocalMapWithDvlGyro()` 활성화 | `Tracking.cc` | 선택 (1단계 이후) |
| `EdgeSonar` 정의 및 Jacobian 유도 | `G2oTypes.h` (신규) | **필수** |
| `LocalDVLIMUSonarBundleAdjustment` sonar edge 삽입 | `DvlGyroOptimizer.cpp` | **필수** |
| `KeyFrameCulling()` 재활성화 검토 | `LocalMapping.cc` | 권장 |
| sonar 입력을 별도 큐로 분리 | `node.cpp` | **권장** |
| `mPoorVision` 플래그 기반 adaptive λ_s | `Tracking.cc` / `DvlGyroOptimizer.cpp` | 선택 (RUSSO 아이디어) |
| ROS1 upstream 포팅 검증 (항목 10) | `Tracking.cc`, `PnPsolver.cc` | 완료 — 알고리즘 기반 동일 확인, `CreateNewKeyFrame()` 케이스만 미해결 보류 |
