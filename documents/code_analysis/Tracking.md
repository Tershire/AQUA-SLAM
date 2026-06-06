# Tracking.cc 분석

**파일 위치**: `src/Tracking.cc`  
**전체 라인**: 6911줄  
**역할**: 프레임 단위 카메라 자세 추적 스레드. 외부에서 stereo 이미지와 DVL/자이로 데이터를 받아 매 프레임마다 자세를 추정하고, 키프레임이 필요하면 LocalMapping에 전달한다.

---

## 진입점 구조 (외부 → Tracking)

RosHandling(또는 System)이 새 이미지 프레임 도착 시 아래 중 하나를 호출한다.

| 함수 | 용도 |
|---|---|
| `GrabImageStereoDvl(...)` | DVL 플래그 포함 스테레오 입력 → `Track()` 호출 |
| `GrabImageStereoDvlgyro(...)` | DVL+자이로 입력 → `TrackDVLGyro()` 호출 (현재 주 경로) |
| `GrabImageStereoDvlKLT(...)` | KLT 트래커 실험용 경로 → `TrackKLT()` 호출 |
| `GrabImuData(point)` | IMU 측정값 큐(`mlQueueImuData`)에 삽입 |
| `GrabDVLGyroData(point)` | DVL+자이로 측정값 큐(`mlQueueDVLGyroData`)에 삽입 |

---

## 메인 루프

### `Track()` — 범용 루프 (L.2048)
원래 ORB-SLAM3의 IMU_STEREO 경로용. DVL_STEREO 모드에서도 동작하지만 현재 코드에서는 `GrabImageStereoDvl()` 경로에서만 호출된다.

```
PreintegrateDvlGro()
  → StereoInitialization()  [NOT_INITIALIZED 시]
  → CheckReplacedInLastFrame()
  → TrackReferenceKeyFrame() 또는 TrackWithMotionModel()
  → TrackLocalMap()  [TrackLocalMapWithDvlGyro()는 주석 처리됨]
  → NeedNewKeyFrame() → CreateNewKeyFrame()
  → PublishOrb()
```

### `TrackDVLGyro()` — AQUA-SLAM 주 경로 (L.2682)
`GrabImageStereoDvlgyro()`에서 호출. DVL+자이로 통합 경로.

```
PreintegrateDvlGro3()    ← Integrator 사용 (구 PreintegrateDvlGro와 다름)
  → StereoInitialization()
  → CheckReplacedInLastFrame()
  → TrackWithMotionModel() [실패 시 TrackReferenceKeyFrame(), 그래도 실패 시 PredictStateDvlGro()]
  → TrackLocalMap()        [mCalibrated 여부 무관하게 호출; TrackLocalMapWithDvlGyro()는 주석 처리됨]
  → NeedNewKeyFrame() → CreateNewKeyFrame()
  → PublishOrb()           [mInitialized == true 일 때만 발행]
```

### `TrackDVLImu()` (L.3073)
함수 몸체가 비어 있음. 미구현 상태.

### `TrackKLT()` (L.3078)
LKTracker를 사용하는 실험적 경로. 현재 production에서 사용되지 않음.

---

## 함수 목록

### 초기화

| 함수 | 역할 |
|---|---|
| `Tracking(...)` | 생성자. 카메라·ORB·IMU 파라미터 파싱, `mT_c_cm`/`mT_e_c`/`mT_g_e` 고정 외부 변환 설정, Integrator 생성 |
| `ParseCamParamFile(fSettings)` | YAML에서 카메라 내부 파라미터 파싱 |
| `ParseORBParamFile(fSettings)` | ORB 추출기 파라미터 파싱 |
| `ParseIMUParamFile(fSettings)` | IMU/DVL 외부 변환(`mT_dvl_c`, `mT_imu_dvl`) 파싱 |
| `StereoInitialization()` | N>500 특징점 조건 충족 시 초기 KF 생성, MapPoint 삼각측량 후 Atlas·LocalMapper에 등록. loss integration 중이면 `PoseOnlyOptimizationDVLIMU()` 실행 후 초기화 |
| `StereoInitializationKLT()` | KLT 경로용 초기화. 현재 미사용 |
| `MonocularInitialization()` / `CreateInitialMapMonocular()` | 단안 경로용 초기화 (DVL 모드에서 미사용) |
| `CreateMapInAtlas()` | 추적 완전 실패 시 새 서브맵 생성. DVL 모드에서는 loss integration 시작 (`mpIntegrator->SetDoLossIntegration(true)`) |

### 프리인테그레이션

| 함수 | 역할 |
|---|---|
| `PreintegrateIMU()` | IMU_STEREO/IMU_MONO 경로용 IMU 프리인테그레이션 |
| `PreintegrateDvlGro()` | DVL+자이로 프리인테그레이션 (구 경로, `Track()` 내에서 사용). 큐에서 측정값을 꺼내 KF·Frame 수준의 `DVLGroPreIntegration` 객체에 적분 |
| `PreintegrateDvlGro2()` | 위의 실험적 변형. 현재 호출 없음 |
| `PreintegrateDvlGro3()` | 신규 경로. `mpIntegrator->IntegrateMeasurements()`에 위임. `TrackDVLGyro()` 내에서 사용 |

### 자세 예측

| 함수 | 역할 |
|---|---|
| `PredictStateIMU()` | IMU 프리인테그레이션으로 현재 프레임 자세 예측. `mbMapUpdated` 여부에 따라 KF 기준 또는 LastFrame 기준으로 분기 |
| `PredictStateDvlGro()` | DVL+자이로 프리인테그레이션으로 현재 프레임 자세 예측. loss integration 중이면 `mvpLossKF` 기준으로, 아니면 `mpLastKeyFrame` 기준으로 DVL 위치 적분 후 카메라 자세로 변환 |

### 트래킹 (프레임 수준)

| 함수 | 역할 |
|---|---|
| `TrackReferenceKeyFrame()` | BoW로 reference KF와 매칭 후 `PoseOptimization()` 실행. 15개 미만 매칭이면 실패 |
| `TrackWithMotionModel()` | 속도 모델로 자세 초기 예측 후 projection matching + `PoseOptimization()`. 매칭 20개 미만이면 window 확장 재시도 |
| `TrackWithMotionModelAndEKF()` | `PoseOptimizationWithBA_and_EKF()` 사용 버전. 현재 호출되지 않음 |
| `TrackWithVisualAndEKF()` | `PoseOptimizationWithBA_and_EKF()` 사용 버전. 현재 호출되지 않음 |
| `UpdateLastFrame()` | 지난 프레임 자세를 reference KF 기준으로 갱신. Localization 모드에서는 임시 MP도 생성 |

### 트래킹 (로컬 맵 수준)

| 함수 | 역할 |
|---|---|
| `TrackLocalMap()` | 로컬 맵 업데이트 → `SearchLocalPoints()` → `PoseOptimization()`. inlier < 10이면 실패. inlier < ORB개수×10%이면 `mPoorVision = true` 세팅 |
| `TrackLocalMapWithDvlGyro()` | 위 함수의 DVL+자이로 버전. `PoseDvlGyrosOPtimizationLastFrame/LastKeyFrame()` 사용. inlier < 30이면 실패, inlier < ORB개수×15%이면 `mPoorVision = true`. **현재 Track()/TrackDVLGyro() 내에서 모두 주석 처리됨** |
| `SearchLocalPoints()` | 로컬 KF의 MP를 현재 프레임에 projection해서 추가 매칭 |
| `UpdateLocalMap()` | `UpdateLocalKeyFrames()` + `UpdateLocalPoints()` 호출 |
| `UpdateLocalKeyFrames()` | 현재 프레임과 covisibility 있는 KF들을 `mvpLocalKeyFrames`에 수집 |
| `UpdateLocalPoints()` | `mvpLocalKeyFrames` 내 모든 MP를 `mvpLocalMapPoints`에 수집 |

### 키프레임 관리

| 함수 | 역할 |
|---|---|
| `NeedNewKeyFrame()` | KF 삽입 여부 결정. 미보정 상태(`!mCalibrated`)에서는 `mKF_init_step` 시간 간격 + DVL 측정값 존재만 확인. 보정 완료 후에는 ORB-SLAM3 기준(c1a/c1b/c1c/c2) + 시간 간격(c3) 조합 |
| `CreateNewKeyFrame()` | 현재 프레임으로 KF 생성. `mpDenseMapper`에 등록, preintegration 객체 리셋(`mpIntegrator->CreateNewIntFromKF_C2C()`), 깊이 기준 새 MP 생성, `LocalMapper->InsertKeyFrame()` 호출 |
| `CreateNewKeyFrameKLT()` | KLT 경로용 KF 생성. 현재 미사용 |
| `CreateNewMapPoints()` | 모든 KF와 epipolar 삼각측량으로 새 MP 생성 (LocalMapping의 것과 별도). Tracking 스레드 내부에서 직접 호출됨 |

### 재지역화 / 리셋

| 함수 | 역할 |
|---|---|
| `Relocalization()` | BoW 후보 KF 검색 → ORBmatcher BoW 매칭 → MLPnPsolver RANSAC → `PoseOptimization()` 반복. 50개 inlier 달성 시 성공 |
| `CheckReplacedInLastFrame()` | LocalMapping이 MP를 교체한 경우 LastFrame의 포인터 갱신 |
| `Reset(bLocMap)` | 전체 리셋. LocalMapper·LoopClosing·Atlas 모두 리셋 |
| `ResetActiveMap(bLocMap)` | 현재 활성 맵만 리셋 |

### DVL/자이로 바이어스 갱신

| 함수 | 역할 |
|---|---|
| `UpdateFrameDVLGyro(b, pCurrentKeyFrame)` | LocalMapping의 BA 결과로 바이어스가 갱신되면 호출됨. `mLastBias` 저장 후, LastFrame/CurrentFrame이 동일 KF 기준이면 DVL 프리인테그레이션으로 두 프레임 자세도 재계산 |
| `UpdateFrameIMU(s, b, pCurrentKeyFrame)` | IMU 경로용 바이어스·스케일 갱신 |
| `ComputeGyroBias(vpFs, bwx, bwy, bwz)` | 프레임 집합에서 자이로 바이어스 추정 |
| `ComputeVelocitiesAccBias(vpFs, bax, bay, baz)` | 프레임 집합에서 가속도 바이어스 추정 |
| `ResetFrameIMU()` | 구현 없음 (빈 함수) |
| `IntegrateDVLVelocity()` | DVL 속도 적분 유틸리티 (단독 호출 미확인) |

### 발행 / 시각화

| 함수 | 역할 |
|---|---|
| `topicPublishDVLOnly()` | 초기화 미완 또는 NOT_INITIALIZED 상태에서 현재 자세를 `PublishOrb()`로 발행. `mInitialized == false`이면 조기 반환 |
| `drawOptimizationResult()` | 비교 시각화 디버그 함수. 다중 최적화 방법 결과를 나란히 표시 |
| `saveMatchingResults(...)` | 속도 매칭 결과를 CSV로 저장하는 디버그 함수 |
| `SaveOptimizationResult()` | 최적화 결과를 파일로 저장하는 디버그 함수 |

### 유틸리티

| 함수 | 역할 |
|---|---|
| `ComputeF12(pKF1, pKF2)` | 두 KF 간 Fundamental matrix 계산 |
| `GetExtrinsicPara()` / `SetExtrinsicPara(calib)` | `mpImuCalib` 뮤텍스 보호 읽기/쓰기 |
| `SetBeamOrientation(alpha, beta)` / `GetBeamOrientation(...)` | DVL 빔 방향 파라미터(`mAlpha`, `mBeta`) 뮤텍스 보호 읽기/쓰기 |
| `LoadEKFReading(...)` | EKF 자세/속도 외부 입력 저장 (현재 EKF 경로는 미사용) |
| `getMvpLossKf()` / `clearPartLossKF()` | loss KF 집합(`mvpLossKF`) 뮤텍스 보호 접근자 |
| `setOptV()` / `getOptV()` | 최적화 DVL 속도 뮤텍스 보호 읽기/쓰기 |
| `getLossIntegrationRef()` | loss integration 레퍼런스 `DVLGroPreIntegration` 포인터 반환 |
| `SetLocalMapper()` / `SetLoopClosing()` / `SetStepByStep()` | 의존 모듈 포인터 주입 |
| `InformOnlyTracking(flag)` | `mbOnlyTracking` 플래그 설정 |
| `ChangeCalibration(strSettingPath)` | 런타임 카메라 파라미터 교체 |
| `GetLocalMapMPS()` / `GetMatchesInliers()` / `GetNumberDataset()` / `NewDataset()` | 상태 조회 |

---

## 핵심 함수 상세

### `TrackDVLGyro()` (L.2682) — 주 추적 루프

전체 흐름 및 분기:

```
[매 프레임]
PreintegrateDvlGro3()        ← Integrator 위임
  ↓
[NOT_INITIALIZED]
  StereoInitialization()
    if DoLossIntegration:
      PoseOnlyOptimizationDVLIMU(mvpLossKF)  ← 새 맵 진입 시 loss KF 최적화
  topicPublishDVLOnly()
  ↓
[OK]
  CheckReplacedInLastFrame()
  TrackWithMotionModel()
    실패 → TrackReferenceKeyFrame()
      실패 → PredictStateDvlGro()     ← KF 충분하면 강제 bOK=true
  TrackLocalMap()
    실패 → PredictStateDvlGro()
  ↓
[bOK]
  NeedNewKeyFrame() → CreateNewKeyFrame()
  PublishOrb()    ← mInitialized && !mTcw.empty() 조건
  mVelocity 갱신 (T_cj_ci)
  ↓
[LOST]
  KF < mKFThresholdForMap → ResetActiveMap()
  그 외 → CreateMapInAtlas()  ← loss integration 시작
```

### `PoseOptimization` 경로 요약

| 호출 위치 | 사용 최적화 함수 | 비고 |
|---|---|---|
| `TrackReferenceKeyFrame()` | `Optimizer::PoseOptimization()` | 순수 시각 g2o 최적화 |
| `TrackWithMotionModel()` | `Optimizer::PoseOptimization()` | 순수 시각 |
| `TrackLocalMap()` | `Optimizer::PoseOptimization()` | 순수 시각 |
| `TrackLocalMapWithDvlGyro()` | `PoseDvlGyrosOPtimizationLastFrame/KeyFrame()` | DVL+자이로 포함, **주석 처리됨** |
| `Relocalization()` | `Optimizer::PoseOptimization()` | 순수 시각, 최대 3회 반복 |
| `StereoInitialization()` | `PoseOnlyOptimizationDVLIMU()` | loss KF 배치 최적화 |

> **중요**: TrackLocalMap()에서는 DVL을 사용하지 않는다. DVL+자이로 포함 트래킹 최적화(`TrackLocalMapWithDvlGyro()`)는 구현되어 있지만 Track()과 TrackDVLGyro() 모두에서 주석 처리되어 있다. (L.2410, L.2838 주석 참조)

### `CreateNewKeyFrame()` (L.5335)

1. `mpLocalMapper->IsInitializing()` → 초기화 중이면 즉시 반환
2. `mpLocalMapper->SetNotStop(true)` 획득
3. Frame → KeyFrame 변환, `pKF->mPrevKF = mpLastKeyFrame` 연결
4. DVL_STEREO: `mpIntegrator->CreateNewIntFromKF_C2C(mLastBias, ...)` → 새 KF 기준 preintegration 시작
5. 깊이 기준 정렬 후 신규 MP 생성 (최대 100개, `mThDepth` 이내)
6. `mpDenseMapper->InsertNewKF(pKF)` 등록
7. `mpLocalMapper->InsertKeyFrame(pKF)` → LocalMapping 큐에 삽입

> **주의**: bias 최적화 블록(`PoseOptimizationDVLIMUBiasOnly`)이 주석 처리되어 있어 KF 생성 시 bias 갱신이 없다. `mLastBias`는 `UpdateFrameDVLGyro()` 경로로만 업데이트된다.

### `Relocalization()` (L.5981)

순수 시각 BoW 기반. DVL 정보 미사용.

```
BoW 쿼리 → 후보 KF 목록
  ↓
후보별 ORBmatcher.SearchByBoW() (< 15 매칭 시 제외)
  ↓
MLPnPsolver.iterate(5) RANSAC
  ↓
PoseOptimization() → inlier >= 10 유지
  ↓
inlier < 50: SearchByProjection(window=10) 후 재최적화
  inlier < 50: SearchByProjection(window=3) 후 최종 최적화
  ↓
inlier >= 50 → 성공
```

> **참고**: RECENTLY_LOST 상태에서 DVL 보정(`mCalibrated`)이 완료된 경우 `Relocalization()` 대신 `PredictStateDvlGro()`가 먼저 시도된다 (L.2284). 재지역화는 미보정 상태의 fallback이다.

### `UpdateFrameDVLGyro()` (L.6436)

LocalMapping 스레드의 BA가 bias를 갱신하면 호출된다. 하지만 현재 함수 내부 대부분이 주석 처리되어 있고, 실제로 수행하는 동작은:
1. `mLastBias = b` 저장 (뮤텍스 보호)
2. mLastFrame/mCurrentFrame이 동일 KF 기준이면 DVL 프리인테그레이션으로 자세 재계산

---

## 스레드 간 관계

```
RosHandling ──GrabImageStereoDvlgyro()──► Tracking (이 파일)
RosHandling ──GrabDVLGyroData()────────► Tracking.mlQueueDVLGyroData

Tracking
  │  preintegration
  ├──► Integrator (PreintegrateDvlGro3)
  │
  │  pose optimization
  ├──► Optimizer::PoseOptimization()
  ├──► Optimizer::PoseOnlyOptimizationDVLIMU()
  │
  │  KF 전달
  ├──InsertKeyFrame()──► LocalMapping
  │
  │  bias 갱신 수신
  ◄──UpdateFrameDVLGyro()── LocalMapping
  │
  │  지도 업데이트 감시
  ├──► Atlas (R/W, map lock)
  │
  │  발행
  └──PublishOrb()──► RosHandling
```

- **RosHandling → Tracking**: 이미지/센서 데이터 주입. Tracking은 호출 스레드에서 동기적으로 실행됨
- **Tracking → LocalMapping**: `InsertKeyFrame()`으로 KF 전달, `InterruptBA()`로 BA 중단 요청
- **LocalMapping → Tracking**: `UpdateFrameDVLGyro()`로 bias 역전달
- **Tracking → Atlas**: 맵 잠금 하에 KF/MP 생성 및 조회
- **Tracking → RosHandling**: `PublishOrb()`, `PublishImgWithInfo()`로 결과 발행

---

## VISO 통합 관점에서 주목할 부분

### sonar grabber 추가 위치
`GrabDVLGyroData()`와 같은 패턴으로 `GrabSonarData()` 메서드를 추가하면 된다.
- 선언: `include/Tracking.h` L.101 근처
- 구현: `src/Tracking.cc` L.1102 근처 (큐 삽입 패턴 동일)
- 큐: `mlQueueDVLGyroData`에 해당하는 `mlQueueSonarData` 신설

### 시각 열화 감지 위치
`mCurrentFrame.mPoorVision` 플래그가 세팅되는 곳:
- **L.5090** (`TrackLocalMap()`): inlier < ORB특징점수 × 10%
- **L.5185** (`TrackLocalMapWithDvlGyro()`): inlier < ORB특징점수 × 15% (현재 미사용)

이 플래그를 이용해 소나 가중치를 높이거나 소나 전용 자세 예측으로 전환하는 분기를 삽입할 수 있다.

### DVL+자이로 프리인테그레이션 확장 위치
`PreintegrateDvlGro3()` (L.1629) → `mpIntegrator->IntegrateMeasurements()`로 위임됨. 소나 데이터를 함께 넘기려면 `Integrator::IntegrateMeasurements()`의 인터페이스 확장이 필요하다.

### 트래킹 최적화 확장 위치
`TrackLocalMapWithDvlGyro()` (L.5111)가 소나 항을 추가하기에 가장 적합한 위치다. 현재 주석 처리된 이 함수를 활성화한 뒤 `mlamda_sonar` 인자를 추가하면 된다.

---

## 주의 사항

- **`TrackLocalMapWithDvlGyro()` 미사용**: 구현은 있지만 `Track()`(L.2410)과 `TrackDVLGyro()`(L.2838) 모두에서 주석 처리. Tracking 수준에서 DVL이 자세 최적화에 기여하지 않는 상태. (LocalMapping의 BA에서만 DVL 사용)

- **`VISUAL_LOST` 상태 미완성**: `mState = VISUAL_LOST` 분기가 주석 처리되어 있음(L.2247). 시각 손실 전용 처리 경로가 없고 RECENTLY_LOST로 통합됨

- **`TrackDVLImu()` 빈 함수**: L.3073에 선언되어 있으나 구현 없음

- **bias 최적화 주석**: `CreateNewKeyFrame()` 내부 `PoseOptimizationDVLIMUBiasOnly` 블록(L.5386–5397)이 주석 처리. KF 생성 시 bias 즉시 갱신이 없음

- **`ResetFrameIMU()` 빈 함수**: L.1986에 선언, 구현 없음 (코드에 `// ResetFrameIMU() no implementation` 주석 있음, L.2480)

- **`PublishOrb()` 조건**: `TrackDVLGyro()`에서는 `mInitialized && !mTcw.empty()` 조건(L.2867)이 있고, `Track()`에서는 `!mTcw.empty()` 조건만 있음(L.2496). `mInitialized`는 LocalMapping의 `InitializeDvlIMU()` 완료 시 `mpTracker->mCalibrated = true`와 함께 세팅됨

- **구 preintegration 경로 잔존**: `PreintegrateDvlGro()` (L.1229)와 `PreintegrateDvlGro2()` (L.1418)는 구 경로이며, 현재 `Track()` 내에서만 사용됨. 주 경로인 `TrackDVLGyro()`는 `PreintegrateDvlGro3()` → `Integrator` 패턴을 사용

- **`LKTracker` 주석 처리**: 생성자에서 `mpLKTracker = new LKTracker(...)` 가 주석 처리됨(L.197). KLT 경로는 실질적으로 비활성화 상태

- **`time_recently_lost` 파라미터**: YAML `Tracker.ReLocalTime`으로 설정. DVL_STEREO 모드에서만 오버라이드됨(L.134). 기본값은 생성자 초기화 리스트에서 `5.0`초
