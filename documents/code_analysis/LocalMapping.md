# LocalMapping.cc 분석

**파일 위치**: `src/LocalMapping.cc`  
**전체 라인**: 1831줄  
**역할**: 키프레임 단위 지도 최적화 스레드. Tracking에서 넘어온 KF를 받아 MapPoint를 관리하고 BA를 수행한다.

---

## 메인 루프

### `Run()`
LocalMapping 스레드의 진입점. 새 키프레임이 들어올 때마다 아래 순서로 처리한다.

```
ProcessNewKeyFrame()
  → MapPointCulling()
  → CreateNewMapPoints()
  → SearchInNeighbors()
  → Bundle Adjustment  ← IMU 초기화 여부에 따라 분기
  → InitializeDvlIMU() (조건부)
  → mpLoopCloser->InsertKeyFrame()
```

---

## 함수 목록

### KF 처리

| 함수 | 역할 |
|---|---|
| `InsertKeyFrame(pKF)` | Tracking이 호출. KF를 큐(`mlNewKeyFrames`)에 넣고 BA 중단 플래그 세움 |
| `CheckNewKeyFrames()` | 큐에 새 KF가 있는지 확인 |
| `ProcessNewKeyFrame()` | 큐에서 KF를 꺼내 BoW 계산, MapPoint 관계 갱신, Covisibility Graph 업데이트, Atlas에 등록 |
| `EmptyQueue()` | 큐를 비울 때까지 ProcessNewKeyFrame() 반복 |

### MapPoint 관리

| 함수 | 역할 |
|---|---|
| `MapPointCulling()` | 최근 추가된 MP 중 나쁜 것 제거. 기준: `FoundRatio < 0.25`, KF 2개 이후 관찰 수 부족, KF 3개 이후 무조건 졸업 |
| `CreateNewMapPoints()` | covisibility graph 이웃 KF 10개와 epipolar geometry로 삼각측량 → 새 MP 생성 |
| `SearchInNeighbors()` | 이웃 KF들 간 MP 중복을 찾아 fusion (같은 점 여러 개 → 하나로 합침) |

### Bundle Adjustment — 핵심 분기 (L.148–167)

```cpp
if (!mpAtlas->IsIMUCalibrated()) {
    // IMU 초기화 전: 순수 시각 BA (ORB-SLAM3 기본)
    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, ...)
} else {
    // IMU 초기화 후: DVL + IMU tightly coupled BA  ← AQUA-SLAM 핵심 기여
    DvlGyroOptimizer::LocalDVLIMUBundleAdjustment(
        mpAtlas, mpCurrentKeyFrame, ...,
        mpTracker->mlamda_DVL,      // λ_d  — YAML 파라미터
        mpTracker->mlamda_visual    // λ_v  — YAML 파라미터
    )
}
```

> **VISO 통합 포인트**: 이 분기에 `mlamda_sonar`을 추가 인자로 넘기면 된다.

### IMU/DVL 초기화

| 함수 | 역할 |
|---|---|
| `InitializeDvlIMU()` | DVL+IMU 초기화 메인 함수. `DvlIMUInitOptimization()`으로 자이로 바이어스 + 중력 방향 추정. 평균 오차 / 이동량 기준으로 3단계 분기 처리 |
| `InitializeDvlGyro(priorG, bFirst)` | 자이로 바이어스만 초기화 (InitializeDvlIMU 내부에서 사용) |
| `RefineGravityDvlIMU()` | KF 200개 / 400개 시점에 중력 방향 재추정. 장시간 운용 시 점진적 개선 |

#### InitializeDvlIMU() 분기 요약 (L.1679–1757)

| 조건 | 처리 |
|---|---|
| 이동량 부족 | 기본 초기화만 (FullBA 없음, bias 리셋) |
| avg error < 0.01 + KF > 20 | **완전 초기화** + FullBA |
| avg error < 0.1 | 중간 초기화 + FullBA |
| 나머지 | 강제 초기화 (FullBA 없음) |

초기화 완료 시 세워지는 플래그:
- `mpAtlas->SetIMUCalibrated()` → 다음 BA부터 `LocalDVLIMUBundleAdjustment()` 사용
- `mpTracker->mCalibrated = true`
- `mpAtlas->SetDvlImuInitialized()`

### 제어 / 유틸

| 함수 | 역할 |
|---|---|
| `KeyFrameCulling()` | 중복 KF 제거 — **현재 주석 처리됨** (L.246–248). KF가 계속 쌓임 |
| `FullBA()` | 전체 map에 대해 global BA 수행. 초기화 완료 후 1회 호출됨 |
| `GetTravelDistance()` | 현재 KF들로부터 총 이동 거리(m)와 회전량(rad) 계산. 초기화 조건 판단에 사용 |
| `ResetKFBias()` | 모든 KF의 IMU bias를 0으로 초기화 |
| `RequestStop()` / `Stop()` / `Release()` | LoopClosing의 map merging 시 LocalMapping 일시 정지 |
| `RequestReset()` / `ResetIfRequested()` | 추적 실패 시 전체 리셋 |
| `ComputeF12()` | 두 KF 간 Fundamental matrix 계산. CreateNewMapPoints() 내부에서 사용 |

---

## 스레드 간 관계

```
Tracking ──InsertKeyFrame()──► LocalMapping (이 파일)
                                    │
                                    ├── Atlas (R/W)
                                    ├── Optimizer / DvlGyroOptimizer (BA 호출)
                                    └── InsertKeyFrame() ──► LoopClosing
```

- **Tracking → LocalMapping**: `InsertKeyFrame()`으로 KF 전달
- **LocalMapping → LoopClosing**: `mpLoopCloser->InsertKeyFrame()`으로 매 KF 전달
- **LocalMapping ↔ Atlas**: KF 추가, 전체 KF 조회, 초기화 플래그 관리
- **RosHandling**: LocalMapping에서 직접 발행하지 않음. Atlas를 통해 RosHandling::Run()이 polling

---

## 주의 사항

- `KeyFrameCulling()`이 주석 처리되어 있어 장시간 운용 시 KF가 무한히 누적됨
- `PublishIntegration()` 호출도 모두 주석 처리 — LocalMapping에서 ROS 발행 없음
- 초기화 조건(`mInitTranslationThred`, `mInitRotationThred`)은 YAML에서 로드
