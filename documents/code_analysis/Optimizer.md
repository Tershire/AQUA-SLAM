# Optimizer.cc / DvlGyroOptimizer.cpp 분석

**파일 위치**: `src/Optimizer.cc` (14,144줄), `src/DvlGyroOptimizer.cpp` (4,929줄)  
**역할**:
- `Optimizer.cc` — ORB-SLAM3 기반 g2o 최적화 루틴 전체. 시각 전용 BA, DVL 전처리 BA, 초기화 최적화, 포즈-온리 추적 등을 담당.
- `DvlGyroOptimizer.cpp` — DVL + Gyro + IMU를 결합한 tightly-coupled BA 루틴. LocalMapping의 핵심 BA 분기에서 호출되는 AQUA-SLAM 핵심 기여 코드.

---

## 함수 목록

### Optimizer.cc

| 함수 | 역할 |
|---|---|
| `GlobalBundleAdjustemnt(pMap, nIterations, ...)` | 전체 맵에 대한 global BA (시각 전용) |
| `BundleAdjustment(vpKFs, vpMP, ...)` | 지정 KF/MP 집합에 대한 BA |
| `FullInertialBA(pMap, ...)` | 전체 맵에 대한 IMU 포함 BA (ORB-SLAM3) |
| `PoseOptimization(pFrame)` | 단일 프레임 포즈 추정 (MapPoint 고정, 시각 전용) |
| `PoseOnlyOptimizationDVLIMU(loss_kfs, ...)` | 손실 KF들에 대한 포즈-온리 DVL+IMU 최적화 |
| `OptimizationDVLIMU(loss_kfs, ...)` | KF 집합 전체에 DVL+IMU 결합 최적화 |
| `PoseOptimizationWithBA_and_EKF(pFrame, pLastFrame, ...)` | 현재+이전 프레임에 DVL·EKF를 결합한 포즈 최적화 |
| `PoseOptimizationWithBA_and_EKF2(...)` | 위의 변형 버전 |
| `PoseOptimizationWithEKF(pFrame, pLastFrame)` | EKF 기반 포즈 최적화 |
| `LocalBundleAdjustment(pKF, pbStopFlag, vpNonEnoughOptKFs)` | 시각 전용 Local BA (초기화 전 사용, ORB-SLAM3 기본) |
| `LocalBundleAdjustment(pKF, pbStopFlag, pMap, num_fixedKF)` | 위의 Map 버전 오버로드 |
| `LocalDVLBundleAdjustment(pKF, pbStopFlag, pMap, num_fixedKF)` | DVL 상대변환 edge 포함 Local BA (IMU 없음) |
| `LocalDVLRefinement(pKF, ...)` | 전체 맵에 걸쳐 DVL edge로 정제 |
| `OptimizeEssentialGraph(...)` | Loop closing 후 Essential Graph 최적화 (다수 오버로드) |
| `OptimizeSim3(...)` | Sim3 최적화 (다수 오버로드) |
| `LocalInertialBA(pKF, ...)` | ORB-SLAM3 IMU 포함 Local BA |
| `InertialOptimization(...)` | ORB-SLAM3 IMU 초기화 최적화 (다수 오버로드) |
| `DvlIMUInitOptimization(pMap, priori_g, priori_a)` | DVL+IMU 초기화: 자이로 바이어스·중력 방향·속도 추정 |
| `DvlIMURefineOptimization(pAtlas)` | 초기화 이후 refine 최적화 |
| `DvlBeamOptimization(pMap)` | DVL 빔 방향 교정 최적화 |
| `DvlBeamOptimization_dvl(pMap)` | DVL 빔 방향 교정 변형 버전 |
| `DvlGyroInitOptimization(pMap, bg, ...)` | 자이로+DVL 외부 파라미터 초기화 (버전 1~6) |
| `DvlGyroInitOptimization2/3/4/5/6(...)` | 위의 변형 버전들 |
| `PoseDvlGyrosOPtimizationLastFrame(pFrame, ...)` | 단일 프레임 포즈 추적 (DVL+Gyro 결합, 마지막 프레임 기준) |
| `PoseDvlGyrosOPtimizationLastKeyFrame(pFrame, ...)` | 단일 프레임 포즈 추적 (DVL+Gyro 결합, 마지막 KF 기준) |
| `PoseInertialOptimizationLastKeyFrame(pFrame, ...)` | ORB-SLAM3 IMU 포즈 추적 (마지막 KF 기준) |
| `PoseInertialOptimizationLastFrame(pFrame, ...)` | ORB-SLAM3 IMU 포즈 추적 (마지막 프레임 기준) |
| `MergeBundleAdjustmentVisual(...)` | 맵 병합 시 시각 BA |
| `MergeInertialBA(...)` | 맵 병합 시 IMU 포함 BA |
| `GlobalVAPoseGraphOptimization(...)` | 전역 포즈 그래프 최적화 |
| `Marginalize/Condition/Sparsify(...)` | 정보 행렬 조작 유틸 (marginalization 보조) |

### DvlGyroOptimizer.cpp

| 함수 | 역할 |
|---|---|
| `LocalDVLBundleAdjustment(pKF, pbStopFlag, pMap, num_fixedKF)` | DVL 상대변환 edge + 시각 BA (IMU 없음, 초기화 전 사용) |
| `LocalDVLGyroBundleAdjustment(pKF, ...)` | DVL + Gyro preintegration BA |
| `LocalDVLIMUBundleAdjustment(pAtlas, pKF, ...)` | **메인 tight BA**: 시각 + DVL + IMU + Gyro 결합 (초기화 후 사용) |
| `LocalDVLIMUBundleAdjustment2(pAtlas, pKF, ...)` | 위의 변형 버전 (속도 edge 추가, EdgeDvlVelocity 적용) |
| `FullDVLIMUBundleAdjustment(pAtlas, pKF, ...)` | 전체 맵에 걸친 DVL+IMU full BA |
| `LocalDVLIMUPoseGraph(pAtlas, pKF, pMap)` | DVL+IMU 포즈 그래프 최적화 |
| `FullDVLGyroBundleAdjustment(pbStopFlag, pMap, lamda_DVL)` | 전체 맵 DVL+Gyro BA |
| `PoseDvlGyrosOPtimizationLastFrame(pFrame, ...)` | 단일 프레임 포즈 추적 (DVL+Gyro) |
| `PoseDvlGyrosOPtimizationLastKeyFrame(pFrame, ...)` | 단일 프레임 포즈 추적 (DVL+Gyro, 마지막 KF 기준) |
| `DvlGyroInitOptimization(pMap, bg, ...)` | DVL+Gyro 초기화: 외부 파라미터 T_dvl_c 추정 |

---

## 중요 함수 상세 설명

### `LocalBundleAdjustment(pKF, pbStopFlag, pMap, num_fixedKF)` — 시각 전용 BA

**위치**: `Optimizer.cc` L.3354  
**호출 시점**: IMU 초기화 전, LocalMapping::Run()에서 호출됨.

**구조**:
1. 현재 KF와 covisibility graph 이웃 KF들을 `lLocalKeyFrames`로 수집
2. 이들이 관찰하는 MapPoint들을 `lLocalMapPoints`로 수집
3. LocalMP를 관찰하지만 local KF가 아닌 KF들을 `lFixedCameras`로 고정
4. Fixed KF가 2개 미만이면 가장 오래된 local KF들을 fixed로 이동시킴

**vertex**:
- `g2o::VertexSE3Expmap` — local KF 포즈 (optimizable)
- `g2o::VertexSE3Expmap` — fixed KF 포즈 (fixed)
- `g2o::VertexSBAPointXYZ` — MapPoint 3D 위치 (marginalized)

**edge**:
- `EdgeSE3ProjectXYZ` — 모노 재투영 잔차 (2D), Huber kernel δ=√5.991
- `g2o::EdgeStereoSE3ProjectXYZ` — 스테레오 재투영 잔차 (3D), Huber kernel δ=√7.815
- `EdgeSE3ProjectXYZToBody` — 우측 카메라 재투영 잔차

**최적화 일정**: 5회 → 아웃라이어 확인 → 10회 재최적화  
**아웃라이어 임계**: 모노 chi2 > 5.991, 스테레오 chi2 > 7.815  
**주의**: 아웃라이어가 전체의 50% 이상이면 즉시 return (KF/MP 업데이트 없음).

---

### `LocalDVLBundleAdjustment(pKF, pbStopFlag, pMap, num_fixedKF)` — DVL 전처리 BA

**위치**: `Optimizer.cc` L.3944  
**호출 시점**: 초기화 전 DVL 전용 BA로 사용 (Optimizer.cc 버전).  
**특이점**: `pKF->IntegrateDVL(pKF->mPrevKF)`를 vertex 추가 전에 호출하여 DVL pre-integration을 수행.

**시각 전용 BA와의 차이점**:
- KF 윈도우 방식: covisibility graph 대신 시간순 역방향으로 최대 Nd=10개 KF 수집
- 추가 edge: `EdgeSE3DVLBA` — 인접 KF 쌍(i-1, i) 사이에 DVL 상대변환 잔차 추가
  ```cpp
  EdgeSE3DVLBA *e = new EdgeSE3DVLBA(p_cur->mT_ei_ej, p_cur->mT_e_c);
  e->setVertex(0, vertex(p_pre->mnId));
  e->setVertex(1, vertex(p_cur->mnId));
  e->setInformation(I_6x6 * 50,000,000);
  ```
  - `mT_ei_ej`: DVL 측정으로 구한 KF 간 상대 변환
  - `mT_e_c`: DVL-카메라 외부 파라미터

---

### `DvlIMUInitOptimization(pMap, priori_g, priori_a)` — 초기화 최적화

**위치**: `Optimizer.cc` L.13523  
**호출 시점**: `LocalMapping::InitializeDvlIMU()` 내부.  
**반환값**: DVL edge들의 평균 오차 (`avg_dvl`). LocalMapping에서 초기화 성공 여부 판단에 사용.

**vertex**:
- `VertexPoseDvlIMU` — 모든 KF 포즈 (**fixed**, 시각 BA 결과로 고정)
- `VertexVelocity` — 각 KF의 속도 (optimizable)
- `VertexGyroBias` (단일 공유) — 자이로 바이어스 (2단계 최적화)
- `VertexAccBias` (단일 공유) — 가속도계 바이어스 (2단계에서 unfixed)
- `g2o::VertexSE3Expmap` vT_d_c — DVL-카메라 외부 파라미터 (fixed)
- `g2o::VertexSE3Expmap` vT_g_d — Gyro-DVL 외부 파라미터 (fixed)
- `VertexGDir` — 중력 방향 R_b0_w (optimizable)

**edge**:
- `EdgePriorGyro` — 자이로 바이어스 prior (정보행렬 = priori_g * I₃)
- `EdgePriorAcc` — 가속도계 바이어스 prior (정보행렬 = priori_a * I₃)
- `EdgeDvlIMU` — KF 쌍 사이의 DVL+IMU preintegration 잔차 (9D)
  - 연결: VP1, VP2, VV1, VV2, VG, VA, VT_d_c, VT_g_d, VR_w_b0
  - 정보행렬: preintegration 공분산 C[0:9, 0:9]의 역행렬

**최적화 일정**: 20회 (VG, VA fixed) → 20회 (VG, VA unfixed)  
**결과**: 모든 KF의 bias를 동일한 VG, VA 값으로 업데이트.

---

### `PoseOptimization(pFrame)` — 단일 프레임 포즈 추정

**위치**: `Optimizer.cc` L.897  
**호출 시점**: Tracking에서 매 프레임 포즈 추적 시 (IMU 미사용 또는 초기화 전).

**구조**: MapPoint를 고정하고 현재 프레임의 포즈만 최적화.

**vertex**:
- `g2o::VertexSE3Expmap` — 현재 프레임 포즈 (단일 vertex, id=0)

**edge**:
- `EdgeSE3ProjectXYZOnlyPose` — 모노 재투영 잔차 (2D)
- `EdgeSE3ProjectXYZOnlyPoseToBody` — 우측 카메라 재투영 잔차
- `g2o::EdgeStereoSE3ProjectXYZOnlyPose` — 스테레오 재투영 잔차 (3D)

**최적화 일정**: 4 × 10회 반복. 각 라운드 후 아웃라이어 표시, 3라운드 이후 robust kernel 제거.  
**반환값**: inlier 수 (`nInitialCorrespondences - nBad`).

---

### `LocalDVLIMUBundleAdjustment(pAtlas, pKF, pbStopFlag, pMap, num_fixedKF, lamda_DVL, lamda_visual)` — 메인 Tight BA

**위치**: `DvlGyroOptimizer.cpp` L.1077  
**호출 시점**: IMU 초기화 완료 후, `LocalMapping::Run()`에서 호출됨.  
**인자**: `lamda_DVL`, `lamda_visual` — YAML 파라미터 (`mlamda_DVL`, `mlamda_visual`), 정보행렬 가중치로 사용.

**KF 윈도우**: 현재 KF부터 시간 역순으로 최대 Nd=10개 OptKFs 수집. FixedKFs = OptKFs 경계 이전 KF + 연결 KF 최대 50개.

**vertex 구성** (per KF):
| vertex | id 공식 | fixed 여부 |
|---|---|---|
| `VertexPoseDvlIMU` | `pKFi->mnId` | OptKFs: false, FixedKFs: true |
| `VertexGyroBias` | `maxKFid+1 + pKFi->mnId` | OptKFs: false, FixedKFs: true |
| `VertexAccBias` | `(maxKFid+1)*2 + pKFi->mnId` | IMU캘리완료 시 false, 미완료 시 true |
| `VertexVelocity` | `(maxKFid+1)*3 + pKFi->mnId` | IMU캘리완료 시 false, 미완료 시 true |
| `g2o::VertexSBAPointXYZ` | `(maxKFid+1)*5 + i` | LocalFixedMP면 true |

**공유 vertex** (전체 1개):
- vT_d_c `(maxKFid+1)*4` — T_dvl_c (fixed)
- vT_g_d `(maxKFid+1)*4+1` — T_imu_dvl (fixed)
- VGDir `(maxKFid+1)*4+2` — 중력 방향 (fixed, 초기화 완료 후 사용)

**edge 구성**:

| edge 타입 | 잔차 차원 | 연결 vertex | 가중치 |
|---|---|---|---|
| `EdgeMonoBA_DvlGyros` | 2D | VP_KFi + VPoint | `invSigma2 * lamda_visual` |
| `EdgeStereoBA_DvlGyros` | 3D | VP_KFi + VPoint | `invSigma2 * lamda_visual` |
| `EdgeAccRW` | 3D | VA_prev + VA_cur | preintegration C[12:15,12:15]⁻¹ |
| `EdgeGyroRW` | 3D | VG_prev + VG_cur | preintegration C[9:12,9:12]⁻¹ |
| `EdgeDvlIMU2` | 9D (vi, vj, P_dvl) | VP1+VP2+VV1+VV2+VG+VA+VT_d_c+VT_g_d+VR_b0_w | 고정값 diag(1e5, 1e5, 1e5) |
| `EdgeDvlIMU` | 9D (R, V, P_acc) | VP1+VP2+VV1+VV2+VG+VA+VT_d_c+VT_g_d+VR_b0_w | preintegration C[0:9,0:9]⁻¹ (캘리완료 시) |

**EdgeDvlIMU (9D) 잔차 내용**:
```
e_R = Log(dR^T * R_b_c * R_ci_cj * R_c_b)         ← 회전 residual
e_V = R_b_c * R_ci * (R_cj*R_c_dvl*vj - ...) - dV  ← 속도 residual
e_P = R_b_c * R_ci * (P_cj - P_ci - ...) - dP_acc  ← 위치 residual (IMU)
```

**EdgeDvlIMU2 (9D) 잔차 내용**:
```
e_vi = vdi - mVelocity_i   ← DVL 측정속도 vs. 최적화 속도 (KF i)
e_vj = vdj - mVelocity_j   ← DVL 측정속도 vs. 최적화 속도 (KF j)
e_P  = P_dvl_est - dP_dvl  ← DVL preintegration 위치 residual
```

**최적화 일정**: 20회 초기 최적화 → 아웃라이어 표시 (mono chi2>10, stereo chi2>20) → 4 × 10회 재최적화  
**결과 회수**: 포즈, bias, DVL velocity를 각 KF에 반영. MapPoint 위치 업데이트.

**`LocalDVLIMUBundleAdjustment2`와의 차이**:
- `LocalDVLIMUBundleAdjustment` — EdgeDvlIMU + EdgeDvlIMU2 모두 사용
- `LocalDVLIMUBundleAdjustment2` — EdgeDvlVelocity (DVL 측정속도를 직접 속도 vertex에 구속) + EdgeDvlGyroBA 사용

---

### DVL 관련 g2o edge 요약

| edge | 차원 | 설명 |
|---|---|---|
| `EdgeSE3DVLBA` | 6D | KF 쌍 간 DVL 상대변환 잔차. `LocalDVLBundleAdjustment`에서 사용 |
| `EdgeDvlIMU` | 9D | DVL+IMU preintegration 통합 잔차 (R, V, P_acc). 9-vertex edge |
| `EdgeDvlIMU2` | 9D | DVL 속도 측정 + DVL 위치 preintegration 잔차 (vi, vj, P_dvl). 9-vertex edge |
| `EdgeDvlVelocity` | 3D | DVL 측정 속도 → VertexVelocity 직접 구속. `computeError()` = `mV - VG->estimate()` |
| `EdgeDvlGyroBA` | 6D | DVL+Gyro preintegration 잔차. `LocalDVLIMUBundleAdjustment2`에서 사용 |
| `EdgeSE3DVLIMU` | 6D | VertexPoseDvlIMU 쌍 사이 상대 SE3 구속 |

---

## VISO 통합 관점: EdgeSonarPoint 추가 위치

`EdgeSonarPoint` (소나 점 재투영 잔차)를 추가해야 할 구체적 위치:

### 1. 메인 tight BA: `LocalDVLIMUBundleAdjustment` — `DvlGyroOptimizer.cpp` L.1331–1455

시각 edge 추가 블록(`unique_lock<mutex> lock(MapPoint::mGlobalMutex)` 이후) 바로 다음, L.1455와 L.1460 사이에 삽입:

```cpp
// ─── VISO 통합 포인트 ───────────────────────────────
// EdgeSonarPoint를 여기에 추가
// SonarPoint는 pKF->mvpSonarPoints 등에서 가져옴
for (auto sonarPt : pKFi->mvpSonarPoints) {
    EdgeSonarPoint *e = new EdgeSonarPoint(sonarPt->GetWorldPos());
    e->setVertex(0, optimizer.vertex(pKFi->mnId));  // VertexPoseDvlIMU
    e->setMeasurement(sonarPt->GetMeasurement());
    e->setInformation(... * lamda_sonar);
    optimizer.addEdge(e);
}
// ────────────────────────────────────────────────────
```

### 2. 포즈-온리 추적: `PoseDvlGyrosOPtimizationLastFrame` — `Optimizer.cc` L.10945

시각 edge 추가 블록 이후, 최적화 시작(`optimizer.initializeOptimization`) 전에 삽입.

> 상세 분석은 아래 별도 섹션 참조 → [PoseDvlGyrosOPtimizationLastFrame/LastKeyFrame 상세](#posedvlgyrosoptimizationlastframelastkeyframe--단일-프레임-포즈-추적-dvlgyro)

### 3. 초기화 전 경로: `LocalBundleAdjustment` — `Optimizer.cc` L.3354

MapPoint loop 이후 (L.3664 직후)에 삽입. 단, 초기화 전에는 `VertexPoseDvlIMU` 대신 `VertexSE3Expmap`을 사용하므로 edge vertex 타입 주의.

### 4. DVL 전용 BA: `LocalDVLBundleAdjustment` — `Optimizer.cc` L.3944

DVL edge 추가 블록 (L.4249–4266) 이후에 삽입.

> **인자 추가 필요**: `LocalDVLIMUBundleAdjustment`에 `double lamda_sonar`를 추가하고, `LocalMapping::Run()`에서 `mpTracker->mlamda_sonar`를 함께 전달해야 함.

---

### `PoseDvlGyrosOPtimizationLastFrame/LastKeyFrame` — 단일 프레임 포즈 추적 (DVL+Gyro)

**위치**: `Optimizer.cc` L.10945 (LastFrame), L.11288 (LastKeyFrame)  
**호출 시점**: `TrackLocalMapWithDvlGyro()` 내부 — 현재 **주석 처리**되어 호출되지 않음.  
**차이**: LastFrame은 이전 프레임(pFrame->mpPrevFrame)을 기준으로, LastKeyFrame은 이전 키프레임을 기준으로 DVL+Gyro 잔차를 구성.

**g2o 그래프 구성**:

| vertex | 타입 | fixed 여부 |
|---|---|---|
| 현재 프레임 포즈 | `VertexPoseDvlIMU` | **최적화 대상** |
| 이전 프레임/KF 포즈 | `VertexPoseDvlIMU` | **고정** |
| 자이로 바이어스 | `VertexGyroBias` | **고정** (추정 안 함) |
| T_dvl_c 외부 교정 | `g2o::VertexSE3Expmap` | 고정 |
| T_imu_dvl 외부 교정 | `g2o::VertexSE3Expmap` | 고정 |

**edge 구성**:

| edge | 잔차 | 비고 |
|---|---|---|
| `EdgeMonoOnlyPose_DvlGyros` | 2D 재투영 | 시각 edge |
| `EdgeStereoOnlyPose_DvlGyros` | 3D 재투영 | 시각 edge |
| `EdgeDvlGyroTrack` | 6D (rot 3 + trans 3) | DVL+Gyro 상대 포즈 구속 |

**`EdgeDvlGyroTrack` 잔차 내용** (`G2oTypes.cc` L.2520):
```
R_est  = R_gyros_dvl · R_dvl_c · R_ci_cj · R_c_dvl · R_dvl_gyros  ← 카메라 기반 상대 회전
t_est  = R_dvl_c · (R_ci_cj · R_c_dvl · t_dvl_c - t_dvl_c + R_ci_cj · Δt_c)  ← 카메라 기반 상대 이동

e_R = LogSO3(dR_gyro.T · R_est)  ← gyro preintegration vs. 카메라 회전 불일치
e_t = t_est - dP_dvl             ← DVL preintegration vs. 카메라 이동 불일치
```

DVL+gyro 측정과 카메라 추정 간 상대 포즈 불일치를 하나의 g2o 그래프에서 시각 잔차와 함께 최소화 → **구조상 tightly-coupled**.

**미완성 항목** (`//todo_tightly` 주석 3곳):

| 항목 | 현재 상태 |
|---|---|
| 자이로 바이어스 | 고정 — 갱신 안 됨 |
| 속도 상태 | 없음 ("maybe add velocity to optimization" 주석) |
| IMU prior edge | `EdgePriorPoseImu` 주석 처리 — 이전 프레임의 공분산 전파 없음 |
| Jacobian (linearizeOplus) | 주석 처리 → 수치 미분 fallback |

**`LocalDVLIMUBundleAdjustment`와 비교**:

| 항목 | PoseDvlGyrosOPtimization | LocalDVLIMUBundleAdjustment |
|---|---|---|
| 범위 | 단일 프레임 | 슬라이딩 윈도우 (최대 10 KF) |
| 자이로 바이어스 | 고정 | **최적화** |
| 속도 상태 | 없음 | **최적화** |
| 공분산 전파 | 없음 | preintegration 공분산 사용 |
| 활성 상태 | 미활성 (주석) | **활성** |

---

## 주의 사항

- `LocalDVLIMUBundleAdjustment`에서 **현재 KF 이후(미래 KF)의 포즈도 상대 변환으로 업데이트**한다 (L.1684–1712). 이는 BA 결과를 전체 트랙에 전파하기 위한 처리이며, 소나 포인트 추가 시 이 propagation 로직에 영향을 주지 않도록 주의.
- `EdgeDvlIMU`와 `EdgeDvlIMU2`는 **동일 KF 쌍에 동시에 추가된다** (L.1554–1616). 이 구조는 중복이지만 의도적 설계이며, 두 edge가 다른 residual을 제공한다.
- `VertexAccBias` / `VertexVelocity`는 `IsIMUCalibrated()` 여부에 따라 fixed/unfixed가 바뀐다. 초기화 전에는 속도와 acc bias가 고정되어 있으므로 소나 edge를 추가해도 이들은 최적화되지 않는다.
- `DvlIMUInitOptimization`에서 자이로 바이어스는 **단일 공유 vertex**로 모든 KF에 동일한 값이 적용된다. 초기화 후 `LocalDVLIMUBundleAdjustment`에서는 KF별 개별 bias를 사용한다.
- `LocalDVLBundleAdjustment` (`Optimizer.cc` 버전, L.3944)와 `DvlGyroOptimizer::LocalDVLBundleAdjustment` (L.37)는 **이름이 같고 역할이 유사하지만 다른 함수**이다. 전자는 `EdgeSE3DVLBA`, 후자는 `EdgeDvlGyroBA` 기반이다.
- chi2 임계값: `LocalDVLIMUBundleAdjustment`에서 mono 10, stereo 20 — `LocalBundleAdjustment`의 5.991/7.815와 다름.
- `mPoorVision` 플래그가 세워진 KF는 시각 edge 가중치가 동일하게 유지된다 (현재 코드에서는 1배 적용, 원래 의도는 증폭). 소나를 추가할 때 이 KF에서는 소나 가중치를 높이는 것이 자연스러운 확장이다.
