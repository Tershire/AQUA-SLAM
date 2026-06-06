# G2oTypes.h / G2oTypes.cc 분석

**파일 위치**: `include/G2oTypes.h`, `src/G2oTypes.cc`  
**전체 라인**: 헤더 2043줄 / 구현 2907줄  
**역할**: AQUA-SLAM의 g2o 기반 최적화에 쓰이는 모든 Vertex·Edge 클래스 정의. ORB-SLAM3 원본 타입(IMU 관련)에 DVL/DVL+IMU 연동 타입을 추가·확장했다.

---

## g2o 구조 개요

```
Vertex (최적화 변수)
  ├── 포즈계: VertexPose, VertexPoseDvlIMU, VertexPose4DoF
  ├── 운동량: VertexVelocity
  ├── IMU bias: VertexGyroBias, VertexAccBias
  ├── 캘리브레이션: VertexDVLBeamOritenstion, VertexGDir, VertexScale
  └── 역깊이: VertexInvDepth

Edge (잔차 제약)
  ├── 시각 재투영: EdgeMono*, EdgeStereo*, EdgeMonoBA_DvlGyros, EdgeStereoBA_DvlGyros
  ├── IMU 프리인테그레이션: EdgeInertial, EdgeInertialGS
  ├── DVL+자이로 초기화: EdgeDvlGyroInit/2/3, EdgeDvlGyroTrack, EdgeDvlGyroBA
  ├── DVL+IMU (주요 BA): EdgeDvlIMU, EdgeDvlIMU2, EdgeDvlIMUWithBias
  ├── DVL+IMU 초기화: EdgeDvlIMUInitWithoutBias, EdgeDvlIMUInit
  ├── 중력 보정: EdgeDvlIMUGravityRefine, EdgeDvlIMUGravityRefineWithBias
  ├── DVL 속도 직접 제약: EdgeDvlVelocity
  ├── DVL 빔 캘리브레이션(dead code): EdgeDVLBeamCalibration1/2
  ├── EKF 연동 보조: EdgeSE3DVLPoseOnly/2, EdgeSE3DVLBA, EdgeSE3DVLIMU, EdgeDVLRefine
  ├── bias 랜덤워크: EdgeGyroRW, EdgeAccRW
  ├── 사전(prior): EdgePriorPoseImu, EdgePriorAcc, EdgePriorGyro
  ├── 포즈그래프: Edge4DoF
  └── 궤적 정렬: EdgeTrajAlign
```

---

## Vertex 목록

| 클래스 | DoF | 추정값 타입 | 최적화 변수 |
|---|---|---|---|
| `VertexPose` | 6 | `ImuCamPose` | IMU 바디 포즈 (R_wb, t_wb). ORB-SLAM3 원본. IMU 좌표계 기준 업데이트 |
| `VertexPoseDvlIMU` | 6 | `DvlImuCamPose` | 카메라 포즈 (R_wc, t_wc). AQUA-SLAM 핵심 포즈 타입. DVL/IMU 외부 파라미터 포함 |
| `VertexPose4DoF` | 4 | `ImuCamPose` | 평면 운동 가정 포즈 — yaw + translation 3DoF만 최적화. 루프 클로징 포즈그래프용 |
| `VertexVelocity` | 3 | `Eigen::Vector3d` | 속도 벡터 (월드 좌표계). DVL 측정 또는 IMU 적분으로 구속 |
| `VertexGyroBias` | 3 | `Eigen::Vector3d` | 자이로스코프 bias |
| `VertexAccBias` | 3 | `Eigen::Vector3d` | 가속도계 bias |
| `VertexDVLBeamOritenstion` | 8 | `Eigen::Matrix<double,8,1>` | 4빔 DVL 각도 (alpha_0..3, beta_0..3). `EdgeDVLBeamCalibration` 전용 |
| `VertexGDir` | 2 | `GDirection` | 중력 방향 행렬 R_wg. yaw를 고정하고 roll/pitch만 2DoF로 업데이트 |
| `VertexScale` | 1 | `double` | 단안 스케일. 로그 공간에서 업데이트: `s *= exp(delta)` |
| `VertexInvDepth` | 1 | `InvDepthPoint` | 역깊이 포인트 (호스트 KF 기준) |

### 핵심 포즈 타입 비교

**`ImuCamPose`** (ORB-SLAM3 원본)
- 내부 변수: `Rwb`, `twb` (IMU 바디 포즈)
- 카메라 투영: `Rcw`, `tcw` 파생
- `Update()`: IMU 좌표계 기준 업데이트

**`DvlImuCamPose`** (AQUA-SLAM 추가)
- 내부 변수: `Rwc`, `twc` (카메라 포즈 직접)
- DVL·IMU 외부 파라미터 저장: `R_c_imu`, `R_imu_c`, `R_c_dvl`, `R_dvl_c`, `t_c_dvl`, `t_dvl_c`
- 스테레오 기저선 변환 `T_r_l` 포함
- boost serialization 지원 (맵 저장/로드)

---

## Edge 목록

### 시각 재투영 Edge

| 클래스 | 차원 | 연결 Vertex | 잔차 의미 |
|---|---|---|---|
| `EdgeMono` | 2 | `VertexSBAPointXYZ` + `VertexPose` | 단안 재투영 오차. ORB-SLAM3 원본. `linearizeOplus` 해석적 Jacobian 구현 |
| `EdgeMonoOnlyPose` | 2 | `VertexPose` | 포즈만 최적화, MP 고정. Tracking용 |
| `EdgeMonoOnlyPose_DvlGyros` | 2 | `VertexPoseDvlIMU` | `EdgeMonoOnlyPose`의 DvlIMU 버전. `linearizeOplus` 미구현 (수치 미분) |
| `EdgeMonoBA_DvlGyros` | 2 | `VertexPoseDvlIMU` + `VertexSBAPointXYZ` | BA용 단안. `linearizeOplus` 미구현 |
| `EdgeStereo` | 3 | `VertexSBAPointXYZ` + `VertexPose` | 스테레오 재투영. ORB-SLAM3 원본 |
| `EdgeStereoOnlyPose` | 3 | `VertexPose` | 스테레오 포즈만. ORB-SLAM3 원본 |
| `EdgeStereoOnlyPose_DvlGyros` | 3 | `VertexPoseDvlIMU` | 스테레오 포즈만. DvlIMU 버전 |
| `EdgeStereoBA_DvlGyros` | 3 | `VertexPoseDvlIMU` + `VertexSBAPointXYZ` | BA용 스테레오. `linearizeOplus` 미구현 |

### IMU 프리인테그레이션 Edge (ORB-SLAM3 원본)

| 클래스 | 차원 | 연결 Vertex (순서) | 잔차 의미 |
|---|---|---|---|
| `EdgeInertial` | 9 | VP1, VV1, VG1, VA1, VP2, VV2 | 두 KF 간 IMU 적분 제약. 회전(3) + 속도(3) + 위치(3). 해석적 Jacobian 구현 |
| `EdgeInertialGS` | 9 | VP1, VV1, VG1, VA1, VP2, VV2, VGDir, VScale | IMU + 중력방향 + 스케일 동시 최적화. 초기화용 |

**`EdgeInertial::computeError()` 요약**:
```
e_R = Log(dR^T * R_b1w^T * R_b2w)         // 회전 오차
e_V = R_b1w^T * (v2 - v1 - g*dt) - dV    // 속도 오차
e_P = R_b1w^T * (p2 - p1 - v1*dt - 0.5*g*dt^2) - dP  // 위치 오차
```
where `dR, dV, dP` = 바이어스 보정된 프리인테그레이션 값

### DVL+자이로 초기화 Edge

이 Edge 군은 `DvlGyroInitOptimization*()` 함수에서 사용되며, IMU가 초기화되기 전 자이로 바이어스와 DVL-카메라 외부 파라미터를 추정하는 용도다.

| 클래스 | 차원 | Vertex 수 | 역할 |
|---|---|---|---|
| `EdgeDvlGyroInit` | 6 | 5개 (VP1, VP2, V_bw, VT_d_c, VT_g_d) | 자이로 바이어스·DVL extrinsic 동시 추정. `dP_dvl` 기반 위치 잔차 |
| `EdgeDvlGyroInit2` | 6 | 6개 (+VertexDVLBeamOritenstion) | 빔 각도도 동시 추정. **e_R 항이 상수 0으로 하드코딩** |
| `EdgeDvlGyroInit3` | 6 | 6개 (+VertexVelocity) | 속도를 추가 최적화. **e_R 항이 상수 0으로 하드코딩** |
| `EdgeDvlGyroTrack` | 6 | 5개 | 초기화 후 Tracking 단계용. `dR`, `dP_dvl` 직접 사용 |
| `EdgeDvlGyroBA` | 6 | 5개 | LocalBA용 자이로 + DVL 제약. 회전(3) + 위치(3) 잔차 |

공통 잔차 패턴 (EdgeDvlGyroInit/BA):
```
R_est = R_g_d * R_d_c * R_ci_c0 * R_c0_cj * R_c_d * R_d_g   // 회전 추정
t_est = t_d_c - R_d_c * R_ci_c0 * R_c0_cj * R_c_d * t_d_c
        + R_d_c * (R_ci_c0 * t_c0_cj - R_ci_c0 * t_c0_ci)   // 위치 추정
e_R = Log(dR^T * R_est)
e_p = t_est - dP_dvl
```

### DVL+IMU 주요 BA Edge

실제 `LocalDVLIMUBundleAdjustment`에서는 `EdgeSE3DVLBA`를 사용하고, 초기화 최적화에서 아래 Edge들이 사용된다.

| 클래스 | 차원 | Vertex 수 | 역할 |
|---|---|---|---|
| `EdgeDvlIMU` | 9 | 9개 (VP1, VP2, VV1, VV2, VG, VA, VT_d_c, VT_g_d, VR_G) | 핵심 DVL+IMU 제약. 회전(3)+속도(3)+위치(3). `linearizeOplus` 주석 처리 → 수치 미분 |
| `EdgeDvlIMUWithBias` | 9 | 9개 (동일) | `EdgeDvlIMU`와 구조 동일. 위치 항에서 `dP_dvl` 대신 `dP_acc` (IMU 위치 적분) 사용 |
| `EdgeDvlIMU2` | 9 | 9개 | 두 개의 DVL 속도 측정(`mpInt_i`, `mpInt_j`)을 동시에 처리. 속도 오차(6) + DVL 위치 오차(3) |

**`EdgeDvlIMU::computeError()` 상세** (G2oTypes.cc L.2213):
```cpp
// 1. 외부 파라미터에서 R_b_c 계산
T_b_c = T_gyros_dvl * T_dvl_c  →  R_b_c, t_b_c

// 2. 회전 잔차
R_est = R_b_c * R_ci_c0 * R_c0_cj * R_c_b
e_R   = Log(dR^T * R_est)

// 3. 속도 잔차 (DVL 속도를 IMU 바디 프레임으로 변환)
avg_v = (v1 + v2) / 2
VDelta_est = R_b_c * R_ci_c0 * (R_c0_cj * R_c_dvl * v2
             - R_c0_ci * R_c_dvl * v1 - R_c_b * R_b0w * g * dt)
e_V = VDelta_est - dDelta_V

// 4. 위치 잔차 (IMU 적분 기반)
P_acc_est = R_b_c * R_ci_c0 * (R_c0_cj * t_c_b + t_c0_cj
            - (R_c0_ci * t_c_b + t_c0_ci) - R_c0_ci * R_c_dvl * v1 * dt
            - 0.5 * R_c_b * R_b0w * g * dt^2)
e_P = P_acc_est - dP_acc

_error << e_R, e_V, e_P
```

주목할 점: 속도 잔차에는 DVL 측정값(`v1`, `v2`)이 직접 들어가며, 중력항은 `R_b0w`(=`VertexGDir`)를 통해 월드 프레임 중력을 IMU 바디 프레임으로 변환한다.

### DVL+IMU 초기화 Edge

| 클래스 | 차원 | Vertex 수 | 역할 |
|---|---|---|---|
| `EdgeDvlIMUInitWithoutBias` | 3 | 9개 | 속도 잔차만 (e_V). 초기화 1단계용 |
| `EdgeDvlIMUInit` | 3 | 9개 | 속도 잔차만 (e_V). 구조는 `EdgeDvlIMUInitWithoutBias`와 동일 |
| `EdgeDvlIMUGravityRefine` | 3 | 9개 | 중력 방향 재보정용. 속도 잔차만 |
| `EdgeDvlIMUGravityRefineWithBias` | 9 | 9개 | 중력 방향 재보정 + bias. 회전+속도+위치 잔차 |

### DVL 속도 직접 제약

#### `EdgeDvlVelocity` — 핵심 분석

```cpp
class EdgeDvlVelocity: public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexVelocity>
```

- **연결 Vertex**: `VertexVelocity` (1개)
- **측정값**: DVL 원시 속도 벡터 `mV` (생성자에서 주입)
- **잔차**: `_error = mV - VG->estimate()` — DVL 관측 속도와 최적화 중인 속도의 차이
- **Jacobian**: `J = I_{3x3}` (속도에 대한 항등 행렬)
- **가중치**: DVL 유효 여부(`pKFi->mbDVL`)에 따라 100 또는 0.1 적용

**사용 위치**: `Optimizer::OptimizationDVLIMU()` (L.1881, 1892) — DVL+IMU 초기화 BA에서 VV1, VV2 각각에 추가

**의미**: 최적화 변수인 속도 vertex를 DVL 속도 측정으로 직접 구속한다. IMU 적분 edge(`EdgeDvlIMU`)는 9DoF 복합 제약이어서 속도 단독 구속력이 약할 때 이 edge로 보강한다.

### EKF 연동 보조 Edge

| 클래스 | 차원 | 역할 |
|---|---|---|
| `EdgeSE3DVLPoseOnly` | 3 | EKF 포즈와 BA 포즈 간 점 위치 불일치 최소화. (3DoF 점 잔차) |
| `EdgeSE3DVLPoseOnly2` | 6 | EKF-BA 간 상대 포즈 SE3 잔차 (R, t 각 3DoF) |
| `EdgeSE3DVLBA` | 6 | EKF 상대 변환과 ORB BA 상대 포즈를 연결. `LocalDVLIMUBundleAdjustment` 내 실제 사용 |
| `EdgeDVLRefine` | 6 | DVL EKF 외부 파라미터 T_e_c 최적화용. 포즈 오차 최소화 |
| `EdgeSE3DVLIMU` | 6 | 두 `VertexPoseDvlIMU` 간 상대 포즈 제약. 루프 클로징 시 DVL 기반 상대 포즈 고정 |

**`EdgeSE3DVLBA`는 실제 BA에서 사용**되는 DVL 제약 edge임을 주목:
```cpp
// DvlGyroOptimizer.cpp L.347
EdgeSE3DVLBA *e = new EdgeSE3DVLBA(p_cur->mT_ei_ej, p_cur->mT_e_c);
e->setInformation(Matrix6d::Identity() * 50000000);  // 매우 강한 가중치
```

### Bias 랜덤워크 / Prior Edge

| 클래스 | 잔차 | 역할 |
|---|---|---|
| `EdgeGyroRW` | `bg2 - bg1` | 연속 KF 간 자이로 바이어스 변화량 제약 |
| `EdgeAccRW` | `ba2 - ba1` | 연속 KF 간 가속도계 바이어스 변화량 제약 |
| `EdgePriorAcc` | `bprior - ba` | 초기 acc bias 사전 정보 |
| `EdgePriorGyro` | `bprior - bg` | 초기 gyro bias 사전 정보 |
| `EdgePriorPoseImu` | 15DoF | 포즈+속도+bias 전체에 대한 사전 정보 (마지막 최적화 결과 재활용) |

### 기타 Edge

| 클래스 | 역할 |
|---|---|
| `Edge4DoF` | 루프 클로징 포즈그래프. 4DoF 변환 사전 측정 제약 |
| `EdgeTrajAlign` | 추정 궤적을 GT 궤적에 정렬. alignment 행렬 T_g0_e0를 최적화 변수로 사용 |

---

## 상세 분석: EdgeDVLBeamCalibration1/2 (dead code 경고)

```cpp
class EdgeDVLBeamCalibration1: public g2o::BaseUnaryEdge<4, DVLGroPreIntegration*, VertexDVLBeamOritenstion>
class EdgeDVLBeamCalibration2: public g2o::BaseUnaryEdge<4, DVLGroPreIntegration*, VertexDVLBeamOritenstion>
```

**역할**: 4빔 DVL의 빔 방향각(alpha, beta)을 캘리브레이션. 빔 번호별로 `v_beam = f(v_body, alpha_id, beta_id)` 모델 사용.

**차이점**:
- `BeamCalibration1`: 속도 기준으로 `v_dk_visual` (시각 오도메트리 속도) 사용
- `BeamCalibration2`: 속도 기준으로 `v_dk_dvl` (DVL 측정 속도) 사용

**Dead code 판정 근거**:
1. `EdgeDVLBeamCalibration1/2`는 `Optimizer::DvlBeamOptimization()` / `DvlBeamOptimization_dvl()` 에서만 인스턴스화됨
2. 이 함수들은 `DvlGyroInitOptimization4()` 내부에서만 호출됨
3. `DvlGyroInitOptimization4()`는 **어디서도 호출되지 않음** (LocalMapping.cc는 `DvlGyroInitOptimization3()`만 사용)

**추가 문제점**:
- `EdgeDvlGyroInit2::computeError()` (G2oTypes.cc L.1784–1791)에 빔 각도 ground truth값이 하드코딩:
  ```cpp
  alpha_gt << 67.5/180*M_PI, ..., ...  // 4개 모두 동일
  beta_gt  << 45/180*M_PI,  ..., ...   // 4개 모두 동일
  ```
  이 값은 특정 DVL 모델(Nortek DVL1000)의 기본 각도로 보이며, YAML 파라미터화되어 있지 않다.

---

## IMU 관련 Edge 사용 패턴 요약

| 함수 | 사용 Edge |
|---|---|
| `Optimizer::LocalBundleAdjustment()` (IMU 전) | `EdgeMono`, `EdgeStereo` |
| `DvlGyroOptimizer::LocalDVLIMUBundleAdjustment()` | `EdgeSE3DVLBA`, `EdgeSE3ProjectXYZ`, `EdgeStereoSE3ProjectXYZ` |
| `Optimizer::OptimizationDVLIMU()` (초기화) | `EdgeDvlGyroBA`, `EdgeDvlVelocity`, `EdgePriorAcc/Gyro` |
| `Optimizer::DvlIMUInitOptimization()` | `EdgeDvlIMU`, `EdgeDvlVelocity`, `EdgeDvlIMUInit` |
| `Optimizer::DvlGyroInitOptimization3()` | `EdgeDvlGyroInit3`, `VertexDVLBeamOritenstion` |
| `Optimizer::InertialOptimization()` | `EdgeInertial`, `EdgeGyroRW`, `EdgeAccRW` |

---

## VISO 통합 관점: EdgeSonarPoint 추가 시 참고 사항

소나 포인트를 BA에 추가한다면 `EdgeDvlVelocity`보다 `EdgeSE3DVLBA`의 패턴을 참고하는 것이 더 적합하다.

### EdgeDvlVelocity를 참고할 경우 (속도 제약)
```
- 소나가 속도를 측정하는 경우
- BaseUnaryEdge<3, Vector3d, VertexVelocity> 패턴 재사용
- mV 를 소나 속도 측정값으로 교체
- 단, 좌표계 변환 필요: 소나 프레임 → 카메라 프레임
```

### EdgeSonarPoint (재투영 방식)
```
- 소나 포인트를 3D 맵 포인트로 취급
- BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexPoseDvlIMU> 패턴 사용
  (EdgeMonoBA_DvlGyros 구조 참고)
- 소나의 range/bearing 모델에 따라 computeError() 정의
  e = [range_obs - range_est, bearing_obs - bearing_est]
- computeError() 내 투영: DvlImuCamPose에 sonar 외부 파라미터
  (R_sonar_c, t_sonar_c) 추가 필요
```

### 핵심 변형 포인트

1. **`DvlImuCamPose`에 소나 외부 파라미터 추가**: `R_sonar_c`, `t_sonar_c` 필드 추가 및 `DvlImuCamPose(KeyFrame*)` 생성자 수정
2. **측정 좌표계**: DVL은 속도 벡터이므로 좌표계 변환이 선형. 소나 포인트는 비선형 투영
3. **`linearizeOplus` 구현 필요 여부**: DvlGyros 계열은 모두 수치 미분(미구현) 사용. 소나 추가 시도 동일 패턴으로 시작 가능
4. **정보 행렬 가중치**: `LocalMapping.md`의 `mlamda_DVL` / `mlamda_visual`처럼 `mlamda_sonar` 추가

---

## 주의 사항

1. **Dead code 위치**:
   - `EdgeDVLBeamCalibration1/2` (G2oTypes.h L.1853, 1928): 정의·구현은 있으나 실행 경로 없음
   - `EdgeDvlGyroInit2` (L.1619): `DvlGyroInitOptimization2()`에서 호출되나, 해당 함수 자체의 실제 호출 여부 확인 필요
   - `EdgeDvlIMU::linearizeOplus()` (G2oTypes.cc L.2333–2453): 전체가 주석 처리 → 수치 미분 사용 중

2. **하드코딩된 값**:
   - `EdgeDvlGyroInit2::computeError()` (G2oTypes.cc L.1784): `alpha_gt = 67.5°`, `beta_gt = 45°`
   - `DvlBeamOptimization` 내 초기값 (Optimizer.cc L.13978): `alpha_beta = 1.5°, 1.0°` 반복

3. **G2O_REGISTER_TYPE 등록**: G2oTypes.cc 끝부분에 직렬화를 위한 타입 등록이 있다. 새 Edge 추가 시 등록 필요:
   ```cpp
   G2O_REGISTER_TYPE(EdgeSonarPoint, EdgeSonarPoint)
   ```

4. **`VertexDVLBeamOritenstion` 오타**: 클래스명에 "Oritenstion" (Orientation+Tention 오타). 코드 전반에서 일관되게 사용 중이므로 수정 시 일괄 치환 필요.

5. **`EdgeDvlIMU`의 `linearizeOplus` 미구현**: 수치 미분 사용으로 인해 최적화 속도가 느려질 수 있다. 해석적 Jacobian은 주석으로만 남아 있음 (G2oTypes.cc L.2333).
