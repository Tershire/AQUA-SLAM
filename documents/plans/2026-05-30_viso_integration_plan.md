# VISO + AQUA-SLAM 통합 계획

**작성일**: 2026-05-30  
**참고 논문**: VISO: Robust Underwater Visual-Inertial-Sonar SLAM with Photometric Rendering for Dense 3D Reconstruction (arXiv:2601.01144v2, IEEE RA-L 2026)  
**대상 코드**: AQUA-SLAM (IEEE TRO 2025), branch `simulation/stonefish`

---

## 1. 배경 및 목표

### 두 시스템의 차이

| 항목 | AQUA-SLAM | VISO |
|---|---|---|
| 센서 | DVL + stereo camera + IMU | 3D sonar + stereo camera + IMU |
| 핵심 기여 | DVL tightly-coupled BA | 3D sonar tightly-coupled BA + dense mapping |
| 지도 유형 | Sparse ORB feature map | Dense TSDF mesh (photometric rendering) |
| Loop closure | DBoW2 visual | 없음 |
| 온라인 교정 | 미구현 (논문에만 기술) | T_CSo coarse-to-fine 구현됨 |

### 통합의 이점 (SAR AUV 미션 기준)

- DVL 속도 측정 + 3D sonar 기하 정보의 상호 보완
- 완전 암흑 환경에서 sonar만으로 시각 SLAM 수준의 localisation 유지 (VISO 실험: 조명 있음 0.201 m, 완전 암흑 0.213 m RMSE)
- Dense 3D map → 난파선 내부 경로계획 및 구조자 위치 추정
- 검증된 하드웨어: VISO 실험 플랫폼이 BlueROV2 + **WaterLinked 3D-15** + Nortek Nucleus 1000 — 동일 소나 사용 가능

---

## 2. 시스템 구조 비교

### VISO 핵심 알고리즘

#### 2.1 Online Extrinsic Calibration (T_CSo)

**Coarse** (hand-eye 스타일, VISO 식 4):
```
argmin_{T̂_CSo}  Σ ||T_{Ci-1,Ci} · T̂_CSo  −  T̂_CSo · T_{Soi-1,Soi}||²
```

**Refined** (카메라 landmark ↔ sonar point cloud 정합, VISO 식 7):
```
T_CSo = T_WCi⁻¹ · T_P2P · T_WCi · T̂_CSo
```
- T_IC (camera↔IMU)는 VINS-Mono 방식 online calibration [VISO ref. 27]

#### 2.2 3D Sonar Data Association

1. 포인트 클라우드를 voxel로 분할, PCA로 surface normal 계산
2. IMU 예측 prior `T̂_{Sok,Soi}`로 현재 프레임을 keyframe으로 변환
3. voxel correspondence 조건: 거리 `< γ`, normal 유사도 `> l`
4. Back-projected 2D-2D RANSAC으로 outlier 제거

#### 2.3 Sonar Residual (VISO 식 10)

```
E_so = T_WIk · T_ISo · P^k  −  T_WIi · T_ISo · P^i
```
- P^k, P^i: keyframe / current frame의 소나 feature point
- T_ISo: sonar frame → IMU frame 변환 (calibration 결과)

#### 2.4 Joint Optimization (VISO 식 18)

```
J(X) = Σ E_So^T P_So E_So  +  Σ E_I^T P_I E_I  +  Σ E_C^T P_C E_C
```

#### 2.5 Dense Mapping

```
[x, y, z, g]^T = T_WIi · T_ISo · P_Soi,j
g_ij = C(π_s(T_WIi · T_ISo · P_Soi,j))   ← 소나 포인트에 카메라 색상 부여
```
- 최종 dense map: vdbfusion TSDF mesh

---

## 3. 통합 계획 (단계별)

### 1단계: Sonar Localisation — Local BA에 sonar residual 추가 ⬅ 우선순위 1

**목표**: `LocalDVLBundleAdjustment()`를 확장하여 3D sonar 측정을 추가 잔차로 포함

**수정 파일**:
- `include/G2oTypes.h` — `EdgeSonarPoint` g2o edge 정의 추가
- `src/Optimizer.cc` — Local BA cost function에 sonar 항 추가
- `src/LocalMapping.cc` — sonar keyframe 데이터 전달 경로
- `src/System.cc` / `include/System.h` — 3D sonar 입력 인터페이스

**확장 cost function**:
```
E_total = λ_v · E_visual  +  λ_d · E_DVL  +  λ_r · E_rot  +  λ_s · E_sonar
```

**참고**: `EdgeDVLVelocity` (G2oTypes.h) 구조를 그대로 참고하여 `EdgeSonarPoint` 구현

---

### 2단계: 3D Sonar 입력 파이프라인 ⬅ 1단계와 병행

**목표**: node.cpp에 `SonarGrabber` 스레드 추가, synchronizer에 sonar 통합

**수정 파일**:
- `src/node.cpp` — `SonarGrabber` 콜백 + `SyncWithImu()`에 sonar 큐 추가
- `include/` — `SonarFrame` 데이터 구조체 정의
- `config/*.yaml` — sonar 관련 파라미터 (T_ISo, voxel size γ, normal threshold l)

**입력 토픽** (WaterLinked 3D-15 기준):
```
/waterlinked/sonar3d/pointcloud  [sensor_msgs/PointCloud2]
```

---

### 3단계: Online Calibration T_CSo 구현 ⬅ 우선순위 2

**목표**: VISO Section III-B의 coarse-to-fine T_CSo 교정 구현  
(AQUA-SLAM 논문 Section V에서 미구현으로 남은 T_DC 교정과 상응하는 소나 버전)

**수정 파일**:
- `src/LocalMapping.cc` — `InitializeSonarCamera()` 함수 추가
- `src/Optimizer.cc` — hand-eye calibration 최적화 (식 4) 구현
- `src/RosHandling.cpp` — 교정 서비스 확장 또는 신규 서비스 추가

**현황 주의**: 기존 `EdgeDVLBeamCalibration1/2` (G2oTypes.h:1853, Optimizer.cc:14018)는 dead code이며 DVL beam 교정용으로 T_CSo 교정과는 별개임

---

### 4단계: Dense Mapping (TSDF) ⬅ 우선순위 3

**목표**: 소나 포인트 클라우드 + 카메라 색상 렌더링 → TSDF mesh 실시간 생성

**추가 의존성**:
- [vdbfusion](https://github.com/PRBonn/vdbfusion) — TSDF integration 라이브러리 (VISO ref. 30)
- OpenVDB

**수정 파일**:
- `src/RosHandling.cpp` — `PublishDenseMap()` 추가
- 신규: `src/DenseMapper.cc` / `include/DenseMapper.h`
- 출력 토픽: `/aqua_slam/dense_map` [sensor_msgs/PointCloud2 또는 mesh_msgs/Mesh]

**현재 OctoMap과의 관계**: 병행 운용 또는 대체. SAR 미션에서는 TSDF mesh가 더 유용할 가능성 높음.

---

### 5단계: Sonar 기반 Loop Closure (연구 과제)

**현황**: 미해결 연구 문제. VISO 자체도 loop closure 미구현.  
**방향**: 소나 scan descriptor (예: M2DP, FPFH) 기반 place recognition → DBoW2 대체/보완  
**권장**: 1~4단계 안정화 후 별도 연구 과제로 진행

---

## 4. 데이터 흐름 (통합 후)

```
WaterLinked 3D-15 ──┐
Left Camera ─────────┤
Right Camera ─────────┤── SyncWithImu() ──► Tracking ──► LocalMapping
IMU ─────────────────┤                                      │
DVL ─────────────────┘                              LocalDVLSonarBA()
                                                     E = λv·Evisual
                                                       + λd·EDVL
                                                       + λr·Erot
                                                       + λs·Esonar
                                                            │
                                              ┌─────────────┴──────────────┐
                                         LoopClosing               DenseMapper
                                         (DBoW2 visual)          (TSDF mesh)
                                              │                        │
                                         RosHandling ◄────────────────┘
                                              │
                              /orb_pose, /orb_odom, /dense_map, TF
```

---

## 5. 참고 코드 위치 (AQUA-SLAM)

| 기능 | 파일 | 위치 |
|---|---|---|
| DVL g2o edge 정의 | `include/G2oTypes.h` | ~L.1400 `EdgeDVLVelocity` |
| Local BA 구현 | `src/Optimizer.cc` | `LocalDVLBundleAdjustment()` |
| IMU preintegration | `src/LocalMapping.cc` | `DVLGroPreIntegration` |
| Sensor sync | `src/node.cpp` | `SyncWithImu()` thread |
| Dead calibration code | `include/G2oTypes.h` | L.1853, L.1928 |
| ROS 출력 | `src/RosHandling.cpp` | `PublishOrb()`, `PublishMap()` |

---

## 6. 관련 문서

- [architecture.md](../architecture.md) — 전체 시스템 구조도
- [implementation_notes.md](../implementation_notes.md) — 논문-코드 갭 (online calibration 미구현 상세)
- VISO 논문: arXiv:2601.01144v2
- AQUA-SLAM 논문: IEEE TRO 2025 (AQUA-SLAM)
