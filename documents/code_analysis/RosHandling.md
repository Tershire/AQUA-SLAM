# RosHandling.cpp 분석

**파일 위치**: `src/RosHandling.cpp`  
**전체 라인**: 1082줄  
**역할**: AQUA-SLAM의 ROS2 인터페이스 전담 클래스. 두 가지 발행 경로를 가진다. (1) Tracking에서 직접 호출하는 고주파(~20Hz) 실시간 포즈, (2) Run() 스레드가 ~4Hz로 Atlas를 폴링해 발행하는 경로/지도.

---

## 핵심 토픽 발행 방식 비교

두 발행 경로는 방식과 의미가 근본적으로 다르다.

| 항목 | `/orb_pose` (`/orb_odom`) | `/orb_path` |
|---|---|---|
| **발행 방식** | 이벤트 (push) | polling (pull) |
| **발행 주체** | Tracking 스레드 → `PublishOrb()` 직접 호출 | RosHandling::Run() 스레드 (`sleep(250ms)`) |
| **주기** | ~20Hz (카메라 프레임 속도) | ~4Hz (고정) |
| **내용** | **현재 프레임 포즈 1개** (가장 최신) | **지금까지의 전체 KF 경로** (누적, 매번 재구성) |
| **최적화 반영** | visual-only `PoseOptimization()` 결과 | BA-refined KF 포즈 (LocalDVLIMUBundleAdjustment 결과) |
| **메시지 타입** | `PoseStamped` / `Odometry` | `nav_msgs/Path` |

> `/orb_path`는 "현재 BA 결과"가 아니라 **Atlas의 모든 KF 포즈를 매 polling마다 전부 재구성해 발행**한다. BA가 과거 KF 포즈를 소급 수정하면 다음 polling 때 자동으로 반영된다.

---

## 전체 구조

```
RosHandling
  │
  ├─ image_transport publishers  (이미지 4개)
  ├─ pose / odometry publishers  (카메라 및 body 좌표계)
  ├─ path publishers             (orb path, body path, DVL integration path)
  ├─ sparse map publisher        (PointCloud2)
  ├─ octomap publisher           (Octomap, 현재 미사용)
  ├─ marker publisher            (KF 위치 시각화)
  ├─ service servers             (4개)
  └─ TF broadcaster
```

---

## Publisher 목록

### image_transport Publishers

| 멤버 | 토픽 | 발행 내용 |
|---|---|---|
| `mp_img_l_pub` | `/aqua_slam/left/image_raw` | 왼쪽 카메라 원본 이미지 |
| `mp_img_r_pub` | `/aqua_slam/right/image_raw` | 오른쪽 카메라 원본 이미지 |
| `mp_img_info_pub` | `/aqua_slam/image/features` | ORB 특징점 오버레이 이미지 |
| `mp_img_merge_cond_pub` | `/aqua_slam/image/map_merge` | 병합 후보 KF 이미지 |

### Pose / Odometry Publishers

| 멤버 | 토픽 | 메시지 타입 | 설명 |
|---|---|---|---|
| `mp_pose_orb_pub` | `/aqua_slam/orb_pose` | `PoseStamped` | 카메라 좌표계 포즈 (world 기준) |
| `mp_odom_orb_pub` | `/aqua_slam/orb_odom` | `Odometry` | 카메라 포즈 + 선속도(Vwb) |
| `mp_odom_orb_body_pub` | `/aqua_slam/orb_odom_body` | `Odometry` | body 좌표계 포즈 + body 속도 |
| `mp_pose_orb_camera_pub` | `/aqua_slam/camera_pose` | `PoseStamped` | (현재 미사용, 주석 처리된 PublishCamera) |
| `mp_pose_integration_ref_pub` | `/aqua_slam/dvl_imu_pose_ref` | `PoseStamped` | loss ref KF DVL 위치 |
| `mp_pose_integration_cur_pub` | `/aqua_slam/dvl_imu_pose` | `PoseStamped` | 현재 DVL 위치 |

### Path Publishers

| 멤버 | 토픽 | 설명 |
|---|---|---|
| `mp_path_orb_pub` | `/aqua_slam/orb_path` | 카메라 좌표계 ORB 경로 (전체 KF) |
| `mp_path_orb_body_pub` | `/aqua_slam/orb_path_body` | body 좌표계 ORB 경로 |
| `mp_integration_path_pub` | `/aqua_slam/dvl_imu_path` | DVL+IMU 누적 적분 경로 |
| `mp_ref_integration_path_pub` | `/aqua_slam/dvl_imu_path_ref` | 트래킹 손실 구간 참조 경로 |
| `mp_path_ekf_pub` | `/aqua_slam/ekf_path` | EKF 경로 (현재 미사용) |

### Map Publishers

| 멤버 | 토픽 | 설명 |
|---|---|---|
| `mp_pointcloud_pub` | `/aqua_slam/sparse_map` | ORB sparse map (PointCloud2, QoS=100) |
| `mp_octomap_pub` | `/aqua_slam/octomap` | Octomap (현재 발행 코드 주석 처리) |
| `mp_markers_pub` | `/aqua_slam/markers` | KF 위치 구체 마커 (녹색: 정상, 자주색: poor vision) |

---

## Run() 스레드: ~4Hz Atlas 폴링

```cpp
void RosHandling::Run(Atlas* pAtlas)
{
    while(1) {
        UpdateMap(pAtlas);           // sparse map 재구성 및 발행
        PublishIntegration(pAtlas);  // 전체 KF 순회 → 경로 발행
        usleep(250000);              // 0.25초 대기 → ~4Hz
    }
}
```

- 별도 스레드에서 실행 (`System.cc`에서 `std::thread` 생성)
- `UpdateMap` → `PublishIntegration` 순서로 매 주기 실행

---

## PublishOrb(): Tracking에서 직접 호출, ~20Hz 고주파 발행

```cpp
void RosHandling::PublishOrb(const Eigen::Isometry3d &T_c0_cj_orb,
                              const Eigen::Isometry3d &T_d_c,
                              double timestamp,
                              const cv::Mat &Vwb)
```

**호출자**: `Tracking.cc` — 매 프레임 추적 완료 후 직접 호출 (~20Hz)

**처리 흐름**:

```
1. T_w_cj = mT_w_c0 * T_c0_cj_orb
   (mT_w_c0: UpdateMap에서 계산된 world←camera0 변환)

2. PoseStamped 발행 → mp_pose_orb_pub
   timestamp = sensor timestamp (double → rclcpp::Time)

3. m_path_orb에 추가 → mp_path_orb_pub 발행
   (PublishIntegration의 경로와 별도로 실시간 누적)

4. BroadcastTF(T_w_rviz, "aqua_slam", "bluerov/base_link")
   T_c_rviz 보정 적용 (Rz(90°) * Ry(-90°))

5. Odometry 발행 → mp_odom_orb_pub
   - Vwb가 있으면 twist.linear 세팅

6. mb_calib_initialized == true이면:
   T_w_bj = T_w_cj * T_imu_c^{-1} * T_body_imu^{-1}
   → body 좌표계 Odometry → mp_odom_orb_body_pub
   - body 속도: v_b = R_w_bj^T * v_w
```

> `mb_calib_initialized`는 UpdateMap()이 IMU calibration 데이터를 처음 읽을 때 세팅된다.

---

## PublishIntegration(): Atlas에서 모든 KF 순회하며 경로 발행

```cpp
void RosHandling::PublishIntegration(Atlas *pAtlas)
```

**호출자**: `Run()` 스레드 (~4Hz)

**처리 흐름**:

```
1. IMU 초기화 확인 (isImuInitialized() == false이면 return)

2. Atlas의 모든 맵 순회 (pAtlas->GetAllMaps())

3. 각 맵의 KF 체인 순회 (pKF->mNextKF 링크드 리스트)
   각 KF에 대해:

   a) DVL 적분 경로 계산:
      T_di_dj = R_g_d^{-1} * dR * R_g_d  (자이로 좌표계 → DVL)
      T_d0_dj = T_d0_dj * T_di_dj (누적)
      T_w_cj_integration = T_w_c0 * T_d_c^{-1} * T_d0_dj * T_d_c
      → m_integration_path에 추가

   b) ORB 경로 계산:
      T_w_cj_orb = T_w_c0 * T_c0_cj
      → orb KF 마커(Marker::SPHERE) 생성
         - mPoorVision=true: 자주색, false: 녹색

   c) DVL 손실 구간 참조 경로 (mpDvlPreintegrationLossRefKF 있으면):
      → m_ref_integration_path에 추가

4. 전체 KF set (all_kf) 재순회:
   → m_path_orb (camera), m_path_orb_body (body) 구성

5. 일괄 발행:
   mp_integration_path_pub, mp_path_orb_pub,
   mp_path_orb_body_pub, mp_ref_integration_path_pub,
   mp_markers_pub
```

경로는 매 호출마다 `.poses.clear()` 후 전체 재구성한다 (incremental append 방식이 아님).

---

## UpdateMap(): sparse map 재구성

```cpp
void RosHandling::UpdateMap(ORB_SLAM3::Atlas *pAtlas)
```

**처리 흐름**:

```
1. IMU 초기화 확인

2. 최초 1회: mT_w_c0 계산
   R_w_c0 = R_b0_w^{-1} * R_imu_c
   yaw 보정: body forward 방향이 world +x에 정렬되도록 Rz(-yaw_err) 적용

3. mb_calib_initialized 세팅 (mT_imu_c, mT_body_imu 저장)

4. 모든 맵의 MapPoint 순회:
   - 나쁜 MP, 관찰 수 < 5 제거
   - world 좌표 → PCL PointXYZRGB (맵마다 다른 색상)
   - pcl::transformPointCloud() → T_w_c0 적용

5. PointCloud2 변환 후 발행 → mp_pointcloud_pub
```

> Octomap 생성 코드(`mp_octree->updateNode`, `octomap_msgs::fullMapToMsg`)는 전체 주석 처리되어 있다.

---

## PublishMap(): 미구현 stub

```cpp
void RosHandling::PublishMap(ORB_SLAM3::Atlas *pAtlas, int state)
```

현재 맵 ID 목록만 수집하고 실제 발행 코드가 없다. `Run()`에서는 호출되지 않으며, `UpdateMap()`이 sparse map 발행을 담당한다.

---

## 서비스 서버

생성자에서 `mp_node->create_service()`로 등록.

| 서비스 | 토픽 | 처리 내용 |
|---|---|---|
| `SavePose` | `/aqua_slam/save` | `mp_system->SaveKeyFrameTrajectory()` 호출 |
| `LoadMap` | `/aqua_slam/load_map` | `mp_system->LoadAtlas()` 호출 |
| `CalibrateDVLGyro` | `/aqua_slam/calibrate` | `mp_LocalMapping->InitializeDvlIMU()` — 자이로 바이어스 초기화 트리거 |
| `FullBA` | `/aqua_slam/full_ba` | `mp_LocalMapping->FullBA()` 트리거 |

> CalibrateDVLGyro는 DVL-자이로 extrinsic 캘리브레이션이 아닌 바이어스 초기화 서비스다. 실제 extrinsic 온라인 캘리브레이션은 미구현 상태.

---

## mT_w_c0: world ← camera0 변환

`PublishOrb()`와 `PublishIntegration()`에서 공통으로 사용하는 좌표계 정렬 변환.

계산 위치: `UpdateMap()` 내부 (최초 1회)

```
R_w_c0 = R_b0_w^{-1} * R_imu_c
t_w_c0 = R_b0_w^{-1} * t_imu_c  (IMU 위치를 world 좌표로 회전)
yaw 보정: body 초기 진행 방향이 world +x에 오도록 Rz(-yaw_err) 적용
```

`R_b0_w`는 `pAtlas->getRGravity()`에서 가져오는 중력 정렬 회전행렬.

---

## 스레드 간 관계

```
Tracking ──PublishOrb()──────────────────► RosHandling (직접 호출, ~20Hz)

System  ──thread──► RosHandling::Run()   (~4Hz)
                         │
                         ├─ UpdateMap(pAtlas)        ← sparse map
                         └─ PublishIntegration(pAtlas) ← 모든 KF 경로

LoopClosing ──PublishImgMergeCandidate()─► RosHandling (병합 후보 이미지)
```

- `PublishOrb()`는 Tracking 스레드에서 직접 호출되므로 mutex 없이 `m_path_orb`에 접근 → Run() 스레드의 `PublishIntegration()` 내 `m_path_orb.poses.clear()`와 경합 가능성 있음
- `UpdateMap()`은 `m_mutex_map` 락으로 보호

---

## VISO 통합 관점: PublishDenseMap() 추가 위치

dense map (소나 포인트 클라우드 등)을 발행할 경우 권장 삽입 위치:

```
Run() 루프:
  UpdateMap(pAtlas)
  PublishIntegration(pAtlas)
  PublishDenseMap(pAtlas)    ← 여기에 추가
  usleep(250000)
```

또는 Update 주기를 분리해 map 발행 주기를 낮추려면:

```cpp
static int cnt = 0;
if (++cnt % 4 == 0)   // 1Hz
    PublishDenseMap(pAtlas);
```

`mp_pointcloud_pub`의 QoS 큐 크기가 100으로 설정되어 있어, dense map처럼 대용량 메시지는 별도 publisher를 추가하는 것이 적합하다.

---

## 주의 사항

- **경로 재구성 방식**: `PublishIntegration()`은 매 호출마다 경로 전체를 재구성한다. KF가 많아질수록 ~4Hz 주기에서 처리 시간이 증가함.
- **PublishOrb()의 m_path_orb 중복**: `PublishOrb()`와 `PublishIntegration()` 모두 `m_path_orb`에 데이터를 추가/재구성한다. `PublishOrb()`는 실시간 append, `PublishIntegration()`은 매번 clear 후 재구성 — 두 경로가 경합한다.
- **Octomap 비활성화**: `UpdateMap()` 내 octomap 생성 코드는 전체 주석 처리. `mp_octomap_pub`은 등록되어 있으나 발행되지 않는다.
- **생성자 초기화**: 노드 재시작 시 RViz2 stale path/cloud를 제거하기 위해 생성자에서 빈 메시지를 즉시 발행한다.
