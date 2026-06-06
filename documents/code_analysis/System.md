# System.cc 분석

**파일 위치**: `src/System.cc`  
**전체 라인**: ~1350줄  
**역할**: SLAM 파이프라인 전체 초기화 및 외부 진입점. 모든 스레드와 서브시스템 객체를 생성·연결하고, 노드로부터 프레임을 받아 Tracking에 전달한다.

---

## 생성자 초기화 순서 (`System::System`)

```
1. ORB Vocabulary 로드 (loadFromTextFile)
2. KeyFrameDatabase 생성
3. Atlas 생성  ← mpAtlas = new Atlas(0)
4. Atlas::SetInertialSensor()  (IMU_STEREO / IMU_MONOCULAR 모드일 때)
5. FrameDrawer 생성
6. DenseMapper 생성 + thread 실행
7. RosHandling 생성  ← mRosHandler
8. Tracking 생성    ← mpTracker  (메인 스레드에서 실행)
9. LocalMapping 생성 + thread 실행  ← mptLocalMapping
10. LoopClosing 생성 + thread 실행  ← mptLoopClosing
11. RosHandling::Run thread 실행    ← viewer thread
12. 스레드 간 포인터 상호 등록
```

> **주의**: `mpLocalMapper`는 생성 순서상 `mRosHandler` 생성 **이후**에 만들어지므로  
> `mRosHandler->setLocalMapping(mpLocalMapper)` 가 생성자 말미(L.250)에 별도로 호출된다.

---

## 스레드 목록

| 스레드 변수 | 실행 함수 | 역할 |
|---|---|---|
| `mptLocalMapping` | `LocalMapping::Run()` | KF 단위 MapPoint 관리 + BA |
| `mptLoopClosing` | `LoopClosing::Run()` | 루프 감지 + Map Merging |
| `dense_mapping` (로컬 변수) | `DenseMapper::Run()` | 밀집 점군 생성 |
| `viewer` (로컬 변수) | `RosHandling::Run(mpAtlas)` | ROS 토픽 발행 |

> `mptViewer` 멤버가 선언되어 있으나 Pangolin Viewer 코드는 전부 주석 처리됨.

---

## 센서 모드

```cpp
enum eSensor {
    MONOCULAR   = 0,
    STEREO      = 1,
    RGBD        = 2,
    IMU_MONOCULAR = 3,
    IMU_STEREO  = 4,
    DVL_STEREO  = 5   // ← AQUA-SLAM 전용 모드
};
```

`node.cpp`에서 항상 `DVL_STEREO=5`로 생성한다.

---

## `TrackStereoGroDVL()` — Tracking 진입점

두 가지 오버로드가 있다.

### 오버로드 1 (L.306) — `vector<IMU::ImuPoint>` 버전

IMU 전용 포인트 타입을 받는 구버전. 현재 `node.cpp`에서는 사용하지 않는다.

```cpp
cv::Mat TrackStereoGroDVL(
    const cv::Mat &imLeft, const cv::Mat &imRight,
    const double &timestamp,
    const vector<IMU::ImuPoint> &vImuMeas,
    bool bDVL, string filename)
```

내부에서:
1. 모드 변경 체크 (LocalizationMode on/off)
2. 리셋 체크
3. `mpTracker->GrabImuData(imuPoint)` 반복
4. `mpTracker->GrabImageStereoDvl()` 호출

### 오버로드 2 (L.372) — `vector<IMU::GyroDvlPoint>` 버전 ← 현재 사용

`node.cpp`의 `SyncWithImu()`가 호출하는 버전.

```cpp
cv::Mat TrackStereoGroDVL(
    const cv::Mat &imLeft, const cv::Mat &imRight,
    const double &timestamp,
    const vector<IMU::GyroDvlPoint> &vDVLGyroMeas,
    bool bDVL, string filename)
```

내부 처리 순서:
```
1. 모드 변경 체크 (LocalizationMode)
2. 리셋 체크 (mbReset / mbResetActiveMap)
3. vDVLGyroMeas 순회:
   └── mpTracker->GrabDVLGyroData(point)  — 전 포인트
   └── angular_v != 0 이면 추가로
       mpTracker->GrabImuData(ImuPoint)   — 표준 IMU preintegration 경로
4. mpTracker->GrabImageStereoDvlgyro()   ← try-catch로 감쌈
   └── 예외 발생 시 Tcw = identity 반환
5. mTrackingState, mTrackedMapPoints, mTrackedKeyPointsUn 업데이트
6. Tcw 반환
```

IMU/DVL 포인트 구분 기준 (L.425):
```cpp
if (m.angular_v.x != 0 || m.angular_v.y != 0 || m.angular_v.z != 0)
    mpTracker->GrabImuData(...);   // angular_v가 있으면 IMU 포인트
// 나머지는 DVL 포인트 (vx,vy,vz 필드만 유효)
```

---

## 주요 멤버 포인터

| 멤버 | 타입 | 역할 |
|---|---|---|
| `mpVocabulary` | `ORBVocabulary*` | BoW 어휘 — KF 루프 감지에 사용 |
| `mpKeyFrameDatabase` | `KeyFrameDatabase*` | KF BoW 인덱스 |
| `mpAtlas` | `Atlas*` | 지도 전체 (KF, MapPoint, Map) |
| `mpTracker` | `Tracking*` | 프레임 단위 추적 |
| `mpLocalMapper` | `LocalMapping*` | KF 단위 BA 스레드 |
| `mpLoopCloser` | `LoopClosing*` | 루프 감지·보정 스레드 |
| `mpFrameDrawer` | `FrameDrawer*` | 현재 프레임 시각화 데이터 |
| `mpDenseMapper` | `DenseMapper*` | 밀집 점군 매핑 스레드 |
| `mRosHandler` | `RosHandling*` | ROS 토픽 발행 스레드 |
| `mp_node` | `rclcpp::Node::SharedPtr` | ROS2 노드 핸들 (구독자/발행자 생성용) |
| `mptLocalMapping` | `std::thread*` | LocalMapping 스레드 핸들 |
| `mptLoopClosing` | `std::thread*` | LoopClosing 스레드 핸들 |
| `mSensor` | `eSensor` | 센서 모드 (`DVL_STEREO=5`) |
| `mDVL_updated` | `bool` | DVL 콜백 수신 플래그 |

---

## 보조 함수

| 함수 | 역할 |
|---|---|
| `dvlCallBack()` | DVL 수신 시 `mDVL_updated = true` 세팅 (락 보호). 현재 node.cpp에서는 직접 사용 안 함 |
| `ActivateLocalizationMode()` | `mbActivateLocalizationMode = true`. LocalMapping 중단·추적 전용 전환 |
| `DeactivateLocalizationMode()` | `mbDeactivateLocalizationMode = true`. 지도 업데이트 재개 |
| `MapChanged()` | Atlas의 big change index로 지도 변경 여부 확인 |
| `Reset()` / `ResetActiveMap()` | 전체 리셋 / 현재 맵만 리셋 |
| `Shutdown()` | 모든 스레드 종료 요청 + join |
| `GetTrackingState()` | 현재 추적 상태 (OK / LOST 등) 반환 |
| `isLost()` | 추적 손실 여부 확인 |
| `SaveTrajectoryTUM()` / `SaveTrajectoryEuRoC()` | 궤적 저장 |
| `SaveKeyFrameTrajectory()` | KF 궤적 저장 |
| `TrackStereoGroDVLKLT()` | KLT optical-flow 기반 추적 변형 (실험적) |

---

## 스레드 간 관계

```
node.cpp / SyncWithImu()
    │
    └── System::TrackStereoGroDVL()  (호출 스레드에서 동기 실행)
              │
              ├── Tracking::GrabDVLGyroData()
              ├── Tracking::GrabImuData()
              └── Tracking::GrabImageStereoDvlgyro()
                        │
                        ├── InsertKeyFrame() ──► LocalMapping::Run()  [별도 thread]
                        └── (KF 전달)        ──► LoopClosing::Run()   [별도 thread]

RosHandling::Run(mpAtlas)  [별도 thread]
    └── Atlas polling → ROS 토픽 발행
```

---

## 주의 사항

- `bLoadMap` 플래그가 하드코딩 `false`로 되어 있어 Atlas 로드 기능이 비활성화됨 (L.125)
- Pangolin Viewer 전체 주석 처리 — `mpViewer`, `mpMapDrawer` 사용 불가
- `TrackStereoGroDVLKLT()` (L.1106)는 별도 KLT 기반 변형이지만 `node.cpp`에서는 호출하지 않음
- `dense_mapping`, `viewer` 스레드는 로컬 변수로 생성되어 `join()`/`detach()` 없이 소멸 — 스레드 생존은 객체 내부 루프가 보장
