# node.cpp 분석

**파일 위치**: `src/node.cpp`  
**전체 라인**: 263줄  
**역할**: ROS2 노드 진입점. 센서 토픽을 구독하고, 버퍼에 쌓인 데이터를 타임스탬프로 정렬해 `System::TrackStereoGroDVL()`에 전달한다.

---

## 전체 구조

```
main()
 ├── System SLAM 생성 (DVL_STEREO 모드)
 ├── ImuGrabber  imugb   ← /imu/data 구독
 ├── DVLGrabber  dvlgb   ← /bluerov2/dvl 구독
 ├── ImageGrabber igb    ← /camera/left, /camera/right 구독
 └── thread: igb.SyncWithImu()  ← 동기화 + Tracking 호출
```

---

## Grabber 클래스

### `ImuGrabber`

| 항목 | 내용 |
|---|---|
| 구독 토픽 | `ImuTopic` (YAML) — 기본값 `/imu/data` |
| 메시지 타입 | `sensor_msgs/msg/Imu` |
| 콜백 | `GrabImu()` — `unique_lock`으로 `imuBuf`에 push |
| 큐 | `queue<Imu::ConstSharedPtr> imuBuf` + `mutex mBufMutex` |

### `DVLGrabber`

| 항목 | 내용 |
|---|---|
| 구독 토픽 | `DvlTopic` (YAML) — 기본값 `/bluerov2/dvl` |
| 메시지 타입 | `nav_msgs/msg/Odometry` |
| 콜백 | `GrabDVL()` — `unique_lock`으로 `dvlBuf`에 push |
| 큐 | `queue<Odometry::SharedPtr> dvlBuf` + `mutex mBufMutex` |
| 데이터 필드 | `twist.twist.linear.{x,y,z}` 만 사용 (속도 벡터) |

### `ImageGrabber`

| 항목 | 내용 |
|---|---|
| 구독 토픽 | `LeftImgTopic`, `RightImgTopic` (YAML) |
| 메시지 타입 | `sensor_msgs/msg/Image` (image_transport `"raw"` 전송) |
| 콜백 | `GrabImageLeft()` / `GrabImageRight()` — 각 큐에 push |
| 큐 | `imgLeftBuf`, `imgRightBuf` + 각각 독립 mutex |
| 보유 포인터 | `mpSLAM`, `mpImuGb`, `mpDvlGb` |

---

## 구독자 등록 (main, L.227–253)

```cpp
// IMU — QoS depth 100
auto imu_sub = node->create_subscription<Imu>(imu_topic, 100,
    [&imugb](msg) { imugb.GrabImu(msg); });

// DVL — QoS depth 100
auto dvl_sub = node->create_subscription<Odometry>(dvl_topic, 100,
    [&dvlgb](msg) { dvlgb.GrabDVL(msg); });

// 스테레오 이미지 — image_transport "raw"
auto it   = image_transport::create_subscription(node, img_l_topic, ..., "raw");
auto it_r = image_transport::create_subscription(node, img_r_topic, ..., "raw");
```

토픽 이름은 YAML에서 읽어온다:
```cpp
cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
string imu_topic   = fsSettings["ImuTopic"];
string dvl_topic   = fsSettings["DvlTopic"];
string img_l_topic = fsSettings["LeftImgTopic"];
string img_r_topic = fsSettings["RightImgTopic"];
```

---

## `SyncWithImu()` — 핵심 동기화 함수

별도 스레드에서 1 ms 폴링 루프로 동작한다 (`thread sync_thread`, L.256).

### 처리 순서

```
while (true) {
    if (imgLeftBuf, imgRightBuf, imuBuf 모두 비어 있지 않음) {

        1. 스테레오 타임스탬프 정렬
           └── |tImLeft - tImRight| > 0.1s 이면 오래된 프레임 pop
           └── 그래도 차이 > 0.1s 이면 skip

        2. IMU 선행 확인
           └── tImLeft > imuBuf.back().stamp 이면 skip (IMU가 아직 부족)

        3. 이미지 꺼내기 (imgLeftBuf, imgRightBuf에서 pop → cv::Mat)

        4. IMU 수집 — stamp <= tImLeft 인 것 전부 pop
           └── GyroDvlPoint(ax,ay,az, wx,wy,wz, 0,0,0, …, t) 생성
           └── vGyroDVLMeas에 추가

        5. DVL 수집 — stamp <= tImLeft 인 것 전부 pop
           └── DvlPoint(vx,vy,vz, …, t) → vDVLMeas 추가
           └── GyroDvlPoint(0,0,0, 0,0,0, vx,vy,vz, …, t) → vGyroDVLMeas 추가

        6. DVL 중복 제거: vDVLMeas.size() >= 2 이면 1개로 잘라냄

        7. vGyroDVLMeas가 비어 있으면 skip

        8. 타임스탬프 기준 정렬 (sort by .t)

        9. System::TrackStereoGroDVL(imLeft, imRight, tImLeft,
                                      vGyroDVLMeas, !vDVLMeas.empty())
    }
    sleep_for(1ms)
}
```

### 동기화 조건 요약

| 조건 | 동작 |
|---|---|
| 세 큐 중 하나라도 빔 | 대기 |
| `\|tImLeft - tImRight\| > 0.1s` | 느린 쪽 pop, 그래도 초과 시 skip |
| `tImLeft > imuBuf.back().stamp` | IMU 미수신 → skip |
| `vGyroDVLMeas` 비어 있음 | IMU도 DVL도 없음 → skip |

### GyroDvlPoint 통합 방식

IMU 측정값과 DVL 측정값을 **같은 `vGyroDVLMeas` 벡터**에 섞어 넣는다.

- IMU 포인트: `(ax,ay,az, wx,wy,wz, 0,0,0, …, t)` — DVL 필드 0
- DVL 포인트: `(0,0,0, 0,0,0, vx,vy,vz, …, t)` — IMU 필드 0

벡터 전달 후 `TrackStereoGroDVL`가 `angular_v != 0` 여부로 IMU/DVL 포인트를 구분한다 (System.cc L.425–428).

---

## VISO 통합 포인트 — SonarGrabber 추가 방법

### 1. 클래스 추가 (L.39 DVLGrabber 아래)

```cpp
class SonarGrabber
{
public:
    void GrabSonar(const sensor_msgs::msg::PointCloud2::SharedPtr &msg)
    {
        unique_lock<mutex> lock(mBufMutex);
        sonarBuf.push(msg);
    }

    queue<sensor_msgs::msg::PointCloud2::SharedPtr> sonarBuf;
    mutex mBufMutex;
};
```

### 2. ImageGrabber 생성자에 포인터 추가 (L.55)

```cpp
// before
ImageGrabber(ORB_SLAM3::System *pSLAM, ImuGrabber *pImuGb, DVLGrabber *pDvlGb)

// after
ImageGrabber(ORB_SLAM3::System *pSLAM, ImuGrabber *pImuGb,
             DVLGrabber *pDvlGb, SonarGrabber *pSonarGb)
    : ..., mpSonarGb(pSonarGb) {}
```

### 3. SyncWithImu() — DVL 수집 블록 이후에 Sonar 수집 블록 삽입 (L.164 부근)

```cpp
// DVL 수집 블록 끝 (L.163) 바로 다음
{
    unique_lock<mutex> lock(mpSonarGb->mBufMutex);
    while (!mpSonarGb->sonarBuf.empty() &&
           rclcpp::Time(mpSonarGb->sonarBuf.front()->header.stamp).seconds() <= tImLeft) {
        // 소나 포인트 변환 후 별도 벡터에 추가
        vSonarMeas.push_back(...);
        mpSonarGb->sonarBuf.pop();
    }
}
```

### 4. TrackStereoGroDVL 호출 시그니처 변경 (L.176)

```cpp
// before
mpSLAM->TrackStereoGroDVL(imLeft, imRight, tImLeft, vGyroDVLMeas, !vDVLMeas.empty());

// after
mpSLAM->TrackStereoGroDVL(imLeft, imRight, tImLeft, vGyroDVLMeas, !vDVLMeas.empty(),
                           vSonarMeas, !vSonarMeas.empty());
```

### 5. main()에서 구독자 추가 (L.234 dvl_sub 아래)

```cpp
SonarGrabber sonargb;
ImageGrabber igb(&SLAM, &imugb, &dvlgb, &sonargb);

string sonar_topic = fsSettings["SonarTopic"];
auto sonar_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    sonar_topic, 10,
    [&sonargb](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        sonargb.GrabSonar(msg);
    });
```

---

## 스레드 간 관계

```
ROS callbacks (spin thread)
  ├── imu_sub  → ImuGrabber::imuBuf
  ├── dvl_sub  → DVLGrabber::dvlBuf
  ├── it       → ImageGrabber::imgLeftBuf
  └── it_r     → ImageGrabber::imgRightBuf
                         │
               SyncWithImu (별도 thread)
                         │
               System::TrackStereoGroDVL()
                         │
               Tracking::GrabImageStereoDvlgyro()
```

- `rclcpp::spin(node)` 는 메인 스레드에서 실행
- `SyncWithImu()` 는 `thread sync_thread`에서 실행
- 버퍼 접근은 모두 mutex 보호

---

## 주의 사항

- `SyncWithImu()`는 무한 루프로 `join()`이 실제로는 불리지 않는다 (`rclcpp::shutdown()` 후 `sync_thread.join()` 은 블록된다)
- DVL 데이터가 한 프레임 사이에 2개 이상 들어오면 강제로 1개로 잘라낸다 (L.166–167) — 이중 기여 방지
- 이미지가 `"raw"` 인코딩(BGR8)으로 수신된다 — Stonefish 시뮬레이터 기본값
