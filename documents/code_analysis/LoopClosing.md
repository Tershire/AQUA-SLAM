# LoopClosing.cc 분석

**파일 위치**: `src/LoopClosing.cc`  
**전체 라인**: 2732줄  
**역할**: 루프 폐쇄 및 서브맵 병합 스레드. LocalMapping이 넘겨준 KF에 대해 DBoW2 기반 장소 인식을 수행하고, 루프 감지 시 포즈 그래프 최적화, 서브맵 감지 시 맵 병합을 실행한다.

---

## 메인 루프

### `Run()`

LoopClosing 스레드의 진입점. `usleep(5000)`으로 ~200Hz 루프를 돌지만, KF 큐가 비어 있으면 즉시 통과한다.

```
while(1):
  CheckNewKeyFrames()
    → NewDetectCommonRegions()        ← DBoW2 검색 + Sim3 검증
        ├─ mbMergeDetected == true    → MergeLocal() or MergeLocal2()
        └─ mbLoopDetected  == true    → CorrectLoop()
  ResetIfRequested()
  CheckFinish() → break
  usleep(5000)
```

루프 감지와 맵 병합이 동시에 감지된 경우, 병합을 먼저 수행하고 루프 변수를 리셋한다.

---

## 함수 목록

### KF 큐 관리

| 함수 | 역할 |
|---|---|
| `InsertKeyFrame(pKF)` | LocalMapping이 호출. KF를 `mlpLoopKeyFrameQueue`에 넣음 (KF ID=0 제외) |
| `CheckNewKeyFrames()` | 큐가 비어 있지 않은지 확인 |

---

## DBoW2 기반 장소 인식

### `NewDetectCommonRegions()`

매 KF마다 호출되는 장소 인식 메인 함수. 두 가지 경우를 동시에 탐지한다.

- **루프 폐쇄**: 현재 KF가 동일 맵 내 과거 위치를 재방문
- **맵 병합**: 현재 KF가 다른 서브맵(비활성 맵)의 위치에 대응

처리 흐름:

```
1. 조기 종료 조건 체크
   - mpTracker->mDetectLoop == false
   - IMU 초기화 미완료 (inertial 모드)
   - 맵 내 KF 수 부족 (< mKFThresholdForMap+10)

2. 이전 프레임에서 이어진 후보가 있으면 (mnLoop/MergeNumCoincidences > 0)
   → DetectAndReffineSim3FromLastKF()  ← 연속성 검증

3. 새 후보 탐색 (BoW DB 검색)
   → mpKeyFrameDB->DetectNBestCandidates()
   → DetectCommonRegionsFromBoW()  ← Sim3Solver RANSAC + 투영 매칭

4. mnLoop/MergeNumCoincidences >= 3 에서 mbLoopDetected / mbMergeDetected = true
```

연속적으로 3개 이상의 KF에서 같은 후보와 기하학적 검증이 통과해야 최종 감지로 확정한다.

### `DetectCommonRegionsFromBoW()`

BoW 후보 KF들에 대해 기하학적 검증을 수행한다.

```
각 후보 KF에 대해:
  1. covisibility 이웃 KF들과 BoW 매칭 (matcherBoW.SearchByBoW)
     임계값: nBoWMatches=20 이상

  2. Sim3Solver RANSAC 실행
     - SetRansacParameters(0.99, 15 inliers, 300 iterations)
     - iterate()로 수렴 시까지 반복

  3. Sim3 수렴 시 → 이웃 KF 포함 확장 투영 매칭
     - FindMatchesByProjection() 으로 nProjMatches >= 20 검증

  4. 성공 시 → 가장 많은 매칭을 얻은 후보를 반환
     (mnLoopNumCoincidences / mnMergeNumCoincidences 카운터 증가)
```

---

## Sim3Solver: 7-DoF 변환 추정

`Sim3Solver`는 `src/Sim3Solver.cc`에 구현되어 있으며, LoopClosing에서 다음 방식으로 사용한다.

```cpp
Sim3Solver solver = Sim3Solver(mpCurrentKF, pMostBoWMatchesKF,
                                vpMatchedPoints, bFixedScale,
                                vpKeyFrameMatchedMP);
solver.SetRansacParameters(0.99, nBoWInliers, 300);

bool bConverge = false;
while(!bConverge && !bNoMore)
    mTcm = solver.iterate(20, bNoMore, vbInliers, nInliers, bConverge);

// 수렴 후
g2o::Sim3 gScm(solver.GetEstimatedRotation(),
               solver.GetEstimatedTranslation(),
               solver.GetEstimatedScale());
```

- **bFixedScale**: IMU stereo 모드에서 `true` (scale=1), monocular에서 `false`
- IMU 초기화가 완료된 inertial 맵에서는 yaw 방향 오차만 허용 (roll/pitch < 0.008 rad)

수렴된 Sim3는 `Optimizer::OptimizeSim3()`로 비선형 정제 후 투영 매칭 재검증(`FindMatchesByProjection`)까지 거친다.

---

## CorrectLoop(): 루프 폐쇄 보정

루프가 확정되면 실행. 현재 KF와 그 covisibility 이웃들의 포즈를 Sim3로 보정한다.

```
1. LocalMapping 일시 정지 요청 (RequestStop)
2. 현재 GBA 실행 중이면 중단

3. Sim3를 이용해 인접 KF들의 포즈 보정 (CorrectedSim3 계산)
   - 현재 KF의 Sim3를 기준으로 이웃 KF들에 전파

4. 보정된 포즈로 MapPoint 위치 재계산

5. 루프 매칭 MapPoint 융합 (중복 제거)
   - mvpLoopMatchedMPs에서 현재 KF의 MP를 replace

6. SearchAndFuse(CorrectedSim3, mvpLoopMapPoints)
   → 루프 KF 이웃의 MP를 현재 KF 이웃에 투영하여 추가 융합

7. 새 covisibility 엣지 구성 (LoopConnections)

8. OptimizeEssentialGraph() 호출
   - inertial 맵: OptimizeEssentialGraph4DoF (yaw + t 만)
   - 그 외: OptimizeEssentialGraph (7DoF Sim3)

9. loop edge 추가 (AddLoopEdge)
10. LocalMapping 재개
```

> 참고: 루프 감지 후 GBA(`RunGlobalBundleAdjustment`)는 현재 **주석 처리**되어 있어 실행되지 않는다 (L.1342–1349).

---

## MergeMaps(): 서브맵 병합

### `MergeLocal()` — 비관성(visual-only) 또는 혼합 맵 병합

```
1. LocalMapping 정지
2. pCurrentMap (active) ↔ pMergeMap (inactive) 식별

3. 병합 영역(welding area) KF 수집:
   - 현재 맵: mpCurrentKF + covisibility 이웃 최대 100개
   - 병합 맵: mpMergeMatchedKF + covisibility 이웃

4. Sim3 보정: spLocalWindowKFs 각 KF에 mg2oMergeScw 전파
   - MapPoint 위치 사전 보정 (mPosMerge / mNormalVectorMerge에 저장)

5. [임계 구간] 락 획득 후:
   - spLocalWindowKFs → pMergeMap으로 이동 (UpdateMap/AddKF/EraseKF)
   - spLocalWindowMPs → pMergeMap으로 이동
   - mpAtlas->ChangeMap(pCurrentMap, pMergeMap)  ← 활성 맵 전환

6. Essential graph 재구성 (spanning tree parent 역전)

7. SearchAndFuse(vCorrectedSim3, vpCheckFuseMapPoint)  ← 중복 MP 융합

8. 잔여 KF/MP → pMergeMap으로 이동, pCurrentMap을 bad 처리

9. GlobalVAPoseGraphOptimization()  ← 병합 후 전역 포즈 그래프 최적화

10. mpAtlas->RemoveBadMaps()
```

### `MergeLocal2()` — IMU 초기화 완료 맵 병합

관성 맵 전용. `MergeLocal()`과 달리 현재 활성 맵에 비활성 맵을 흡수하는 방식.

```
1. ApplyScaledRotation()으로 활성 맵 전체에 Sim3(mSold_new) 적용
2. pMergeMap의 KF/MP를 pCurrentMap으로 이동 (반대 방향)
3. Essential graph 재구성
4. SearchAndFuse() + UpdateConnections()
5. Optimizer::MergeInertialBA()  ← tight-coupled BA로 병합 영역 정제
```

---

## OptimizeEssentialGraph(): 포즈 그래프 전역 최적화

`Optimizer::OptimizeEssentialGraph()` / `OptimizeEssentialGraph4DoF()`  
(`src/Optimizer.cc`에 구현)

LoopClosing에서 호출하는 시점:

| 호출 위치 | 함수 | 설명 |
|---|---|---|
| `CorrectLoop()` | `OptimizeEssentialGraph` or `OptimizeEssentialGraph4DoF` | 루프 폐쇄 후 포즈 그래프 정제 |
| `MergeLocal()` | `GlobalVAPoseGraphOptimization` | 병합 후 전체 그래프 최적화 |
| `MergeLocal2()` | `MergeInertialBA` | 관성 맵 병합 후 tight BA |

- **inertial 맵 조건** (`IsInertial() && isImuInitialized()`): 4DoF 버전 사용 → yaw + translation만 최적화 (roll/pitch는 IMU로 고정)
- **그 외**: 7DoF Sim3 기반 Essential Graph 최적화

---

## Atlas와의 관계

```
LocalMapping ──InsertKeyFrame()──► LoopClosing (이 파일)
                                        │
                                        ├─ mpAtlas->GetCurrentMap()
                                        ├─ mpAtlas->GetAllMaps()
                                        ├─ mpAtlas->ChangeMap()        ← 병합 시 활성 맵 전환
                                        ├─ mpAtlas->SetMapBad()        ← 빈 맵 제거
                                        ├─ mpAtlas->RemoveBadMaps()
                                        └─ mpAtlas->InformNewBigChange()  ← 루프 폐쇄 후
```

- LoopClosing은 Atlas에서 모든 맵 / 현재 맵을 직접 조회하고 수정한다.
- 병합 결과는 Atlas의 맵 목록을 변경하므로, RosHandling::PublishIntegration()이 다음 폴링 주기에 병합된 맵을 포함해 발행한다.
- `mpRosHandler->PublishImgMergeCandidate()`는 NewDetectCommonRegions() 내에서 직접 호출해 병합 후보 이미지를 발행한다.

---

## 생성자 추가 사항 (AQUA-SLAM)

```cpp
LoopClosing::LoopClosing(..., RosHandling* pRosHandler,
                          int mergingThreshold,
                          rclcpp::Node::SharedPtr node)
```

원본 ORB-SLAM3 대비 추가된 인자:
- `pRosHandler`: 병합 후보 이미지 발행용
- `mergingThreshold`: `mnCovisibilityConsistencyTh` 초기값 (기본값 3)
- `node`: ROS2 노드 핸들 (image_transport, RCLCPP_DEBUG 로그용)

루프 전용 이미지 토픽도 생성자에서 광고:
- `/aqua_slam/loop/cur_img` — 현재 KF 이미지
- `AQUA_SLAM/loop/map_img` — 맵 내 매칭 KF 이미지

---

## 주의 사항

- **GBA 비활성화**: `CorrectLoop()` 끝의 `RunGlobalBundleAdjustment` 실행 코드가 주석 처리되어 있어 루프 폐쇄 후 전역 BA가 수행되지 않는다.
- **mTargetMapID 필터**: `SetTargetMap(ID)`로 병합 대상 맵을 제한할 수 있다. `-1`이면 모든 맵 허용.
- **IMU scale 검증**: inertial 맵에서 병합 시 Sim3 scale이 [0.90, 1.10] 범위를 벗어나면 병합 중단.
- `MergeLocal2()` 내 `if (numKFnew < 10)` 조건으로 너무 적은 KF의 맵은 병합 최적화를 건너뜀.
