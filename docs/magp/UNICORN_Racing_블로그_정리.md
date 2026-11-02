# UNICORN Racing(UNIST HMCL) 블로그 정리 — MAGP 2026 적용 관점

> 출처: https://hmcl-unist.github.io/unicorn-racing/ — 한국어 원문 81편 전부 읽음(영어 번역본 81편은 중복이라 제외). 2026-11-02 기준.
> 팀 소개: UNIST 권철현 교수 HMC Lab의 RoboRacer(구 F1TENTH) 팀. 2025 ICRA RoboRacer AGP 2위, 2025 ICCAS Korea 우승, 2023 ICCAS 우승. 스택 공개: https://github.com/HMCL-UNIST/unicorn-racing-stack (ROS 2 Jazzy, Jetson Orin 검증).

---

# PART A. 우리 차에 바로 적용할 기술 (우선순위 순)

## A-1. Localization — 핵심 결론
UNICORN의 선택과 이유를 그대로 가져오면 됩니다.

| 항목 | UNICORN의 결정 | 우리 적용 |
|---|---|---|
| 메인 | **Cartographer pure localization** (.pbstream) | 동일. 어제 세운 방향과 일치 |
| 백업 | **Particle Filter** (mit-racecar PF + range_libc) | 동일. 같은 맵(png/yaml)으로 바로 전환 가능하게 launch 인자로 분기 |
| Cartographer 입력 | LiDAR + IMU 만 그래프 최적화에 사용, **휠 odom은 속도 예측(extrapolation)에만** | `use_odometry = false` + launch remap으로 `/vesc/odom`을 velocity reference로만 |
| 출력 | `/tracked_pose` → EKF → `/car_state/odom` | 우리 `/state` 토픽(Motive 대체) = EKF 출력으로 통일 |
| 초기 위치 | RViz 2D Pose Estimate → `/finish_trajectory` + `/start_trajectory(use_initial_pose)` 재시작 노드 | 그대로 가져오기(set_pose_v2_node). 현장에서 위치 잃었을 때 복구 수단 |

**Cartographer를 PF보다 택한 근거(영상으로 비교):**
- 정지 상태에서 PF는 `position.x`가 진동, Cartographer는 안정 → MPC에 유리.
- 충돌로 벽이 밀리거나 시간이 지나 맵이 달라져도 Cartographer는 최신 submap 3개로 짧은 매핑을 이어가므로 버팀. PF는 "현재 스캔 vs 고정 맵"만 봐서 취약. → **덕트 벽은 충돌하면 쉽게 밀리므로** 이 장점이 MAGP에서 특히 큼.
- 단, 파라미터가 매우 많아 현장 대응이 어렵다 → PF를 반드시 백업으로.

**localization.lua 튜닝값 (UNICORN 공개값, 그대로 시작점으로)**
```lua
TRAJECTORY_BUILDER.pure_localization_trimmer = { max_submaps_to_keep = 3 }
POSE_GRAPH.optimize_every_n_nodes = 2          -- 매핑은 20. 자주·작게 보정 → 위치 점프 작아짐
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 0.1
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(20.)
POSE_GRAPH.global_sampling_ratio = 0.0         -- 초기위치 필수이므로 global localization 끔
options.use_odometry = false                   -- odom을 PGO에서 제외(아래 참고)
```
- 어제 걱정한 "최적화 결과 반영 시 위치가 툭 튀는 문제"에 대한 UNICORN의 답이 `optimize_every_n_nodes = 2`입니다. 보정을 자주 하면 한 번의 점프가 작아집니다.
- 추가로 UNICORN은 `/base_link_pose_with_cov`(PoseWithCovarianceStamped, pos 0.001 m², ori 0.0001 rad²)를 발행하도록 node.cc를 수정해 EKF에 넣습니다. 우리는 `robot_localization` EKF에 `/tracked_pose`를 넣을 때 covariance를 파라미터로 주면 됩니다.

**TF 함정 (반드시 확인)**
- Cartographer `tracking_frame`은 IMU 프레임과 같거나 **회전만** 달라야 함.
- IMU가 뒤집혀/90° 틀어져 장착된 경우 IMU 프레임을 그대로 tracking_frame으로 쓰면 중력 초기화에서 **맵 전체가 뒤집힘**.
- 해법: `base_link → vesc_imu_rot`(이동만) → `vesc_imu`(회전만) 두 단계로 나누고 `tracking_frame = "vesc_imu_rot"`.
- `base_link` = 뒷축 중심, 지면(z=0). `base_link → laser`는 이동만, 회전은 0으로 두고 LiDAR 수평은 **하드웨어로** 맞춤.
- VESC 내장 IMU는 PCB 방향 때문에 보통 Z축 ±90° 틀어져 있음(rpy 표: a +90°, b 0°, c 180°, d −90°).

**긴 직선에서 odom 선속도가 필수**
- 10m급 LiDAR로 20m 직선을 달리면 LiDAR+IMU만으로는 차가 **멈춘 것으로 오판**해 정면 벽이 다가오는 것처럼 맵이 찌그러짐(IMU는 roll/pitch를 몰라 중력 분리가 안 되므로 가속도 적분 불가). Odom 선속도를 넣어야 전진을 인식.
- MAGP 코스 20~30m × 10m → 긴 직선 구간이 있을 수 있으므로 VESC odom이 **정확해야** 함 → 아래 ERPM/Adaptive odom.

**Adaptive Wheel Odometry (저마찰 판 구간 대응)**
- 가설: 가속 시 odom 과대, 제동 시 과소, 슬립 ∝ a_x, 정속에서도 일정 슬립.
- `Adaptive_gain = Theoretical_gain + σ·a_x + δ` (σ slip rate, δ offset — 노면에 따라 튜닝)
- 구현: IMU a_x를 LPF → gain 계산 → `speed = (-erpm + offset)/gain`, |v|<0.05 데드존. (adaptive_vesc_to_odom.cpp)
- MAGP 3회 코스에 **매우 미끄러운 판 2곳**이 있었으므로 그대로 쓸 가치 있음. 최소한 σ, δ 두 파라미터를 rqt로 현장 조정 가능하게.

**ERPM gain 캘리브레이션 (odom 정확도의 출발점)**
- `gain = 60·pole_pairs·gear_ratio / (2π·r)`. Velineon 3500(4극→pole_pairs 2), Traxxas 기어비 예 (83/12)×(37/13)≈19.69, r 0.05 → ≈7520.
- 실험 검증: 슬로모 카메라로 바퀴 1회전 시간 측정(추천) 또는 5m 직진 후 `new = old × odom/actual`.
- **Motive가 있으니 우리는 더 쉽게**: Motive 속도 vs `/vesc/odom` 속도를 bag으로 비교해 gain·σ·δ를 한 번에 맞출 수 있음. 이게 UNICORN에도 없는 우리 이점.

**PF(백업) 튜닝 포인트**
- 모션 노이즈 `MOTION_DISPERSION_X/Y/THETA`: 너무 작으면 코너 슬립 때 GT가 분포 밖으로 나가 발산, 너무 크면 출력 노이즈. 베이스 odom이 정확할수록 작게 둘 수 있음.
- 측정모델은 raycasting(range_libc). Jetson이면 CUDA 빌드, 아니면 CDDT/LUT.
- UNICORN 결론: "고속에서 pose 불안정" → 그래서 Cartographer로 감.

**매핑 노하우**
- 트랙 최소 2바퀴, 센터라인 따라, 급조향 금지, drift 많으면 구불구불한 곳부터 시작.
- 실시간보다 **bag 녹화 → 오프라인 매핑**으로 파라미터 반복 조정 권장.
- 매핑 끝내기: RViz 2D Goal Pose 아무 곳 클릭 → pbstream/png/yaml 자동 저장.
- 2D LiDAR 함정: 마운트 틀어지면 땅/하늘 봄, 바디 강성 약하면 정지 땐 괜찮다가 주행 중 진동으로 발생.

**LiDAR 수평 확인법 (RViz Intensity)**
- Color Transformer = Intensity, **Autocompute Intensity Bounds OFF**, 고정 범위.
- 같은 재질 벽에서 한쪽은 노랑, 반대쪽은 빨강이면 기울어진 것 → 쇼크 업소버 조절 나사로 수평 맞추기. 평평한 바닥에서.

## A-2. Localization → MPC 연결 구조 (우리 계획에 맞춘 정리)
```
/scan ──┐
/imu  ──┼─ Cartographer(pure loc) ─ /tracked_pose ─┐
/vesc/odom (adaptive gain) ─ velocity ref ─────────┤
                                                   ├─ EKF(robot_localization) ─ /state (= 기존 Motive 토픽)
                                                   │        └ odom→base_link 고주파 (VESC odom + IMU)
                                                   ▼
                                         Frenet 변환(/odom_frenet: s, d, e_ψ) ─ MPC / Pure Pursuit
```
- 연구실에서는 Motive가 `/state`를 채우고, 대회장에서는 위 체인이 채움. 컨트롤러는 손대지 않음.
- MPC 레퍼런스 속도는 아래 A-4의 곡률 기반 velocity planner 출력(vx)을 그대로 사용.

## A-3. 제어 (Pure Pursuit 개선 — 안전 기본 차량용, 1~2일)
현재 Pure Pursuit에 아래만 얹어도 무충돌 연속 랩 수가 올라갑니다.
1. **속도·곡률 적응형 lookahead**: `L1 = m_l1·v − curvature_factor·κ̄·v² + q_l1`, `clip[max(t_clip_min, √2·e_lat), t_clip_max]`. 시작값 m_l1 0.47, curvature_factor 0.145, t_clip_min 0.7 (UNICORN 실차, 우리 차·트랙에 맞춰 스케일). 간단판: `l_d = clip(0.3·v, 0.6, 2.5)`.
2. **잔차 헤딩 PID**: KP 0.4~0.8, KD = KP의 5~10%, KI 0.
3. **컨트롤러 내 마찰원 속도 클램프**: `a_long = min(a_max, sqrt(a_total² − a_lat²))` 50Hz — 경로가 뭐든 마지막 안전장치.
4. **AEB**: local waypoint가 비정상적으로 멀면 정지(`AEB_thres`) — 결승 전 기능점검(장애물 앞 정지)에도 쓸 수 있음.
5. **증상표**: 직선 좌우 진동 → m_l1↑ / 코너 커팅·바깥 이탈 → m_l1↓ 또는 curvature_factor↑ / 고속 코너 진입 진동 → curvature_factor↓ / 저속 진동 → t_clip_min↑. 하지 말 것: m_l1 0.001·1.0, curvature_factor 0.9, KD 0.4.
6. **lap_analyser**: 랩타임, 평균/최대 횡오차, 벽 최소거리를 10랩 평균·표준편차로 저장 → 표준편차 큰 세팅은 빠르더라도 버림. 예선 기준(무충돌 연속 랩)과 정확히 같은 지표.

## A-4. 속도 프로파일 (웨이포인트에 vx 입히기 — MPC 레퍼런스로도 사용)
- UNICORN 철학: 정밀 동역학 모델 대신 **GGV(종·횡 가속 한계) 값 몇 개를 경험 튜닝**. 바닥 마찰이 바뀌어도(에어컨 바람, 미끄러운 판) 라인 재최적화 없이 즉시 갱신.
- `v_max(i) = sqrt(a_y,max/|κ(i)|)` → forward pass `v_{i+1} = sqrt(v_i² + 2·a_x·Δs)` → backward pass → min → moving average(window 51).
- friction circle `(a_x/a_x,max)^p + (a_y/a_y,max)^p ≤ 1`, p=`dyn_model_exp` 1.0(보수)~2.0(공격).
- UNICORN 현재값: a_x,max 5.0 / a_y,max 4.5 m/s²; global_velocity_planner.py 변수 6개 v_max 12, ax_max_motor 7, ax_max_brake 8, dyn_model_exp 1.0, a_y_max 6.5, a_x_max 9. 우리 차는 더 낮게 시작.
- TUMFTM `trajectory_planning_helpers.calc_vel_profile` 그대로 사용 가능. 섹터별 속도 스케일(`speed_scaling.yaml`, rqt `Global_limit`/`Sector_scaling`)이 "위험 코너만 느리게"의 가장 빠른 수단.

## A-5. 라인 생성 (현장 15분 파이프라인)
1. SLAM 맵(png) 이진화(threshold 10) → skeleton → 폐곡선(가장 짧은 것) → 픽셀→미터 → 방향 정렬(`reverse`) → **watershed로 좌/우 벽 분리** → d_left/d_right, psi, kappa. 덕트 벽은 occupancy가 깨끗해 잘 됨.
2. 최소곡률 IQP(α 파라미터화, TUMFTM global_racetrajectory_optimization)로 raceline. `safety_width`(차폭+여유)를 조금씩 올리며 "Minimum distance to boundaries" 확인; 트랙폭 초과 시 solver 실패.
3. mintime(CasADi+IPOPT, Pacejka)은 UNICORN도 미채택(수렴 실패 잦음, 곡률 거칠어 곡률기반 속도 튜닝과 안 맞음) → 12일 안엔 비추천.
4. waypoint 포맷 `[id, s, d, x, y, d_right, d_left, psi, kappa, vx, ax]`로 통일 → PP·velocity planner·MPC가 같은 json 공유.
5. 맵 → centerline → raceline → 속도 → 섹터까지 **명령 하나**로 묶기(UNICORN `raceline_generator.launch.xml`). 3회 대회에서 기록 못 낸 팀 다수 = 현장 적응 실패.

## A-6. Frenet·인지·결승(추월) — 시간 남으면
- Frenet 변환: s는 track_length modulo, 최근접점은 **직전 s 근방만** 탐색(얇은 덕트 벽 반대편으로 s 점프 방지). MPC의 e_y, e_ψ도 여기서.
- **Grid Filter**: 맵 erosion(k=11 → 0.25m 마진 @0.05m)으로 트랙 밖 점 제거 → 덕트 이음새로 새는 빔·벽 반사가 상대차로 오인되는 것 차단. O(1) LUT. 단 localization 흔들리면 실제 장애물을 지움.
- **ABD + Dual-Stage 클러스터링**: `D_max = r·sin(Δφ)/sin(λ−Δφ) + 3σ`(≈ a·r+b), Stage2에서 기존 클러스터 끝점과 τ 이내면 재병합. 크기 필터 ≥10점, 0.2~0.5m, ≤9m. 상대차 후방 식별 박스(12×12cm↑)가 정확히 이 크기 범위.
- **Frenet EKF tracking**: 상태 [s, v_s, d, v_d] 등속, NN 매칭, TTL, vs가 −1~8 m/s 밖이면 재초기화, 정적/동적 투표.
- **State machine**: GB_TRACK / TRAILING / OVERTAKE / RECOVERY. 추월은 `ot_sectors.yaml`로 허용 구간에서만. 컨트롤러에 빈 경로 절대 금지(캐시 무효면 글로벌 라인). MPC 솔버 실패 시 PP fallback 구조에도 동일.
- 정적 회피 스플라인: d_apex = (obs.d ± size/2) ± evasion_dist, 벽·라인 반대편 클램프, 복귀 clamp(1.5, 5.0)m, BPoly, 전 샘플 `is_point_inside` 검사.
- GP 궤적 예측은 상대 1바퀴 관측 필요 → 생략, 등속 EKF + TRAILING으로 충분.

## A-7. 하드웨어·운영 (반나절씩)
- **VESC 체인**: 펌웨어 6.05 권장(최신은 센서 인식 문제 보고) → 모터 **단독 분리 상태**에서 FOC RL→λ→Apply→Write → 센서리스(Velineon)면 PID P·D 튜닝. HFI는 출발 10회 중 1회 버벅임·과열 → 비권장.
- **Servo gain/offset**: `servo = gain·δ + offset`, 벨크랭크 비선형 때문에 이론 계산 불가 → ±0.25rad 범위에서 실험. offset≈0.5(혼 중앙 재장착), 0.2rad 저속 원주행 지름 측정 vs `R = L/tan(δ)`.
- **서보 전원**: VESC PPM 5V 직결 말고 BEC(6~7.4V, 서보 스펙 확인) + Signal만. 단 미끄러운 바닥에서 과토크 스핀 주의.
- **타이어**: 실내 매끈 바닥·15~25℃면 Super/Mega Soft, 미본딩이면 순간접착제. (MAGP 규정: 타이어 자국 남는 점착 타이어 금지 → 순정 범위 안에서.)
- **WiFi**: `net.core.rmem_max/wmem_max = 2147483647`, `wifi.powersave = 2`, Intel이면 `iwlwifi power_save=0`, `iwlmvm power_scheme=1`. ping 39→3ms. CycloneDDS xml 인터페이스 양쪽 통일. 대회장용 **자체 공유기 지참 + MAC 고정 IP**.
- **Hokuyo**: 보드 이더넷 static IP(LiDAR .10 → 보드 .20/24), urg_node `ip_address`, RViz scan QoS **Best Effort**. IP 모르면 Wireshark로 Source IP 확인.
- **배선**: VESC 전원 12AWG 실리콘 + XT90-S 안티스파크, 배터리 병렬 포트로 무정전 교체.
- **텔레메트리**: `pitwall.log("name", val)` 식 1줄 로깅 + 매 세션 MCAP 녹화(`/scan|/odom|/imu|/tracked_pose|/state`). 3인 12일 튜닝 루프 속도가 승부.
- **시뮬**: unicorn-racing-stack `race.launch.xml sim:=true` + Pitwall 패널(2D Goal Pose로 상대 스폰, Path 모드 ±0.5 m/s, LiDAR overlay)로 추월 로직 실차 전 검증.

## A-8. 스택을 통째로 가져올지에 대한 판단
- unicorn-racing-stack은 ROS 2 **Jazzy**, RoboStack(conda) 설치, Jetson Orin arm64 검증. Cartographer(수정판 설정), PF, urg_node, vesc_driver, EKF, raceline generator, state machine, PP 컨트롤러, 시뮬레이터까지 포함.
- 우리 Orin Nano가 Jazzy(JetPack 6 + conda면 가능)인지 Humble인지에 따라 갈림. **Jazzy로 맞출 수 있으면 통째로 가져와 localization·매핑·라인 생성 파이프라인은 그대로 쓰고, 컨트롤러만 우리 PP/MPC로 교체**하는 게 12일 일정에 가장 현실적. Humble이면 cartographer 설정(lua)·TF 구성·PF·velocity planner 코드만 추려서 이식.
- 라이선스는 README에 명시 없음 → 그대로 쓰되 팀 소개 자료에 출처 표기.

---

# PART B. 글별 요약 (전 81편)

## B-1. State Estimation / Mapping (10편)
- **Localization 개요** (/posts/localization-overview/): 위치 오차는 detection→Frenet→planning→control로 전파되어 "planner가 나쁜지 위치가 틀린지" 구분조차 불가. 맵 포맷이 알고리즘을 결정: Occupancy Grid→PF/AMCL, Submap(.pbstream)→Cartographer/slam_toolbox, PointCloud→ICP/NDT.
- **Localization – Cartographer vs PF** (/posts/localization-cartographer/): "기억력 3초 금붕어" 매핑. 정지 시 PF 진동/Carto 안정, 맵 변화에 Carto 강건. 상한은 Carto, 백업은 PF.
- **Cartographer Pure Localization 가이드** (/posts/cartographer-localization/): trimmer 3, PGO 2 nodes, 초기위치 필수. 수정 ① PoseWithCovariance 발행 ② Adaptive odom ③ RViz 초기 pose 재시작 노드. localization.lua 값 공개(위 A-1).
- **Cartographer Mapping 가이드** (/posts/cartographer-mapping/): Local SLAM(extrapolation→Ceres scan match→submap) + Global SLAM(branch-and-bound loop closure, PGO). **odom을 PGO에서 제외**. 2바퀴·센터라인·급조향 금지.
- **Adaptive Wheel Odometry** (/posts/adaptive-wheel-odometry/): gain = 이론값 + σ·a_x + δ, IMU a_x LPF.
- **Occupancy Grid Mapping 노하우** (/posts/occupancy-grid-mapping/): LiDAR only/+IMU/+Odom 비교(max_range 3m 재현). 긴 직선엔 odom 필수. bag 오프라인 매핑.
- **PF – 모션모델·리샘플링** (/posts/pf-motion-model/): VESC odom control input + 가우시안 노이즈 필수, 노이즈 과소→발산/과대→노이즈. 매 프레임 multinomial 리샘플.
- **PF – 측정모델** (/posts/pf-measurement-model/): raycasting O(M×N), GPU/LUT(range_libc: Bresenham, Ray Marching, CDDT, GLT). AMCL likelihood field는 O(1)이나 동적 장애물 취약. 고속에서 불안정 → Carto.
- **TF Configuration** (/posts/tf-configuration/): base_link 뒷축·지면, laser 이동만, IMU는 `_rot` 중간 프레임, tracking_frame 제약.
- **base_link 기준 센서 TF** (/posts/base-link-sensor-tf/): SRX1 예시 수치(LiDAR 0.27/0/0.127, VESC IMU 0.10/0/0.127 Z−90°, Microstrain 0.085/0.06/0.065 pitch 180°), quaternion 표, static_transforms.launch, CAR_NAME 분기.

## B-2. Control (2편)
- **Pure Pursuit + 확장 모듈** (/posts/pure-pursuit/): pp_node 50Hz, κ=2l_y/l_d², adaptive lookahead(0.3·v, 0.6~2.5), understeer FF 0.010, 헤딩 PID 0.4/0/0.05, 마찰원 클램프.
- **Advanced Pure Pursuit** (/posts/advanced-pure-pursuit/): L1 guidance, m_l1 0.47 / curvature_factor 0.145 / KP 0.8 / KD 0.05 / t_clip 0.7~8.0, 튜닝 순서·증상표·실패 영상, lap_analyser.

## B-3. Planning (8편)
- **Centerline 추출** (/posts/centerline-extraction/): 이진화→skeleton→폐곡선→watershed→d_left/right, psi, kappa. `raceline_generator.launch.xml`.
- **Global Trajectory Optimization** (/posts/global-trajectory-optimization/): IQP(최소곡률, 메인)+SP(최단, 추월). α 박스제약. ggv/ax_max/b_ax_max csv. safety_width 1.0. 섹터 슬라이서. raceline_tuner.
- **Mintime** (/posts/mintime-optimization/): CasADi+IPOPT, Pacejka(B 7.4, C 1.2, E 0.85), CAR 파라미터(wheelbase 0.3302, I_z 0.04712, 267W, δ_max 0.34). 미채택.
- **Velocity Planner (곡률→속도)** (/posts/velocity-planning/): v=sqrt(a_y/κ), fwd/bwd pass, p=1.0, a_x 5.0/a_y 4.5. global_velocity_planner.py 6변수.
- **Velocity Planner 상세** (/posts/velocity-planner/): friction circle p, 공기저항(c_d 0.0136, m 3.5), 필터 51, calc_vel_profile 시그니처, 튜닝·트러블슈팅 표.
- **Frenet 좌표계** (/posts/frenet-coordinate/): (s,d) 변환식, 닫힌 트랙 modulo, 최근접점 점프 방지.
- **Static Obstacle Avoidance** (/posts/static-obstacle-avoidance/): spliner 20Hz, d_apex, evasion_dist 0.65, min_space 0.85, 복귀 1.5~5.0m, BPoly(접선 psi×0.8), is_point_inside 전수검사.
- **Lane Change Planner** (/posts/lane-change-planner/): ±0.35m 차선 사전생성, OT 섹터, lookahead 15m, GP 예측 0.5s, cosine ease 3.0/3.0m, hysteresis 10프레임.
- **State Machine** (/posts/state-machine/): 규칙 기반 (상태, 경로) 결정, GB_TRACK/TRAILING/OVERTAKE/RECOVERY/START/FTGONLY, 전이 우선순위, local waypoints 80, 빈 경로 금지. 한계: 곡률 속도엔 주행 의도 없음.

## B-4. Perception / Prediction (6편)
- **Grid Filter** (/posts/grid-filter/): is_point_inside O(1), erosion 마진 (k−1)/2·res, flip 1회, localization 의존.
- **2D LiDAR Detection & Clustering** (/posts/detection-clustering/): 각도순 시퀀스 O(n), Breakpoint→ABD→Dual-Stage.
- **Dual-Stage 클러스터링** (/posts/adaptive-breakpoint-detector/): Point-to-Point → Point-to-Cluster 재병합, Global frame 연산, 필터 10점/0.2~0.5m/9m.
- **Feature Extraction** (/posts/detection-feature-extraction/): Search-based rectangle fitting, Closeness 기준 권장.
- **Frenet 기반 Tracking** (/posts/frenet-based-tracking/): detect 10Hz, NN 매칭, EKF [s,vs,d,vd], seam wrap, vs 범위 재초기화, 정적/동적 투표, `/tracking/obstacles`.
- **GP 상대 궤적 예측** (/posts/gp-opponent-prediction/): d=f(s) GPR, half-lap→whole-lap, 0.3m 이탈 시 복귀 궤적, force_trailing.

## B-5. Environment / Tools (11편)
- **WiFi 딜레이** (/posts/wifi-latency-tuning/): 소켓 버퍼 2³¹−1, powersave 2, iwlwifi power_scheme=1. 39→8.8→3ms.
- **Pitwall Telemetry** (/posts/pitwall-telemetry/): pitwall.init/event/log, MCAP record.launch.
- **Pitwall 패널** (/posts/pitwall-panel-usage/): STATE 배너, 상대 스폰(2D Goal Pose, ±0.5 m/s), Publish Point 장애물, inject_mode overlay/merge.
- **실차 빌드** (/posts/racing-stack-build-racecar/): miniforge, `--recursive`, setup_conda_onCar.sh, cyclonedds.xml 양쪽.
- **노트북/시뮬 빌드** (/posts/racing-stack-build-simulation-remote/): setup_conda_onLaptop.sh 약 2분, `unicorn` alias.
- **Docker+VS Code** (/posts/racing-stack-build-docker-vscode/): 구 ROS1 스택(UNICORN.git, ForzaETH 기반).
- **원격 실행** (/posts/racing-stack-remote-run-guide/): ROS1 ROS_IP/MASTER_URI, ssh -tXC, docker exec.
- **ROS1 sim 테스트** (/posts/racing-stack-simulation-test/): map.png/yaml, mapping.launch, speed/ot sector, obstacle_publisher.
- **Dynamic Reconfigure 튜닝** (/posts/dynamic-reconfigure-tuning/): 같은 환경에서 rqt, speed_sector_tuner, save_params.
- **Sim 기초** (/posts/basic-simulator-usage/): race.launch sim:=true, RViz 도구, RB/a 전환.
- **MAC 고정 IP** (/posts/mac-based-static-ip/): 공유기 DHCP 예약.

## B-6. Build / Hardware (27편)
- **부품 리스트** (/posts/parts-list/): SRX8 129만/Fiesta 72만, NUC 14 Pro 157.5만, UST-10LX 230.5만/UST-30LX 400.6만, VESC 6 MKVI £185, HIGHEST B210/B200 서보 15.9만, Stradale Mega Soft, Microstrain 3DM-GX5-25 326만(VESC IMU면 불필요), Hobbywing 4268SD, XT90-S, 12→17mm 허브.
- **회로·배선** (/posts/circuit-wiring-guide/): 4S 직결 장치 XT 병렬, 서보는 BEC, 12AWG, XT30/60/90 전류표, 파워보드 PCB 계획.
- **서보 전압·BEC** (/posts/servo-voltage-bec/): PPM 5V vs 서보 7.4V, Signal만 연결.
- **Servo Parameter** (/posts/servo-parameter/): gain/offset, ±0.25rad, 원주행 실측.
- **ERPM gain** (/posts/erpm-gain-calibration/): 공식, 7520 예, 슬로모/직진 보정.
- **VESC PID** (/posts/vesc-pid-controllers/): P·D 위주, Realtime Data RPM.
- **VESC HFI** (/posts/vesc-hfi-guide/): 저속 극위상 추정, 과열·1/10 버벅임.
- **VESC Hall Sensor** (/posts/vesc-hall-sensor-guide/): SENSE 포트, Hall 탭 측정→Apply→Write.
- **VESC 펌웨어 6.05** (/posts/vesc-firmware-upgrade-guide/), **FOC 탭 먼저** (/posts/vesc-foc-tab-first/), **General 탭** (/posts/vesc-general-tab-motor-settings/), **포트 설정** (/posts/vesc-port-setup/), **Tool 기본** (/posts/vesc-tool-basic-usage/), **Tool 다운로드** (/posts/vesc-tool-download/), **Servo Output 활성화** (/posts/vesc-enable-servo-output/), **배터리·모터 연결** (/posts/vesc-battery-motor-connection/), **VESC IMU** (/posts/vesc-imu/): 펌웨어 업 후 App Settings→IMU, `vesc/sensors/imu/raw`.
- **Hokuyo 셋업** (/posts/hokuyo-lidar-setup/), **Wireshark IP** (/posts/lidar-ip-address-wireshark/), **Intensity 수평 정렬** (/posts/lidar-intensity-vehicle-leveling/).
- **타이어 종류** (/posts/tire-types-overview/), **타이어 본딩** (/posts/tire-bonding-guide/).
- **섀시·상판 체결** (/posts/chassis-top-plate-assembly/), **Upper plate 설계·4종 조합** (/posts/upper-plate-design-details/, -srx-nuc-no-vesc, -srx-nuc-vesc, -traxxas-nuc-vesc, -traxxas-orin-nx-vesc), **12→17mm 허브** (/posts/12mm-to-17mm-hex-hub-adapter/), **피니언 간격** (/posts/motor-pinion-gear-gap-adjustment/), **서보 혼은 정렬 후 결합** (/posts/servo-horn-after-alignment/), **선 정리** (/posts/wire-management-finish/), **상태 LED blink mk3** (/posts/status-led-blink-mk3/), **Unicorn Horn 제작** (/posts/unicorn-horn/).

## B-7. News (8편, 기술 내용 없음)
- 2022 ICRA F1TENTH AGP 5위(필라델피아, 20팀), 2023 ICRA 6위(런던), 2023 ICCAS Korea 우승, 2024 ICCAS 3위, 2025 ICCAS 우승(송도), 2025 ICRA RoboRacer AGP 2위(애틀랜타, 하이라이트 https://www.youtube.com/watch?v=i1krCBQKMIs), 제1회 F1TENTH Korea Championship Dynamic Driving Award(4위, 제주), 2023 F1TENTH Korea Bootcamp 운영, Rahul Mangharam 교수 UNIST 세미나.

---

# PART C. 우리 12일 일정에 반영 (11/2 → 11/13 출국)

| 날짜 | UNICORN에서 가져올 것 |
|---|---|
| 11/2~3 | LiDAR 장착 → **Intensity로 수평 확인**, TF(`base_link` 뒷축·지면, `vesc_imu_rot` 중간 프레임), Hokuyo static IP, RViz Best Effort. VESC 펌웨어 6.05·FOC·IMU 활성화. **Motive vs /vesc/odom으로 ERPM gain 보정**. |
| 11/3~5 | bag 녹화 → 오프라인 Cartographer 매핑(2바퀴, odom PGO 제외) → pure localization(lua 값 그대로) → EKF → `/state`. 같은 bag으로 **Motive 대비 RMSE·점프 횟수·지연** 측정, PF도 같이 돌려 백업 준비. RViz 초기 pose 재시작 노드 이식. |
| 11/4~8 | MPC(Motive 기반) 병렬 개발. 레퍼런스 속도는 곡률 기반 velocity planner(a_y_max 보수적) 출력. PP에는 L1 lookahead + 마찰원 클램프 + AEB 추가. |
| 11/6~7 | 자율주행 영상 촬영, 팀 소개 A4 → 11/8 Slack `#team-entry`. |
| 11/8~11 | MPC 입력을 LiDAR localization으로 교체. Adaptive odom σ,δ 현장 튜닝용 rqt 노출. 맵→라인→속도→섹터 **원커맨드 스크립트**. 5분 연속 주행, lap_analyser로 표준편차 확인. 장애물 앞 정지. |
| 11/12 | 코드 동결, 세팅 2벌(안전=sector scaling 낮게 / 공격), 자체 공유기·MAC 고정 IP·WiFi 절전 해제 확인, 짐. |
