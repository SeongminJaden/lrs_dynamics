# RSDS - ROS2 Satellite Dynamics Simulator

## Overview

ROS2 기반 고정밀 궤도 전파기(HPOP) + RViz2 GUI 시스템
MoveIt2 로봇팔 랑데부/도킹 시뮬레이션 확장 지원

본 프로젝트는 Gazebo 시뮬레이터에서 기본적으로 제공되는 단순 중력 모델을 대체하여, 실제 위성 환경에 보다 근접한 고정밀 지구 중력장 모델을 적용하기 위한 플러그인과 함께, 완전한 HPOP(High Precision Orbit Propagator) 시스템을 구현합니다.

---

## System Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        RSDS HPOP SYSTEM ARCHITECTURE                        │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────┐     ┌──────────────────────────────────────────────────┐ │
│  │   Celestrak  │────▶│                  hpop_tle                        │ │
│  │   TLE API    │     │  - TLE Fetcher (NORAD ID)                        │ │
│  └──────────────┘     │  - TLE Parser & Validator                        │ │
│                       └──────────────────┬───────────────────────────────┘ │
│                                          │                                  │
│                                          ▼                                  │
│  ┌───────────────────────────────────────────────────────────────────────┐ │
│  │                         hpop_core                                     │ │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────┐   │ │
│  │  │ Orbit Propagator│  │   Integrators   │  │ Coordinate Transform│   │ │
│  │  │   (6DOF HPOP)   │  │ RK4/RKF78/ABM   │  │  ECI/ECEF/RTN/LVLH │   │ │
│  │  └────────┬────────┘  └─────────────────┘  └─────────────────────┘   │ │
│  │           │                                                           │ │
│  │           ▼                                                           │ │
│  │  ┌─────────────────────────────────────────────────────────────────┐ │ │
│  │  │                    Perturbation Models                          │ │ │
│  │  │  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌───────────┐ │ │ │
│  │  │  │ GGM05C  │ │  Drag   │ │   SRP   │ │ 3rd Body│ │   Tides   │ │ │ │
│  │  │  │ Gravity │ │NRLMSISE │ │ Shadow  │ │Moon/Sun │ │ Solid/Sea │ │ │ │
│  │  │  │ (n=360) │ │         │ │         │ │         │ │           │ │ │ │
│  │  │  └─────────┘ └─────────┘ └─────────┘ └─────────┘ └───────────┘ │ │ │
│  │  └─────────────────────────────────────────────────────────────────┘ │ │
│  └───────────────────────────────────────────────────────────────────────┘ │
│                                          │                                  │
│              ┌───────────────────────────┼───────────────────────────┐     │
│              │                           │                           │     │
│              ▼                           ▼                           ▼     │
│  ┌─────────────────────┐   ┌─────────────────────┐   ┌─────────────────┐  │
│  │   hpop_analysis     │   │    hpop_maneuver    │   │   hpop_export   │  │
│  │  - Ground Contact   │   │  - Delta-V Planning │   │  - CSV Export   │  │
│  │  - Proximity/CA     │   │  - Orbit Transfer   │   │  - CCSDS OEM    │  │
│  │  - Coverage         │   │  - Station Keeping  │   │  - TLE Output   │  │
│  └─────────────────────┘   └─────────────────────┘   └─────────────────┘  │
│                                          │                                  │
│                                          ▼                                  │
│  ┌───────────────────────────────────────────────────────────────────────┐ │
│  │                      hpop_rviz_plugins                                │ │
│  │  ┌─────────────────────────────────────────────────────────────────┐ │ │
│  │  │                    HPOP Control Panel (Qt)                      │ │ │
│  │  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌───────────┐ │ │ │
│  │  │  │  Satellite  │ │  Propagation│ │  Maneuver   │ │  Contact  │ │ │ │
│  │  │  │   Table     │ │   Control   │ │   Planner   │ │   View    │ │ │ │
│  │  │  └─────────────┘ └─────────────┘ └─────────────┘ └───────────┘ │ │ │
│  │  └─────────────────────────────────────────────────────────────────┘ │ │
│  │  ┌─────────────────────────────────────────────────────────────────┐ │ │
│  │  │                    3D Visualization                             │ │ │
│  │  │  - Earth (Day/Night Texture)    - Orbit Trails                 │ │ │
│  │  │  - Satellites (3D Models)       - Ground Tracks                │ │ │
│  │  │  - Ground Stations              - Coverage Footprints          │ │ │
│  │  └─────────────────────────────────────────────────────────────────┘ │ │
│  └───────────────────────────────────────────────────────────────────────┘ │
│                                                                             │
│  ┌───────────────────────────────────────────────────────────────────────┐ │
│  │              GAZEBO INTEGRATION (Phase 6: MoveIt2 Extension)          │ │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────┐   │ │
│  │  │ Chaser Satellite│  │ Target Satellite│  │   Robot Arm         │   │ │
│  │  │   (6DOF + Arm)  │  │   (Tumbling)    │  │   (MoveIt2)         │   │ │
│  │  └────────┬────────┘  └────────┬────────┘  └──────────┬──────────┘   │ │
│  │           │                    │                      │              │ │
│  │           └────────────────────┼──────────────────────┘              │ │
│  │                                ▼                                      │ │
│  │           ┌─────────────────────────────────────────────┐            │ │
│  │           │         Rendezvous & Docking Simulation     │            │ │
│  │           │  - Relative Navigation (LVLH Frame)         │            │ │
│  │           │  - Proximity Operations                     │            │ │
│  │           │  - Capture & Berthing                       │            │ │
│  │           └─────────────────────────────────────────────┘            │ │
│  └───────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Package Structure

```
ros2_ws/src/
├── lrs_dynamics/              # GGM 중력 플러그인 (Gazebo)
│   ├── include/gazebo_leo_gravity/
│   ├── src/
│   ├── data/GGM05C.gfc
│   └── ...
│
└── hpop_system/               # HPOP 시스템 (메타패키지)
    │
    ├── hpop_msgs/             # ROS2 인터페이스 정의
    │   ├── msg/
    │   │   ├── SatelliteState.msg
    │   │   ├── TLEData.msg
    │   │   ├── OrbitalElements.msg
    │   │   ├── Acceleration.msg
    │   │   ├── ManeuverPlan.msg
    │   │   ├── GroundStation.msg
    │   │   ├── ContactWindow.msg
    │   │   ├── ProximityEvent.msg
    │   │   ├── PropagatorStatus.msg
    │   │   └── SpacecraftPose.msg
    │   ├── srv/
    │   │   ├── FetchTLE.srv
    │   │   ├── AddSatellite.srv
    │   │   ├── RemoveSatellite.srv
    │   │   ├── PlanManeuver.srv
    │   │   ├── PredictContacts.srv
    │   │   ├── AnalyzeProximity.srv
    │   │   ├── ExportEphemeris.srv
    │   │   └── SetPerturbations.srv
    │   └── action/
    │       ├── PropagateOrbit.action
    │       ├── BatchPropagate.action
    │       └── ExecuteManeuver.action
    │
    ├── hpop_core/             # 핵심 전파 라이브러리
    │   ├── include/hpop_core/
    │   │   ├── propagator.hpp
    │   │   ├── integrators/
    │   │   ├── state_vector.hpp
    │   │   ├── orbital_elements.hpp
    │   │   ├── coordinate_frames.hpp
    │   │   ├── time_system.hpp
    │   │   └── constants.hpp
    │   └── src/
    │
    ├── hpop_perturbations/    # 섭동력 모델
    ├── hpop_tle/              # TLE 처리
    ├── hpop_analysis/         # 임무 분석
    ├── hpop_maneuver/         # 기동 계획
    ├── hpop_export/           # 데이터 내보내기
    ├── hpop_rviz_plugins/     # RViz2 GUI
    ├── hpop_gazebo/           # Gazebo 통합 (MoveIt2)
    └── hpop_bringup/          # 런치 및 설정
```

---

## Coordinate Frame System (TF2)

```
                    ┌─────────┐
                    │  earth  │ (ECI J2000)
                    └────┬────┘
                         │
          ┌──────────────┼──────────────┐
          │              │              │
          ▼              ▼              ▼
    ┌──────────┐   ┌──────────┐   ┌──────────┐
    │   ecef   │   │ satellite│   │   sun    │
    │  (ITRF)  │   │   _eci   │   │          │
    └────┬─────┘   └────┬─────┘   └──────────┘
         │              │
         ▼              ▼
    ┌──────────┐   ┌──────────┐
    │ground_stn│   │  lvlh    │ (Local Vertical Local Horizontal)
    │   _ecef  │   │          │
    └──────────┘   └────┬─────┘
                        │
              ┌─────────┼─────────┐
              │         │         │
              ▼         ▼         ▼
        ┌─────────┐ ┌─────────┐ ┌─────────┐
        │ chaser  │ │ target  │ │relative │
        │  _body  │ │  _body  │ │ _state  │
        └────┬────┘ └─────────┘ └─────────┘
             │
             ▼
        ┌─────────┐
        │  arm    │ (MoveIt2 Planning Frame)
        │ _base   │
        └─────────┘
```

---

## Node Graph

```
                              ┌─────────────────┐
                              │  /celestrak_api │
                              └────────┬────────┘
                                       │ HTTP
                                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                            ROS2 Nodes                                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────┐        ┌──────────────────┐        ┌─────────────────┐    │
│  │ tle_service │───────▶│ orbit_propagator │◀───────│ perturbation    │    │
│  │   _node     │  TLE   │     _node        │ Accel  │ _server_node    │    │
│  └─────────────┘        └────────┬─────────┘        └────────┬────────┘    │
│                                  │                           │             │
│                    /hpop/states  │                           │             │
│         ┌────────────────────────┼────────────────────────┐  │             │
│         │                        │                        │  │             │
│         ▼                        ▼                        ▼  ▼             │
│  ┌─────────────┐        ┌──────────────┐        ┌─────────────────┐       │
│  │  contact    │        │  proximity   │        │     export      │       │
│  │   _node     │        │    _node     │        │     _node       │       │
│  └──────┬──────┘        └──────┬───────┘        └────────┬────────┘       │
│         │                      │                         │                │
│         │ /hpop/contacts       │ /hpop/proximity         │                │
│         │                      │                         │                │
│         └──────────────────────┼─────────────────────────┘                │
│                                │                                          │
│                                ▼                                          │
│                    ┌───────────────────────┐                              │
│                    │    hpop_rviz_panel    │◀──── /tf, /markers           │
│                    │    (RViz2 Plugin)     │                              │
│                    └───────────────────────┘                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Development Phases

### Phase 1: Core Infrastructure
- `hpop_msgs` 패키지 (모든 인터페이스 정의) ✅
- `hpop_core` 패키지 구조
- RK4/RKF78 적분기 구현
- 좌표 변환 라이브러리 (ECI↔ECEF↔Keplerian)
- `orbit_propagator_node` 기본 구현
- 기본 TLE 파서

### Phase 2: Perturbation Models
- 대기 항력 모델 (Exponential + NRLMSISE-00)
- SRP 모델 (Shadow/Penumbra)
- 태양/달 위치 계산
- 제3체 섭동 (Moon/Sun gravity)
- 고체 지구 조석

### Phase 3: TLE & Analysis
- Celestrak API 연동
- 지상국 데이터베이스
- 접촉 예측 알고리즘
- 근접/충돌 분석

### Phase 4: Maneuver Planning
- Delta-V 기동 계획 서비스
- Hohmann Transfer 계산
- Lambert Solver (랑데부용)
- Station-keeping 기동

### Phase 5: RViz2 GUI
- NAV2 스타일 제어 패널
- 3D 지구 + 위성 + 궤도 시각화

### Phase 6: Gazebo & MoveIt2 Integration
- Chaser/Target 위성 URDF
- LVLH 프레임 TF 브로드캐스터
- MoveIt2 설정 패키지
- 랑데부 및 포획 시뮬레이션

### Phase 7: Export & Polish
- CSV/CCSDS OEM 내보내기
- 성능 최적화 (100+ 위성)
- 문서화

---

## Technical Specifications

### Numerical Integrators
```cpp
enum class IntegratorType {
    RK4,        // 4th order Runge-Kutta (fast)
    RKF45,      // 4-5 adaptive (default)
    RKF78,      // 7-8 adaptive (high precision)
    ABM         // Adams-Bashforth-Moulton (efficient for long propagations)
};
```

### Perturbation Models
| Model | Source | Accuracy |
|-------|--------|----------|
| Gravity | GGM05C (n=360) | < 1 cm |
| Drag | NRLMSISE-00 | ~10-20% |
| SRP | Cannonball + Shadow | ~5% |
| Third Body | DE430 / Analytical | < 1 m |
| Tides | IERS 2010 | < 10 cm |

### Performance Targets
| Metric | Target |
|--------|--------|
| Satellites | 100+ real-time |
| Update Rate | 10-100 Hz |
| Latency | < 100 ms |
| Gravity calls/sec | 50,000+ |

---

## Configuration Example

### satellites.yaml
```yaml
satellites:
  - norad_id: 25544
    name: "ISS (ZARYA)"
    mass: 420000.0
    cross_section: 1500.0
    drag_coefficient: 2.2
    reflectivity: 1.3

  - norad_id: 48274
    name: "STARLINK-2408"
    mass: 260.0
    cross_section: 22.0
    drag_coefficient: 2.2
    reflectivity: 1.5

propagator:
  integrator: RKF78
  step_size: 60.0
  gravity_degree: 70
  perturbations:
    drag: true
    srp: true
    third_body: true
    solid_tides: false
```

---

## Quick Start

```bash
# Build
cd ~/ros2_ws
colcon build --packages-up-to hpop_bringup

# Launch HPOP System
ros2 launch hpop_bringup hpop_system.launch.py

# Launch RViz2 with HPOP panel
ros2 launch hpop_bringup visualization.launch.py

# Add satellite by NORAD ID
ros2 service call /hpop/fetch_tle hpop_msgs/srv/FetchTLE "{norad_ids: [25544, 48274]}"

# Start propagation
ros2 service call /hpop/start_propagation std_srvs/srv/Trigger
```

---

## GGM Gravity Plugin (Gazebo)

### Background

Gazebo의 내부 물리 엔진을 수정하지 않고, 외부에서 계산한 중력 가속도를 힘(force) 형태로 주입하는 방식으로 동작합니다.

### Spherical Harmonics Gravity Model

중력 퍼텐셜은 구면조화 전개를 사용하여 계산됩니다. Fully-Normalized Associated Legendre Functions를 사용하여 수치적 안정성을 보장합니다.

### Coordinate Transformation

구면좌표계 가속도 성분을 직교좌표계로 변환:

```
ax = ar*cos(φ)*cos(λ) - aφ*sin(φ)*cos(λ) - aλ*sin(λ)
ay = ar*cos(φ)*sin(λ) - aφ*sin(φ)*sin(λ) + aλ*cos(λ)
az = ar*sin(φ) + aφ*cos(φ)
```

### Gazebo API Application

```cpp
link->AddForce(ignition::math::Vector3d(Fx, Fy, Fz));
```

---

## References

- [GGM05C Gravity Model](https://www.csr.utexas.edu/grace/gravity/)
- [Celestrak TLE Data](https://celestrak.org/)
- [Vallado - Fundamentals of Astrodynamics](https://arc.aiaa.org/doi/book/10.2514/4.866647)
- [NAV2 RViz Plugins](https://github.com/ros-navigation/navigation2)
- [MoveIt2 Documentation](https://moveit.ros.org/)
- Montenbruck & Gill, Satellite Orbits

---

## License

Apache-2.0

## Author

seongmin (roboticsmaster@naver.com)
