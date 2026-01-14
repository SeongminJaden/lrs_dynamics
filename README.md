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

제목: Gravitational Acceleration vs Altitude at Equator                                                                                                                                                   
                                                                                                                                                                                                            
  그림 (a) 설명:                                                                                                                                                                                            
  - 적도 상공 200km~2000km 고도에서의 중력 가속도 비교                                                                                                                                                      
  - 파란 실선: GGM05C 중력장 모델 (nmax=70)                                                                                                                                                                 
  - 빨간 점선: 점질량(Point Mass) 근사 모델                                                                                                                                                                 
  - 고도 증가에 따라 역제곱 법칙으로 중력 감소                                                                                                                                                              
    - 200km: 9.22 m/s²                                                                                                                                                                                      
    - 400km: 8.68 m/s²                                                                                                                                                                                      
    - 1000km: 7.33 m/s²                                                                                                                                                                                     
    - 2000km: 5.68 m/s²                                                                                                                                                                                     
                                                                                                                                                                                                            
  그림 (b) 설명:                                                                                                                                                                                            
  - GGM05C와 점질량 모델의 차이 (중력 이상)                                                                                                                                                                 
  - 단위: mGal (1 mGal = 10⁻⁵ m/s²)                                                                                                                                                                         
  - 저고도일수록 중력 이상이 큼                                                                                                                                                                             
    - 200km: +633 mGal                                                                                                                                                                                      
    - 400km: +562 mGal                                                                                                                                                                                      
    - 1000km: +400 mGal                                                                                                                                                                                     
  - 지구 비구면성(J2)에 의한 적도 부풀음 효과                                                                                                                                                               
                                                                                                                                                                                                            
  핵심 포인트:                                                                                                                                                                                              
  "저궤도(LEO)에서는 점질량 근사로 인한 오차가 수백 mGal에 달하며, 이는 정밀 궤도 전파 시 수 km의 위치 오차로 누적될 수 있다."                                                                              
                                                                                                                                                                                                            
  ---                                                                                                                                                                                                       
  그림 2: ggm05c_gravity_vs_latitude                                                                                                                                                                        
                                                                                                                                                                                                            
  슬라이드: 위도별 중력 변화와 J2 효과                                                                                                                                                                      
                                                                                                                                                                                                            
  제목: Latitude-dependent Gravity Variation at 400km Altitude                                                                                                                                              
                                                                                                                                                                                                            
  그림 (a) 설명:                                                                                                                                                                                            
  - 400km 고도에서 위도 -90°~+90° 범위의 중력 분포                                                                                                                                                          
  - GGM05C: 위도에 따른 중력 변화 표현                                                                                                                                                                      
  - 점질량 모델: 위도 무관하게 일정 (8.676 m/s²)                                                                                                                                                            
  - 적도에서 중력 최대, 극점에서 중력 최소                                                                                                                                                                  
                                                                                                                                                                                                            
  그림 (b) 설명:                                                                                                                                                                                            
  - 위도별 중력 이상 분포 (J2 효과)                                                                                                                                                                         
  - 적도 (위도 0°): +561 mGal (양의 이상)                                                                                                                                                                   
  - 극점 (위도 ±90°): -1,110 mGal (음의 이상)                                                                                                                                                               
  - 총 변동폭: 1,672 mGal                                                                                                                                                                                   
                                                                                                                                                                                                            
  물리적 해석:                                                                                                                                                                                              
  - 지구는 완전한 구가 아닌 편평 타원체 (Oblate Spheroid)                                                                                                                                                   
  - 적도 반경이 극반경보다 약 21km 더 큼                                                                                                                                                                    
  - 적도에서는 지표면이 더 멀지만, 질량 분포로 인해 중력 증가                                                                                                                                               
  - 극점에서는 질량 집중이 적어 중력 감소                                                                                                                                                                   
                                                                                                                                                                                                            
  핵심 포인트:                                                                                                                                                                                              
  "J2 섭동항은 위도에 따른 중력 변화의 95% 이상을 설명하며, 궤도 경사각이 큰 위성일수록 주기적인 중력 변동을 크게 경험한다."                                                                                
                                                                                                                                                                                                            
  ---                                                                                                                                                                                                       
  그림 3: ggm05c_global_anomaly_map                                                                                                                                                                         
                                                                                                                                                                                                            
  슬라이드: 전지구 중력 이상 분포도                                                                                                                                                                         
                                                                                                                                                                                                            
  제목: Global Gravity Anomaly Distribution at 400km Altitude                                                                                                                                               
                                                                                                                                                                                                            
  그림 설명:                                                                                                                                                                                                
  - 400km 고도에서의 전지구 중력 이상 분포                                                                                                                                                                  
  - X축: 경도 (-180° ~ +180°)                                                                                                                                                                               
  - Y축: 위도 (-90° ~ +90°)                                                                                                                                                                                 
  - 컬러바: 중력 이상 (mGal)                                                                                                                                                                                
    - 빨간색: 양의 이상 (적도 부근)                                                                                                                                                                         
    - 파란색: 음의 이상 (극지방)                                                                                                                                                                            
                                                                                                                                                                                                            
  관찰 결과:                                                                                                                                                                                                
  - 위도 의존성 뚜렷: 적도→극점으로 갈수록 중력 이상 감소                                                                                                                                                   
  - 경도 의존성 미미: 동서 방향 변화 거의 없음                                                                                                                                                              
  - 이는 J2 (zonal harmonic)이 지배적임을 의미                                                                                                                                                              
  - 고차 tesseral 조화항은 미세한 지역적 패턴 유발                                                                                                                                                          
                                                                                                                                                                                                            
  GGM05C 모델 특성:                                                                                                                                                                                         
  - nmax=70으로 전지구 중력장의 주요 특성 포착                                                                                                                                                              
  - 공간 해상도: 약 300km (반파장)                                                                                                                                                                          
  - 더 높은 해상도 필요 시 nmax 증가 필요                                                                                                                                                                   
                                                                                                                                                                                                            
  핵심 포인트:                                                                                                                                                                                              
  "지구 중력장은 축대칭(zonal) 성분이 지배적이며, GGM05C(n=70)는 위성 궤도 전파에 필요한 정확도를 제공한다."                                                                                                
                                                                                                                                                                                                            
  ---                                                                                                                                                                                                       
  그림 4: ggm05c_leo_orbit_gravity                                                                                                                                                                          
                                                                                                                                                                                                            
  슬라이드: ISS 궤도에서의 중력 변동 분석                                                                                                                                                                   
                                                                                                                                                                                                            
  제목: Gravity Variation Along ISS-like LEO Orbit (420km, 51.6° inc)                                                                                                                                       
                                                                                                                                                                                                            
  그림 (a) - 지상 궤적 (Ground Track):                                                                                                                                                                      
  - ISS와 동일한 궤도 파라미터: 고도 420km, 경사각 51.6°                                                                                                                                                    
  - 색상: 각 위치에서의 중력 이상 값                                                                                                                                                                        
  - 적도 통과 시 빨간색(양), 고위도에서 파란색(음)                                                                                                                                                          
                                                                                                                                                                                                            
  그림 (b) - 궤도 위치별 중력:                                                                                                                                                                              
  - X축: 궤도 위치 (0°~360°, 1주기)                                                                                                                                                                         
  - Y축: 총 중력 가속도 (m/s²)                                                                                                                                                                              
  - 변동 범위: 8.65 ~ 8.68 m/s²                                                                                                                                                                             
  - 궤도당 2회 진동 (적도 2회 통과)                                                                                                                                                                         
                                                                                                                                                                                                            
  그림 (c) - 궤도 위치별 중력 이상:                                                                                                                                                                         
  - 중력 이상 변동 범위: -800 ~ +550 mGal                                                                                                                                                                   
  - 총 변동폭: 약 1,350 mGal                                                                                                                                                                                
  - 적도 통과 시 최대, 최고위도(±51.6°) 도달 시 최소                                                                                                                                                        
                                                                                                                                                                                                            
  그림 (d) - 위도 vs 중력 이상:                                                                                                                                                                             
  - 위도와 중력 이상의 2차 함수 관계                                                                                                                                                                        
  - J2의 르장드르 다항식 P₂(sin φ) = (3sin²φ - 1)/2 특성 반영                                                                                                                                               
  - 색상: 궤도 위치 (시간 순서)                                                                                                                                                                             
                                                                                                                                                                                                            
  핵심 포인트:                                                                                                                                                                                              
  "ISS 궤도의 위성은 매 궤도(~92분)마다 약 1,350 mGal의 중력 변동을 경험하며, 이는 장기간 누적 시 궤도 요소에 유의미한 섭동을 유발한다."                                                                    
                                                                                                                                                                                                            
  ---                                                                                                                                                                                                       
  그림 5: ggm05c_j2_comparison                                                                                                                                                                              
                                                                                                                                                                                                            
  슬라이드: GGM05C vs J2-only 모델 비교                                                                                                                                                                     
                                                                                                                                                                                                            
  제목: Comparison of GGM05C (n=70) and J2-only Analytical Model                                                                                                                                            
                                                                                                                                                                                                            
  그림 설명:                                                                                                                                                                                                
  - 파란 실선: GGM05C (nmax=70, 2,556 계수)                                                                                                                                                                 
  - 빨간 점선: J2-only 해석해                                                                                                                                                                               
  - 연녹색 영역: 고차항 기여분 (J3~J70 + tesseral)                                                                                                                                                          
                                                                                                                                                                                                            
  J2-only 해석 공식:                                                                                                                                                                                        
  Δg ≈ (3/2) × J₂ × (Rₑ/r)² × (GM/r²) × (3sin²φ - 1)                                                                                                                                                        
  - J₂ = 1.08263 × 10⁻³ (지구 동적 형상 계수)                                                                                                                                                               
  - 단일 계수로 중력 이상의 대부분 설명 가능                                                                                                                                                                
                                                                                                                                                                                                            
  비교 결과:                                                                                                                                                                                                
  ┌────────────────┬─────────────┬───────────────┐                                                                                                                                                          
  │      항목      │   J2-only   │ GGM05C (n=70) │                                                                                                                                                          
  ├────────────────┼─────────────┼───────────────┤                                                                                                                                                          
  │ 계수 개수      │ 1개         │ 2,556개       │                                                                                                                                                          
  ├────────────────┼─────────────┼───────────────┤                                                                                                                                                          
  │ 정확도         │ ~95%        │ ~99.9%        │                                                                                                                                                          
  ├────────────────┼─────────────┼───────────────┤                                                                                                                                                          
  │ 계산 시간      │ < 1 μs      │ ~50 μs        │                                                                                                                                                          
  ├────────────────┼─────────────┼───────────────┤                                                                                                                                                          
  │ 적도-극점 차이 │ ~1,600 mGal │ 1,672 mGal    │                                                                                                                                                          
  └────────────────┴─────────────┴───────────────┘                                                                                                                                                          
  고차항 기여:                                                                                                                                                                                              
  - J3, J4, ..., J70: 수십 mGal 보정                                                                                                                                                                        
  - Tesseral 항 (Cnm, Snm): 지역적 변동 표현                                                                                                                                                                
  - 극점 부근에서 GGM05C와 J2-only 차이 최대                                                                                                                                                                
                                                                                                                                                                                                            
  핵심 포인트:                                                                                                                                                                                              
  "실시간 시뮬레이션에서는 J2-only 모델로 충분한 정확도를 얻을 수 있으나, cm급 정밀 궤도 결정(POD)에는 GGM05C 이상의 고차 모델이 필수적이다."                                                               
                                                                                                                                                                                                            
  ---                                                                                                                                                                                                       
  종합 요약 슬라이드                                                                                                                                                                                        
                                                                                                                                                                                                            
  슬라이드: GGM05C 중력장 분석 결론                                                                                                                                                                         
                                                                                                                                                                                                            
  주요 발견:                                                                                                                                                                                                
  ┌─────────────────┬────────────────────┐                                                                                                                                                                  
  │    분석 항목    │        결과        │                                                                                                                                                                  
  ├─────────────────┼────────────────────┤                                                                                                                                                                  
  │ 400km 적도 중력 │ 8.6816 m/s²        │                                                                                                                                                                  
  ├─────────────────┼────────────────────┤                                                                                                                                                                  
  │ 400km 극점 중력 │ 8.6648 m/s²        │                                                                                                                                                                  
  ├─────────────────┼────────────────────┤                                                                                                                                                                  
  │ 적도-극점 차이  │ 1,672 mGal (0.02%) │                                                                                                                                                                  
  ├─────────────────┼────────────────────┤                                                                                                                                                                  
  │ ISS 궤도 변동폭 │ 1,350 mGal/orbit   │                                                                                                                                                                  
  ├─────────────────┼────────────────────┤                                                                                                                                                                  
  │ J2 기여도       │ ~95%               │                                                                                                                                                                  
  ├─────────────────┼────────────────────┤                                                                                                                                                                  
  │ 고차항 기여     │ ~5% (수십 mGal)    │                                                                                                                                                                  
  └─────────────────┴────────────────────┘                                                                                                                                                                  
  결론:                                                                                                                                                                                                     
  1. 지구 중력장의 비구면성은 저궤도 위성 궤도에 유의미한 영향                                                                                                                                              
  2. J2 항이 중력 이상의 대부분을 설명 (축대칭 특성)                                                                                                                                                        
  3. **GGM05C (n=70)**는 정확도와 계산 효율의 최적 균형점                                                                                                                                                   
  4. 본 연구의 HPOP 시스템에 GGM05C 적용으로 고정밀 궤도 전파 구현                                                                                                                                                                              
