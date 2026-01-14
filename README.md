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


  PPT 슬라이드용 문장                                                                                                                                                                                       
                                                                                                                                                                                                            
  슬라이드 1: GGM05C 중력장 모델 소개                                                                                                                                                                       
                                                                                                                                                                                                            
  제목: GGM05C 고정밀 지구 중력장 모델                                                                                                                                                                      
                                                                                                                                                                                                            
  본문:                                                                                                                                                                                                     
  - GGM05C: NASA/GFZ에서 2016년 발표한 Combined Gravity Model                                                                                                                                               
  - 구면조화함수 전개 차수 nmax=70, 총 2,556개 계수 사용                                                                                                                                                    
  - GRACE 위성 데이터 기반 고정밀 중력장 표현                                                                                                                                                               
  - 저궤도(LEO) 위성의 정밀 궤도 전파에 필수적                                                                                                                                                              
                                                                                                                                                                                                            
  ---                                                                                                                                                                                                       
  슬라이드 2: 고도별 중력 변화                                                                                                                                                                              
                                                                                                                                                                                                            
  제목: 고도에 따른 중력 가속도 변화                                                                                                                                                                        
                                                                                                                                                                                                            
  본문:                                                                                                                                                                                                     
  - 역제곱 법칙에 따라 고도 증가 시 중력 감소                                                                                                                                                               
  - 200km 고도: 9.22 m/s² → 2000km 고도: 5.68 m/s²                                                                                                                                                          
  - 점질량 모델 대비 GGM05C는 지구 비구면성 반영                                                                                                                                                            
  - 적도 400km에서 중력 이상: +562 mGal                                                                                                                                                                     
                                                                                                                                                                                                            
  핵심 메시지: 저궤도에서 비구면 중력 효과가 유의미하며, 정밀 궤도 결정에 고차 중력장 모델이 필요함                                                                                                         
                                                                                                                                                                                                            
  ---                                                                                                                                                                                                       
  슬라이드 3: 위도별 중력 변화 (J2 효과)                                                                                                                                                                    
                                                                                                                                                                                                            
  제목: 위도에 따른 중력 이상 분포                                                                                                                                                                          
                                                                                                                                                                                                            
  본문:                                                                                                                                                                                                     
  - 지구 적도 부풀음(Oblateness)으로 인한 중력 편차 발생                                                                                                                                                    
  - 적도: +561 mGal (중력 증가)                                                                                                                                                                             
  - 극점: -1,110 mGal (중력 감소)                                                                                                                                                                           
  - 적도-극점 총 차이: 1,672 mGal (~0.02%)                                                                                                                                                                  
  - J2 섭동항이 전체 이상의 95% 이상 설명                                                                                                                                                                   
                                                                                                                                                                                                            
  핵심 메시지: J2 항이 지배적이며, ISS와 같은 경사 궤도에서 주기적 중력 변동 발생                                                                                                                           
                                                                                                                                                                                                            
  ---                                                                                                                                                                                                       
  슬라이드 4: 전역 중력 이상 분포                                                                                                                                                                           
                                                                                                                                                                                                            
  제목: 400km 고도에서의 전지구 중력장 분포                                                                                                                                                                 
                                                                                                                                                                                                            
  본문:                                                                                                                                                                                                     
  - 적도 지역: 양의 중력 이상 (빨간색)                                                                                                                                                                      
  - 극지방: 음의 중력 이상 (파란색)                                                                                                                                                                         
  - 경도 방향 변화는 미미 → 축대칭 J2 효과 지배적                                                                                                                                                           
  - 고차 tesseral 조화항은 지역적 미세 변동 유발                                                                                                                                                            
                                                                                                                                                                                                            
  핵심 메시지: GGM05C(nmax=70)는 지구 중력장의 주요 공간적 특성을 충분히 표현                                                                                                                               
                                                                                                                                                                                                            
  ---                                                                                                                                                                                                       
  슬라이드 5: ISS 궤도에서의 중력 변동                                                                                                                                                                      
                                                                                                                                                                                                            
  제목: LEO 궤도(ISS, 51.6°)에서의 중력 경험                                                                                                                                                                
                                                                                                                                                                                                            
  본문:                                                                                                                                                                                                     
  - 궤도 고도: 420km, 경사각: 51.6°                                                                                                                                                                         
  - 1 궤도 동안 중력 변동 범위: 8.65 ~ 8.68 m/s²                                                                                                                                                            
  - 중력 이상 변동: -800 ~ +550 mGal (총 1,350 mGal)                                                                                                                                                        
  - 적도 통과 시 최대, 최고위도 도달 시 최소                                                                                                                                                                
  - 궤도당 2회 진동 (적도 2회 통과)                                                                                                                                                                         
                                                                                                                                                                                                            
  핵심 메시지: 위성은 매 궤도마다 수백 mGal의 중력 변동을 경험하며, 이는 궤도 섭동의 주요 원인                                                                                                              
                                                                                                                                                                                                            
  ---                                                                                                                                                                                                       
  슬라이드 6: J2-only vs GGM05C 비교                                                                                                                                                                        
                                                                                                                                                                                                            
  제목: 단순화 모델과 고정밀 모델 비교                                                                                                                                                                      
                                                                                                                                                                                                            
  본문:                                                                                                                                                                                                     
  - J2-only 해석해: Δg ≈ (3/2)J₂(Rₑ/r)²(GM/r²)(3sin²φ-1)                                                                                                                                                    
  - J2 항만으로 중력 이상의 ~95% 설명 가능                                                                                                                                                                  
  - GGM05C는 J3~J70 + tesseral 항 포함 → 나머지 5% 보정                                                                                                                                                     
  - 고차항 기여: 수십 mGal 수준                                                                                                                                                                             
                                                                                                                                                                                                            
  Trade-off:                                                                                                                                                                                                
  ┌───────────────┬────────┬─────────────┐                                                                                                                                                                  
  │     모델      │ 정확도 │  계산 비용  │                                                                                                                                                                  
  ├───────────────┼────────┼─────────────┤                                                                                                                                                                  
  │ J2-only       │ ~95%   │ 매우 빠름   │                                                                                                                                                                  
  ├───────────────┼────────┼─────────────┤                                                                                                                                                                  
  │ GGM05C (n=70) │ ~99.9% │ ~50 μs/call │                                                                                                                                                                  
  └───────────────┴────────┴─────────────┘                                                                                                                                                                  
  핵심 메시지: 실시간 시뮬레이션에는 J2로 충분하나, 고정밀 궤도 결정에는 GGM05C 필요                                                                                                                        
                                                                                                                                                                                                            
  ---                                                                                                                                                                                                       
  슬라이드 7: 핵심 수치 요약                                                                                                                                                                                
                                                                                                                                                                                                            
  제목: GGM05C 중력장 분석 핵심 결과                                                                                                                                                                        
  ┌─────────────────┬────────────────────────┐                                                                                                                                                              
  │      항목       │           값           │                                                                                                                                                              
  ├─────────────────┼────────────────────────┤                                                                                                                                                              
  │ 모델 차수       │ nmax = 70 (2,556 계수) │                                                                                                                                                              
  ├─────────────────┼────────────────────────┤                                                                                                                                                              
  │ 400km 적도 중력 │ 8.6816 m/s²            │                                                                                                                                                              
  ├─────────────────┼────────────────────────┤                                                                                                                                                              
  │ 400km 극점 중력 │ 8.6648 m/s²            │                                                                                                                                                              
  ├─────────────────┼────────────────────────┤                                                                                                                                                              
  │ 적도-극점 차이  │ 1,672 mGal             │                                                                                                                                                              
  ├─────────────────┼────────────────────────┤                                                                                                                                                              
  │ ISS 궤도 변동폭 │ ~1,350 mGal/orbit      │                                                                                                                                                              
  ├─────────────────┼────────────────────────┤                                                                                                                                                              
  │ J2 기여도       │ ~95%                   │                                                                                                                                                              
  ├─────────────────┼────────────────────────┤                                                                                                                                                              
  │ 계산 속도       │ 50 μs/call             │                                                                                                                                                              
  └─────────────────┴────────────────────────┘                                                                                                                                                              
  ---                                                                                                                                                                                                       
  슬라이드 8: 결론                                                                                                                                                                                          
                                                                                                                                                                                                            
  제목: GGM05C 중력장 모델의 의의                                                                                                                                                                           
                                                                                                                                                                                                            
  본문:                                                                                                                                                                                                     
  1. 저궤도 위성 시뮬레이션에 고정밀 중력장 모델 적용 필수                                                                                                                                                  
  2. J2 섭동이 지배적이나, 고정밀 궤도 결정에는 고차항 필요                                                                                                                                                 
  3. GGM05C(n=70)는 정확도와 계산 효율의 최적 균형점                                                                                                                                                        
  4. 본 연구의 HPOP 시스템에 GGM05C 통합으로 km급 궤도 정확도 달성                                                                                                                                          
                                                                                                                                                                                                            
  향후 연구:                                                                                                                                                                                                
  - 대기 항력, 태양복사압 등 비중력 섭동 추가                                                                                                                                                               
  - 실시간 궤도 결정 알고리즘 개발 
