# ROS2-Based LEO Satellite & Robotic Arm Integrated Simulation

## English Version

### 1. Executive Summary

This project builds a high-fidelity simulation platform integrating ROS2 and Gazebo for LEO satellite dynamics and a satellite-mounted robotic arm. It includes real-time orbit propagation using HPOP (or optional SGP4), satellite attitude and manipulator dynamics, coupled dynamics, and environmental physics including position-dependent gravity using GGM05C, atmospheric drag, and solar radiation.

### 2. Background & Motivation

* Robotic arms are essential for space services (repair, assembly, cargo handling).
* Ground verification requires combined orbit propagation and manipulator dynamics.
* Gazebo alone cannot simulate high-precision orbital motion or LEO gravity accurately.

### 3. Objectives

1. HPOP-based real-time orbit propagator ROS2 node
2. Orbit-to-Gazebo bridge for model updates
3. Satellite + 6-DoF manipulator URDF/SDF modeling
4. Coupled dynamics simulation (arm torque → satellite attitude)
5. Verification scenarios: approach, grasp, manipulation

### 4. Scope & Assumptions

* ROS2 Humble/Iron
* Gazebo Fortress/Garden
* HPOP numerical integration with dt-based publishing
* Optional SGP4 for rapid prototyping
* Development PC: quad-core+, 16GB RAM
* dt: 0.1–1.0 s recommended

### 5. System Architecture

**Components:**

1. Orbit Propagator Node (HPOP) → publishes `/satellite/orbit_state`
2. Orbit-to-Gazebo Bridge → ECI → Gazebo conversion
3. Gazebo World & Plugins → ModelPlugin, WorldPlugin
4. Satellite & Manipulator → URDF/SDF with 6-DoF arm
5. Control Stack → Arm + Satellite attitude controller
6. Visualization & Logging → RViz, Gazebo GUI, CSV/rosbag

**Data Flow:**

```
Initial state → HPOP → (r,v) → ECI→Gazebo → Plugin → Satellite Pose → Physics → Arm Controller → Joint Torques → Reaction → Next Step
```

### 6. Messages & Interfaces

* `/satellite/orbit_state` : `nav_msgs/Odometry`
* `/satellite/eci_state` : custom `{ header, r[3], v[3], epoch }`
* `/gazebo/set_model_state` : `gazebo_msgs/ModelState`
* Services: `/orbit/reset`, `/orbit/set_parameters`

### 7. Physics & Gravity Modeling

**Earth Gravity Potential (GGM05C):**

$$
V(r, \phi, \lambda) = \frac{GM}{r} \left[1 + \sum_{n=2}^{n_{max}} \left(\frac{a}{r}\right)^n \sum_{m=0}^{n} \bar{P}*{n,m}(\sin \phi)(\bar{C}*{n,m}\cos(m\lambda) + \bar{S}_{n,m}\sin(m\lambda))\right]
$$


**Fully-Normalized Associated Legendre Functions:**

$$
\bar{P}*{n,m}(x) = N*{n,m} P_{n,m}(x),\quad N_{n,m} = \sqrt{(2-\delta_{m0})(2n+1)\frac{(n-m)!}{(n+m)!}}
$$


**Gravitational Acceleration:**

$$
\mathbf{a} = -\nabla V
$$

Components:

$$
a_r = -\frac{\partial V}{\partial r},\quad a_\phi = -\frac{1}{r} \frac{\partial V}{\partial \phi},\quad a_\lambda = -\frac{1}{r\cos\phi} \frac{\partial V}{\partial \lambda}
$$


**Spherical → Cartesian:**

$$
\begin{aligned}
a_x &= a_r \cos\phi \cos\lambda - a_\phi \sin\phi \cos\lambda - a_\lambda \sin\lambda \
a_y &= a_r \cos\phi \sin\lambda - a_\phi \sin\phi \sin\lambda + a_\lambda \cos\lambda \
a_z &= a_r \sin\phi + a_\phi \cos\phi
\end{aligned}
$$


**Force in Gazebo:**

$$
\mathbf{F} = m \mathbf{a}
$$


**Coupled Satellite-Arm Dynamics:** Arm motion induces torque on satellite. Use HPOP for CoM orbit, Gazebo for attitude + arm, ModelPlugin for reaction.

### 8. Synchronization

* HPOP publish: 10 Hz (dt=0.1s)
* Gazebo update: faster than HPOP
* Frames: ECI → Gazebo world (or ENU)

### 9. Development Environment

* Ubuntu 22.04
* ROS2 Humble/Iron
* Gazebo Fortress/Garden
* MoveIt2
* C++ (plugins), Python (nodes)

### 10. Build & Run

```bash
colcon build --packages-select satellite_description satellite_plugins satellite_control satellite_moveit satellite_utils gazebo_leo_gravity
source install/setup.bash
ros2 launch satellite_sim satellite_sim.launch.py
```

---

## 한글 버전

### 1. 요약

본 프로젝트는 ROS2 + Gazebo 환경에서 LEO 위성의 궤도와 로봇팔 동역학을 통합하는 고정밀 시뮬레이션 플랫폼을 구축하는 것을 목표로 합니다.

### 2. 배경 및 필요성

* 위성 로봇팔은 수리, 조립, 물자 이송 등 우주서비스 핵심
* 지상 검증에는 궤도 + 로봇팔 동역학 통합 시뮬레이션 필요
* Gazebo 단독은 고정밀 궤도와 LEO 중력 시뮬레이션에 한계

### 3. 목표

1. HPOP 기반 실시간 궤도 ROS2 노드
2. Orbit-to-Gazebo 브리지 구현
3. 위성 + 6-DoF 로봇팔 URDF/SDF 모델링
4. 팔 토크 → 위성 자세 반작용 시뮬레이션
5. 검증 시나리오: 접근, 조작, 잡기

### 4. 범위 및 가정

* ROS2 Humble 이상
* Gazebo Fortress/Garden
* HPOP dt 단위 결과 publish
* 선택적 SGP4 프로토타이핑
* 개발 PC: 쿼드코어+, 16GB RAM
* dt: 0.1–1s

### 5. 시스템 아키텍처

**구성요소:**

1. Orbit Propagator Node → `/satellite/orbit_state` publish
2. Orbit-to-Gazebo Bridge → ECI → Gazebo
3. Gazebo World & Plugins → ModelPlugin, WorldPlugin
4. Satellite & Manipulator → URDF/SDF, 6-DoF arm
5. Control Stack → 팔 + 위성 자세 제어
6. Visualization & Logging → RViz, Gazebo GUI, CSV/rosbag

**데이터 흐름:**

```
초기 상태 → HPOP → (r,v) → ECI→Gazebo → Plugin → 위성 pose → 물리엔진 → 팔 제어 → joint torque → Plugin 반작용 → 다음 스텝
```

### 6. 메시지/인터페이스

* `/satellite/orbit_state` : `nav_msgs/Odometry`
* `/satellite/eci_state` : custom `{ header, r[3], v[3], epoch }`
* `/gazebo/set_model_state` : `gazebo_msgs/ModelState`
* 서비스: `/orbit/reset`, `/orbit/set_parameters`

### 7. 물리 모델링

**지구 중력 퍼텐셜 (GGM05C):**

$$
V(r, \phi, \lambda) = \frac{GM}{r} [1 + \sum_{n=2}^{n_{max}} (\frac{a}{r})^n \sum_{m=0}^{n} \bar{P}*{n,m}(\sin \phi)(\bar{C}*{n,m}\cos(m\lambda) + \bar{S}_{n,m}\sin(m\lambda))]
$$


**완전정규화 레젠드르 함수:**

$$
\bar{P}*{n,m}(x) = N*{n,m} P_{n,m}(x),\quad N_{n,m} = \sqrt{(2-\delta_{m0})(2n+1)\frac{(n-m)!}{(n+m)!}}
$$


**중력 가속도:**

$$
\mathbf{a} = -\nabla V,\quad a_r = -\frac{\partial V}{\partial r}, a_\phi = -\frac{1}{r}\frac{\partial V}{\partial \phi}, a_\lambda = -\frac{1}{r\cos\phi}\frac{\partial V}{\partial \lambda}
$$


**구면 → Cartesian 변환:**

$$
\begin{aligned} a_x &= a_r \cos\phi \cos\lambda - a_\phi \sin\phi \cos\lambda - a_\lambda \sin\lambda \
a_y &= a_r \cos\phi \sin\lambda - a_\phi \sin\phi \sin\lambda + a_\lambda \cos\lambda \
a_z &= a_r \sin\phi + a_\phi \cos\phi \end{aligned}
$$


**Gazebo 힘 적용:**
[
\mathbf{F} = m \mathbf{a}
]

**팔-위성 결합 동역학:** 팔 운동 → 위성 토크 반작용. HPOP: CoM, Gazebo: 자세+팔, Plugin: 반작용 계산.

### 8. 동기화

* HPOP publish: 10 Hz (dt=0.1s)
* Gazebo update: HPOP보다 빠름
* 좌표계: ECI → Gazebo world (or ENU)

### 9. 개발 환경

* Ubuntu 22.04
* ROS2 Humble/Iron
* Gazebo Fortress/Garden
* MoveIt2
* C++ (Plugin), Python (Node)

### 10. 빌드 & 실행

```bash
colcon build --packages-select satellite_description satellite_plugins satellite_control satellite_moveit satellite_utils gazebo_leo_gravity
source install/setup.bash
ros2 launch satellite_sim satellite_sim.launch.py
```

